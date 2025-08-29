#include "MPController.hpp"

static double getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

ModelPredictiveController::ModelPredictiveController(
    std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller)
{
    fixed_delta_time_ = 0.2f;
    autonomousDrive_  = "SAE_0";
    speed_lock_       = false;
    xboxController_   = xbox_controller;
    current_speed_    = 0.0f;
    desired_speed_    = 0.0f;
    current_steering_ = 0.0f;

    lane_departure_threshold_ = 0.1f;

    session_ = session;

    publisher_ = std::make_unique<ControllerPublisher>(session_);

    coeffs_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Coeffs",
        [this](const zenoh::Sample& sample)
        {
            std::string coeffs_str = sample.get_payload().as_string();

            std::stringstream ss(coeffs_str);
            std::string token;
            parsed_coeffs_.clear();
            while (std::getline(ss, token, ','))
            {
                parsed_coeffs_.push_back(std::stod(token));
            }
        },
        zenoh::closures::none));

    activeAutonomyLevel_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/ActiveAutonomyLevel",
        [this](const zenoh::Sample& sample)
        {
            std::string activeAutonomyLevel = sample.get_payload().as_string();
            std::cout << "Active Autonomy Level: " << activeAutonomyLevel
                      << std::endl;
            setAutonomousDriveState(activeAutonomyLevel);
        },
        zenoh::closures::none));

    speed_lock_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed/Lock",
        [this](const zenoh::Sample& sample)
        {
            std::string value_str = sample.get_payload().as_string();

            bool lock_value = false;
            if (value_str.find("1") != std::string::npos)
            {
                lock_value = true;
            }

            speed_lock_ = lock_value;

            std::cout << "Speed lock "
                      << (lock_value ? "activated" : "deactivated")
                      << std::endl;
        },
        zenoh::closures::none));

    currentSpeed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed",
        [this](const zenoh::Sample& sample)
        {
            float speed    = std::stof(sample.get_payload().as_string());
            current_speed_ = speed * (M_PI * 0.067) / 60.0;
        },
        zenoh::closures::none));

    std::cout << "MPC controller created!" << std::endl;
}

ModelPredictiveController::~ModelPredictiveController() {}

void ModelPredictiveController::init(size_t horizon, double wheelbase,
                                     double Ts, const Eigen::Matrix4d& Q,
                                     const Eigen::Matrix2d& R,
                                     const Eigen::Matrix4d& Qf, int height,
                                     int width, double target_velocity,
                                     bool carla_mode)
{
    N_               = horizon;
    L_               = wheelbase;
    Ts_              = Ts;
    Q_               = Q;
    R_               = R;
    Qf_              = Qf;
    carlaMode_       = carla_mode;
    desired_speed_   = 0.0f;
    target_velocity_ = target_velocity;
    height_          = height;
    width_           = width;

    last_u_flat_.setConstant(2 * N_, 0.0);

    std::cout << "MPC initialized with horizon=" << N_ << ", wheelbase=" << L_
              << ", timestep=" << Ts_ << std::endl;
}

void ModelPredictiveController::solve(const Eigen::Vector4d& x0,
                                      const std::vector<double>& traj_coeffs)
{
    // static int solve_count = 0;
    // if (++solve_count >= 2) exit(0);

    // double time = getCurrentTime();
    // Conversion factors
    const double mx = 0.6 / (width_ * 0.6 / 0.6); // meters per pixel in x
    const double my = 0.6 / height_;                // meters per pixel in y
    // double time2 = getCurrentTime();
    // std::cout << "Time for conversion factors: " << (time2 - time) << " s" <<
    // std::endl; Convert pixel-space trajectory coeffs to meter-space
    std::vector<double> meter_coeffs(4);

    // const double image_width_px = 1024.0;
    // const double center_x_px = image_width_px / 2.0;
    // meter_coeffs[0] = mx * (traj_coeffs[0] - center_x_px);
    meter_coeffs[0] = mx * traj_coeffs[0];
    meter_coeffs[1] = mx * traj_coeffs[1] / my;
    meter_coeffs[2] = mx * traj_coeffs[2] / (my * my);
    meter_coeffs[3] = mx * traj_coeffs[3] / (my * my * my);
    std::cout << "Meter coefficients: "
              << meter_coeffs[0] << ", "
              << meter_coeffs[1] << ", "
              << meter_coeffs[2] << ", "
              << meter_coeffs[3] << std::endl;

    // Build reference trajectory in meter-space
    std::vector<Eigen::Vector4d> x_ref(N_ + 1);
    // const double v_init = x0(3);
    for (size_t k = 0; k <= N_; ++k)
    {
        // double v_ref = v_init + (target_velocity_ - v_init) * double(k) / N_;
        double v_ref   = target_velocity_;
        double y_ref   = x0(1) + k * v_ref * Ts_;
        double x_ref_m = meter_coeffs[0] + meter_coeffs[1] * y_ref +
                         meter_coeffs[2] * y_ref * y_ref +
                         meter_coeffs[3] * y_ref * y_ref * y_ref;
        double dx_dy = meter_coeffs[1] + 2 * meter_coeffs[2] * y_ref +
                       3 * meter_coeffs[3] * y_ref * y_ref;
        double psi_ref = std::atan(dx_dy);
        x_ref[k] << x_ref_m, y_ref, psi_ref, v_ref;
    }
    // time = getCurrentTime();
    // std::cout << "Time for reference trajectory: " << (time - time2) << " s"
    // << std::endl; std::cout << "Reference state at step " << N_ << ": "
    //           << x_ref[N_].transpose() << std::endl;

    // 3.1) warm‐start u_flat from last cycle:
    Eigen::VectorXd u_flat(2 * N_);
    for (size_t k = 0; k < N_ - 1; ++k)
    {
        u_flat(2 * k)     = last_u_flat_(2 * (k + 1));
        u_flat(2 * k + 1) = last_u_flat_(2 * (k + 1) + 1);
    }
    // u_flat(2*(N_-1))     = x0(3) + (target_velocity_ - x0(3));
    // u_flat(2*(N_-1) + 1) = 0.0;

    double speed_target = std::clamp(target_velocity_, 0.0, 2.0);
    u_flat(2 * (N_ - 1)) =
        std::clamp(x0(3) + 0.1 * (speed_target - x0(3)), 0.0, 2.0);
    u_flat(2 * (N_ - 1) + 1) = 0.0;
    // time2 = getCurrentTime();
    // std::cout << "Time for warm-start: " << (time2 - time) << " s" <<
    // std::endl;

    // lambda to compute cost for any u_try
    auto computeCost = [&](const Eigen::VectorXd& u_try)
    {
        std::vector<Eigen::Vector4d> x_seq(N_ + 1);
        x_seq[0] = x0;
        double J = 0.0;

        for (size_t k = 0; k < N_; ++k)
        {
            Eigen::Vector2d uk;
            uk << u_try(2 * k), u_try(2 * k + 1);
            x_seq[k + 1] = backwardEuler(x_seq[k], uk);

            Eigen::Vector4d dx = x_seq[k] - x_ref[k];
            J += dx.transpose() * Q_ * dx;
            J += uk.transpose() * R_ * uk;

            // add Δu cost:
            Eigen::Vector2d uk_prev =
                (k == 0) ? Eigen::Vector2d{x0(3), current_steering_}
                         : Eigen::Vector2d{u_try(2 * (k - 1)),
                                           u_try(2 * (k - 1) + 1)};

            // compute Δu
            Eigen::Vector2d du = uk - uk_prev;

            // compute speed‐adaptive steering smoothness weight:
            double v_k = x_seq[k][3]; // predicted speed at step k
            // std::cout << "v_k: " << v_k << std::endl;
            double speed_frac = std::clamp(v_k / 1 * 500, 0.0, 1500.0);
            // e.g. at v=0 → w_ddelta = base; at v=v_max → w_ddelta = 2*base
            double w_dv     = 1 + speed_frac; // keep throttle pretty free
            double w_ddelta = w_ddelta_base_ + speed_frac;
            // std::cout << "w_ddelta: " << w_ddelta << std::endl;

            // add the Δu cost:
            J += w_dv * du(0) * du(0) + w_ddelta * du(1) * du(1);
        }
        // terminal cost
        Eigen::Vector4d dxN = x_seq[N_] - x_ref[N_];
        J += dxN.transpose() * Qf_ * dxN;
        return J;
    };

    // 3.2) gradient‐descent with backtracking line-search
    const double tol   = 1e-4;
    const int max_iter = 20;
    double alpha0 = 0.1, beta = 0.5;
    Eigen::VectorXd grad(2 * N_);

    // initial cost
    double J_curr = computeCost(u_flat);

    for (int iter = 0; iter < max_iter; ++iter)
    {
        // finite-difference gradient
        // const double eps = 1e-2;
        // for (int i=0; i<2*(int)N_; ++i) {
        //     Eigen::VectorXd up = u_flat, um = u_flat;
        //     up(i) += eps;  um(i) -= eps;
        //     grad(i) = (computeCost(up) - computeCost(um)) / (2*eps);
        // }

        std::vector<Eigen::Vector4d> x_seq(N_ + 1);
        std::vector<Eigen::Vector2d> u_seq(N_);
        x_seq[0] = x0;
        for (size_t k = 0; k < N_; ++k)
        {
            u_seq[k]     = Eigen::Vector2d(u_flat(2 * k), u_flat(2 * k + 1));
            x_seq[k + 1] = backwardEuler(x_seq[k], u_seq[k]);
        }

        // Backward pass
        std::vector<Eigen::Vector4d> lambda(N_ + 1);
        lambda[N_] = Qf_ * (x_seq[N_] - x_ref[N_]);
        for (int k = N_ - 1; k >= 0; --k)
        {
            double v     = x_seq[k](3);
            double psi   = x_seq[k](2);
            double delta = u_seq[k](1);

            Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
            A(0, 0)           = 1.0;
            A(0, 2)           = Ts_ * v * std::cos(psi);
            A(0, 3)           = Ts_ * std::sin(psi);

            A(1, 1) = 1.0;
            A(1, 2) = -Ts_ * v * std::sin(psi);
            A(1, 3) = Ts_ * std::cos(psi);

            A(2, 2) = 1.0;
            A(2, 3) = Ts_ / L_ * std::tan(delta);

            Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();
            B(2, 1) = Ts_ * v / (L_ * std::cos(delta) * std::cos(delta));
            B(3, 0) = 1.0;

            lambda[k] =
                Q_ * (x_seq[k] - x_ref[k]) + A.transpose() * lambda[k + 1];
            grad.segment<2>(2 * k) =
                2.0 * R_ * u_seq[k] + B.transpose() * lambda[k + 1];

            Eigen::Vector2d uk = u_seq[k];
            Eigen::Vector2d uk_prev =
                (k == 0) ? Eigen::Vector2d{x0(3), current_steering_}
                         : u_seq[k - 1];
            Eigen::Vector2d du = uk - uk_prev;
            double v_k         = x_seq[k][3];
            double speed_frac  = std::clamp(v_k / 1 * 500, 0.0, 1500.0);
            speed_frac         = 0;
            double w_dv        = 0 + speed_frac;
            // double w_ddelta = w_ddelta_base_ + speed_frac;
            double w_ddelta = 0;

            // Gradient w.r.t. uk
            grad.segment<2>(2 * k) +=
                2.0 * Eigen::Vector2d(w_dv * du(0), w_ddelta * du(1));
            // Gradient w.r.t. uk_prev (if k > 0)
            if (k > 0)
            {
                grad.segment<2>(2 * (k - 1)) -=
                    2.0 * Eigen::Vector2d(w_dv * du(0), w_ddelta * du(1));
            }
        }

        if (grad.norm() < tol)
            break;

        // backtracking line-search
        double alpha = alpha0;
        Eigen::VectorXd u_next(2 * N_);
        double J_next;
        while (true)
        {
            u_next = u_flat - alpha * grad;
            // clamp
            for (size_t k = 0; k < N_; ++k)
            {
                u_next(2 * k) = std::clamp(u_next(2 * k), 0.0, 10.0);
                u_next(2 * k + 1) =
                    std::clamp(u_next(2 * k + 1), -0.7854, 0.7854);
            }
            J_next = computeCost(u_next);
            if (J_next < J_curr || alpha < 1e-6)
                break;
            alpha *= beta;
        }
        u_flat = u_next;
        J_curr = J_next;
    }
    // store for next warm‐start
    last_u_flat_ = u_flat;

    // final rollout for predicted trajectory
    std::vector<Eigen::Vector4d> x_seq(N_ + 1);
    x_seq[0] = x0;
    for (size_t k = 0; k < N_; ++k)
    {
        Eigen::Vector2d u_k;
        u_k << u_flat(2 * k), u_flat(2 * k + 1);
        x_seq[k + 1] = backwardEuler(x_seq[k], u_k);
    }
    predicted_trajectory_ = x_seq;
    // publish trajectory in pixel coordinates
    std::ostringstream oss;
    for (size_t i = 0; i < x_seq.size(); ++i)
    {
        double x_pix = x_seq[i](0) / mx;
        double y_pix = x_seq[i](1) / my;
        oss << x_pix << "," << y_pix;
        if (i + 1 < x_seq.size())
            oss << ";";
    }
    std::cout << "Publishing trajectory: " << oss.str() << std::endl;
    publisher_->publishMpcTrajectory(oss.str());

    // set outputs
    // if (carlaMode_) {
    desired_speed_ = u_flat(0);
    // } else {
    // desired_speed_ = u_flat(0) * 60.0 / (M_PI * 0.067); // convert from m/s
    // to rpm
    // }
    current_steering_ = u_flat(1);

    std::cout << "MPC control - " << "        Speed: " << desired_speed_
              << ",         Steering: " << current_steering_
              << "target Speed: " << target_velocity_ << std::endl;
    // time = getCurrentTime();
    // std::cout << "Time to publish: " << (time - time2) << " s" << std::endl;
    // summary
    // double total_error = 0.0;
    // for (size_t k = 0; k < N_; ++k) {
    //     Eigen::Vector4d dx = x_seq[k] - x_ref[k];
    //     total_error += dx.squaredNorm();
    // }
    // std::cout << "[MPC_SUMMARY] Q="
    //           << Q_(0,0) << "," << Q_(1,1) << ","
    //           << Q_(2,2) << "," << Q_(3,3)
    //           << " R=" << R_(0,0) << "," << R_(1,1)
    //           << " total_error=" << total_error << std::endl;
    // 3. Add debug output in solve():
    std::cout << "Initial state: " << x0.transpose() << std::endl;
    std::cout << "Target velocity: " << target_velocity_ << " m/s" << std::endl;
    std::cout << "Timestep: " << Ts_ << " s" << std::endl;
    std::cout << "Expected step distance: " << target_velocity_ * Ts_ << " m"
              << std::endl;
    std::cout << "Real distance: " << current_speed_ * Ts_ << " m" << std::endl;
}

// // Forward Euler discretization
Eigen::Vector4d
ModelPredictiveController::backwardEuler(const Eigen::Vector4d& x,
                                         const Eigen::Vector2d& u)
{
    // double v_next   = x(3) + Ts_ * u(0);
    // double psi_next = x(2) + Ts_ * (v_next / L_) * std::tan(u(1));
    // double Xf_next  = x(0) + Ts_ * v_next * std::cos(psi_next);
    // double Yf_next  = x(1) + Ts_ * v_next * std::sin(psi_next);

    double v   = x(3);
    double psi = x(2);

    double v_next   = u(0);
    double psi_next = psi + Ts_ * (v / L_) * std::tan(u(1));
    double Xf_next  = x(0) + Ts_ * v * std::sin(psi);
    double Yf_next  = x(1) + Ts_ * v * std::cos(psi);

    Eigen::Vector4d x_next;
    x_next << Xf_next, Yf_next, psi_next, v_next;

    return x_next;
}

void ModelPredictiveController::setAutonomousDriveState(
    std::string current_state)
{
    autonomousDrive_ = current_state;
}

std::string ModelPredictiveController::getAutonomousDriveState() const
{
    return autonomousDrive_;
}
// SAE_0
void ModelPredictiveController::manualControl()
{
    // double current_time   = getCurrentTime();
    float manual_steering = xboxController_->getManualSteering();
    // float manual_speed    = xboxController_->getManualSpeed();
    publisher_->publishSteering(manual_steering);

    double now = getCurrentTime();

    if (parsed_coeffs_.size() == 4)
    {
        Eigen::Vector4d x0;
        x0(0) = 0;
        x0(1) = 0;
        x0(2) = current_steering_;
        x0(3) = (current_speed_ > 0.01) ? current_speed_ : 0.1;

        this->solve(x0, parsed_coeffs_);
    }
    else
    {
        std::cerr << "Invalid number of coefficients: " << parsed_coeffs_.size()
                  << " (expected 4)" << std::endl;
    }
    std::cout << "SOLVE TIME: " << getCurrentTime() - now << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_1_LKAS
void ModelPredictiveController::LKASControl()
{
    // double current_time   = getCurrentTime();
    // float manual_steering = xboxController_->getManualSteering();
    // float manual_speed    = xboxController_->getManualSpeed();

    // if (std::abs(cameraError_) > lane_departure_threshold_ &&
    //     std::abs(cameraError_) < 1)
    // {
    //     float direction = manual_steering +(steeringPID(cameraError_,
    //     current_time) - manual_steering) * 0.5f;
    //     // publisher_->publishAlert("Lane Departure");
    //     publisher_->publishSteering(direction);
    // }
    // else
    // {
    //     publisher_->publishSteering(manual_steering);
    // }
    // if (!speed_lock_)
    //     publisher_->publishSpeed(manual_speed);
    // else
    //     publisher_->publishSpeed(
    //         speedPidController_->speedPID(0 - current_speed_, current_time));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_1_ACC
void ModelPredictiveController::adaptiveCruiseControl()
{
    // double current_time   = getCurrentTime();
    // float manual_steering = xboxController_->getManualSteering();
    // float manual_speed    = xboxController_->getManualSpeed();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_2
void ModelPredictiveController::partialControl()
{
    // double current_time   = getCurrentTime();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_3
void ModelPredictiveController::conditionalAutomation()
{
    // double current_time   = getCurrentTime();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_4
void ModelPredictiveController::autonomousControl()
{
    // double current_time   = getCurrentTime();
    if (parsed_coeffs_.size() == 4)
    {
        Eigen::Vector4d x0;
        x0(0) = 0;
        x0(1) = 0;
        x0(2) = current_steering_;
        x0(3) = (current_speed_ > 0.01) ? current_speed_ : 0.1;

        this->solve(x0, parsed_coeffs_);
    }
    else
    {
        std::cerr << "Invalid number of coefficients: " << parsed_coeffs_.size()
                  << " (expected 4)" << std::endl;
    }
    // double steering = current_steering_ *180 / M_PI + 90;
    double steering =
        std::clamp(current_steering_ * 180 * 3 / M_PI + 90, 0.0, 180.0);

    publisher_->publishSteering(steering);
    if (!this->speed_lock_)
    {
        publisher_->publishDesiredSpeed(desired_speed_);
        publisher_->publishCurrentGear(1);
    }
    else
    {
        publisher_->publishDesiredSpeed(0);
        publisher_->publishCurrentGear(0);
    }
    std::cout << "MPC control - " << "Current speed: " << current_speed_
              << ", Current Steering: " << steering << std::endl;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(fixed_delta_time_ * 1000)));
}

void ModelPredictiveController::run()
{
    while (true)
    {
        if (!xboxController_->getPidEnable())
        {
            std::string sae_level = getAutonomousDriveState();

            if (sae_level.find("SAE_0") != std::string::npos)
            {
                manualControl();
            }
            else if (sae_level.find("SAE_1_LKAS") != std::string::npos)
            {
                LKASControl();
            }
            else if (sae_level.find("SAE_1_ACC") != std::string::npos)
            {
                adaptiveCruiseControl();
            }
            else if (sae_level.find("SAE_2") != std::string::npos)
            {
                partialControl();
            }
            else if (sae_level.find("SAE_3") != std::string::npos)
            {
                conditionalAutomation();
            }
            else if (sae_level.find("SAE_4") != std::string::npos)
            {
                autonomousControl();
            }
            else
            {
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
