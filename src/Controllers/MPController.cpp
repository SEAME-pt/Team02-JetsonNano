#include "MPController.hpp"

static double getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

ModelPredictiveController::ModelPredictiveController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller)
{
    fixed_delta_time_   = 0.05f;
    autonomousDrive_    = "SAE_0";
    speed_lock_         = false;
    xboxController_     = xbox_controller;
    current_speed_      = 0.0f;
    desired_speed_      = 0.0f;
    current_steering_      = 0.0f;

    lane_departure_threshold_ = 0.1f;

    speedKp_ = 0.12f;
    speedKi_ = 1.3f;
    speedKd_ = 0.01f;
    speedPidController_ = new SpeedPidController();
    speedPidController_->init(speedKp_, speedKi_, speedKd_, fixed_delta_time_);
    
    session_ = session;
    
    publisher_ = std::make_unique<ControllerPublisher>(session_);
    
    coeffs_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Coeffs",
        [this](const zenoh::Sample& sample)
        {
            std::string coeffs_str = sample.get_payload().as_string();
            
            std::vector<double> parsed_coeffs;
            std::stringstream ss(coeffs_str);
            std::string token;
            
            while (std::getline(ss, token, ',')) {
                parsed_coeffs.push_back(std::stod(token));
            }
            
            if (parsed_coeffs.size() == 4) {
                Eigen::Vector4d x0;
                x0(0) = 0;
                x0(1) = 0;
                x0(2) = current_steering_;
                x0(3) = (current_speed_ > 0.01) ? current_speed_ : 0.1;
                
                
                // this->solve(x0, parsed_coeffs);

                
                // std::cout << "Received coefficients: " 
                //           << parsed_coeffs[0] << ", " 
                //           << parsed_coeffs[1] << ", " 
                //           << parsed_coeffs[2] << ", " 
                //           << parsed_coeffs[3] << std::endl;
            } else {
                std::cerr << "Invalid number of coefficients: " 
                          << parsed_coeffs.size() << " (expected 4)" << std::endl;
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
            if (value_str.find("1") != std::string::npos) {
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
            current_speed_ = speed;
        },
        zenoh::closures::none));
    std::cout << "MPC controller created!" << std::endl;
}

ModelPredictiveController::~ModelPredictiveController() {
    delete speedPidController_;
}

void ModelPredictiveController::init(size_t horizon, double wheelbase, double Ts,
                             const Eigen::Matrix4d& Q,
                             const Eigen::Matrix2d& R,
                             const Eigen::Matrix4d& Qf)
{
    N_ = horizon;
    L_ = wheelbase;
    Ts_ = Ts;
    Q_ = Q;
    R_ = R;
    Qf_ = Qf;
    desired_speed_ = 8.0;

    last_u_flat_.setConstant(2*N_, 0.0);
    
    std::cout << "MPC initialized with horizon=" << N_ 
              << ", wheelbase=" << L_ 
              << ", timestep=" << Ts_ << std::endl;
}



void ModelPredictiveController::solve(const Eigen::Vector4d& x0,
                                         const std::vector<double>& traj_coeffs)
{
    // static int solve_count = 0;
    // if (++solve_count >= 2) exit(0);

    // Conversion factors
    const double mx         = 6.0 / 1024.0;  // meters per pixel in x
    const double my         = 7.0 / 512.0;   // meters per pixel in y

    // Convert pixel-space trajectory coeffs to meter-space
    std::vector<double> meter_coeffs(4);

    // const double image_width_px = 1024.0;
    // const double center_x_px = image_width_px / 2.0;
    // meter_coeffs[0] = mx * (traj_coeffs[0] - center_x_px);
    meter_coeffs[0] = mx * traj_coeffs[0];
    meter_coeffs[1] = mx * traj_coeffs[1] / my;
    meter_coeffs[2] = mx * traj_coeffs[2] / (my * my);
    meter_coeffs[3] = mx * traj_coeffs[3] / (my * my * my);

    // Build reference trajectory in meter-space
    std::vector<Eigen::Vector4d> x_ref(N_ + 1);
    const double v_init = x0(3);
    for (size_t k = 0; k <= N_; ++k) {
        double v_ref = v_init + (target_velocity_ - v_init) * double(k) / N_;
        double y_ref = x0(1) + k * v_ref * Ts_;
        double x_ref_m = meter_coeffs[0]
                       + meter_coeffs[1] * y_ref
                       + meter_coeffs[2] * y_ref * y_ref
                       + meter_coeffs[3] * y_ref * y_ref * y_ref;
        double dx_dy = meter_coeffs[1] + 2 * meter_coeffs[2] * y_ref +
               3 * meter_coeffs[3] * y_ref * y_ref;
        double psi_ref = std::atan(dx_dy);
        x_ref[k] << x_ref_m, y_ref, psi_ref, v_ref;
    }
    // std::cout << "Reference state at step " << N_ << ": "
    //           << x_ref[N_].transpose() << std::endl;

    // 3.1) warm‐start u_flat from last cycle:
    Eigen::VectorXd u_flat(2*N_);
    // shift everything by one step
    for (size_t k = 0; k < N_-1; ++k) {
        u_flat(2*k)     = last_u_flat_(2*(k+1));
        u_flat(2*k + 1) = last_u_flat_(2*(k+1) + 1);
    }
    // append [v_target, 0]
    u_flat(2*(N_-1))     = x0(3) + (target_velocity_ - x0(3));  
    u_flat(2*(N_-1) + 1) = 0.0;

    // lambda to compute cost for any u_try
    auto computeCost = [&](const Eigen::VectorXd& u_try) {
        // rollout with backwardEuler and sum dxᵀQdx + uᵀRu
        std::vector<Eigen::Vector4d> x_seq(N_+1);
        x_seq[0] = x0;
        double J = 0.0;
        
        for (size_t k = 0; k < N_; ++k) {
            Eigen::Vector2d uk; uk << u_try(2*k), u_try(2*k+1);
            x_seq[k+1] = backwardEuler(x_seq[k], uk);

            Eigen::Vector4d dx = x_seq[k] - x_ref[k];
            J += dx.transpose()*Q_*dx;
            J += uk.transpose()*R_*uk;
            
            // add Δu cost:
            Eigen::Vector2d uk_prev = (k==0)
            ? Eigen::Vector2d{x0(3), current_steering_}
            : Eigen::Vector2d{u_try(2*(k-1)), u_try(2*(k-1)+1)};
            
            // compute Δu
            Eigen::Vector2d du = uk - uk_prev;
            
            // compute speed‐adaptive steering smoothness weight:
            double v_k       = x_seq[k][3];   // predicted speed at step k
            // std::cout << "v_k: " << v_k << std::endl;
            double speed_frac = std::clamp(v_k / 10 * 500, 0.0, 700.0);
            // e.g. at v=0 → w_ddelta = base; at v=v_max → w_ddelta = 2*base
            double w_dv     = 1 + speed_frac;   // keep throttle pretty free
            double w_ddelta  = w_ddelta_base_ + speed_frac;
            // std::cout << "w_ddelta: " << w_ddelta << std::endl;

            // add the Δu cost:
            J += w_dv     * du(0) * du(0)
                + w_ddelta * du(1) * du(1);
        }
        // terminal cost
        Eigen::Vector4d dxN = x_seq[N_] - x_ref[N_];
        J += dxN.transpose()*Qf_*dxN;
        return J;
    };

    // 3.2) gradient‐descent with backtracking line-search
    const double tol = 1e-4;
    const int    max_iter = 20;
    double alpha0 = 0.1, beta = 0.5;
    Eigen::VectorXd grad(2*N_);

    // initial cost
    double J_curr = computeCost(u_flat);

    for (int iter=0; iter<max_iter; ++iter) {
        // finite-difference gradient
        const double eps = 1e-3;
        for (int i=0; i<2*(int)N_; ++i) {
        Eigen::VectorXd up = u_flat, um = u_flat;
        up(i) += eps;  um(i) -= eps;
        grad(i) = (computeCost(up) - computeCost(um)) / (2*eps);
        }
        if (grad.norm()<tol) break;

        // backtracking line-search
        double alpha = alpha0;
        Eigen::VectorXd u_next(2*N_);
        double J_next;
        while (true) {
        u_next = u_flat - alpha * grad;
        // clamp
        for (size_t k=0; k<N_; ++k) {
            u_next(2*k)     = std::clamp(u_next(2*k),     0.0, 10.0);
            u_next(2*k+1)   = std::clamp(u_next(2*k+1),  -0.7854, 0.7854);
        }
        J_next = computeCost(u_next);
        if (J_next < J_curr || alpha < 1e-6) break;
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
    for (size_t k = 0; k < N_; ++k) {
        Eigen::Vector2d u_k;
        u_k << u_flat(2*k), u_flat(2*k+1);
        x_seq[k+1] = backwardEuler(x_seq[k], u_k);
    }
    predicted_trajectory_ = x_seq;

    // publish trajectory in pixel coordinates
    std::ostringstream oss;
    for (size_t i = 0; i < x_seq.size(); ++i) {
        double x_pix = x_seq[i](0) / mx;
        double y_pix = x_seq[i](1) / my;
        oss << x_pix << "," << y_pix;
        if (i + 1 < x_seq.size()) oss << ";";
    }
    publisher_->publishMpcTrajectory(oss.str());

    // set outputs
    desired_speed_     = u_flat(0);
    current_steering_  = u_flat(1);

    std::cout << "Speed: " << desired_speed_
              << ", Steering: " << current_steering_ << std::endl;

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


    double v = x(3);
    double psi = x(2);

    double v_next   = u(0);
    double psi_next = psi + Ts_ * (v / L_) * std::tan(u(1));
    double Xf_next  = x(0) + Ts_ * v * std::sin(psi);
    double Yf_next  = x(1) + Ts_ * v * std::cos(psi);

    Eigen::Vector4d x_next;
    x_next << Xf_next, Yf_next, psi_next, v_next;

    return x_next;
}

void ModelPredictiveController::setAutonomousDriveState(std::string current_state)
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
    double current_time   = getCurrentTime();
    float manual_steering = xboxController_->getManualSteering();
    float manual_speed    = xboxController_->getManualSpeed();
    publisher_->publishSteering(manual_steering);
    std::cout << "Manual control - "
              << "Current speed: " << current_speed_
              << ", Steering: " << manual_steering
              << ", Manual speed: " << manual_speed << std::endl;
    if (!speed_lock_)
        publisher_->publishSpeed(manual_speed);
    else
    {
        if (manual_speed <= 0)
        {
            publisher_->publishSpeed(manual_speed);
        }
        else
        {
            publisher_->publishSpeed(speedPidController_->speedPID(
                0 - current_speed_, current_time));
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    //     float direction = manual_steering +(steeringPID(cameraError_, current_time) - manual_steering) * 0.5f;
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
    double current_time   = getCurrentTime();
    double steering = current_steering_ *180 / M_PI + 90;

    publisher_->publishSteering(steering);
    if (!this->speed_lock_)
    {
        publisher_->publishSpeed(speedPidController_->speedPID(
            desired_speed_ - current_speed_, current_time));
        publisher_->publishCurrentGear(1);
    }
    else
    {
        publisher_->publishSpeed(speedPidController_->speedPID(
            0 - current_speed_, current_time));
        publisher_->publishCurrentGear(0);
    }
    std::cout << "MPC control - "
              << "Current speed: " << current_speed_
              << ", Steering: " << steering << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(fixed_delta_time_ * 1000)));
}

void ModelPredictiveController::run()
{
    while (true)
    {
        if (!xboxController_->getPidEnable()) {
            std::string sae_level = getAutonomousDriveState();

            if (sae_level.find("SAE_0") != std::string::npos) {
                manualControl();
            } else if (sae_level.find("SAE_1_LKAS") != std::string::npos) {
                LKASControl();
            } else if (sae_level.find("SAE_1_ACC") != std::string::npos) {
                adaptiveCruiseControl();
            } else if (sae_level.find("SAE_2") != std::string::npos) {
                partialControl();
            } else if (sae_level.find("SAE_3") != std::string::npos) {
                conditionalAutomation();
            } else if (sae_level.find("SAE_4") != std::string::npos) {
                autonomousControl();
            } else {

            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
