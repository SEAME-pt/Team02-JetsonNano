#include "MPController.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ModelPredictiveController::ModelPredictiveController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller)
{
    fixed_delta_time_   = 0.02f;
    autonomousDrive_    = "SAE_0";
    speed_lock_         = false;
    xboxController_     = xbox_controller;
    current_speed_      = 0.0f;
    desired_speed_      = 0.0f;
    current_steering_      = 0.0f;
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
                x0(3) = current_speed_;
                
                this->solve(x0, parsed_coeffs);
                
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
    
    std::cout << "MPC initialized with horizon=" << N_ 
              << ", wheelbase=" << L_ 
              << ", timestep=" << Ts_ << std::endl;
}

void ModelPredictiveController::solve(const Eigen::Vector4d& x0,
                                         const std::vector<double>& traj_coeffs)
{
    std::vector<Eigen::Vector4d> x_ref(N_ + 1);
    double y_ref_current = x0(1);

    double v_target = target_velocity_;

    for (size_t k = 0; k <= N_; ++k)
    {
        double y = y_ref_current + k * x0(3) * Ts_;
        double x = traj_coeffs[0] + traj_coeffs[1] * y +
                   traj_coeffs[2] * y * y + traj_coeffs[3] * y * y * y;
        double dx_dy = traj_coeffs[1] + 2 * traj_coeffs[2] * y +
                       3 * traj_coeffs[3] * y * y;
        double psi = std::atan(dx_dy) +
                     M_PI / 2.0; // Adjust heading because Y is perpendicular
        x_ref[k] << x, y, psi, v_target;
    }

    Eigen::VectorXd u_flat = Eigen::VectorXd::Zero(2 * N_);
    double alpha           = 0.1;
    const int max_iter     = 20;
    const double tol       = 1e-4;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        std::vector<Eigen::Vector4d> x_seq(N_ + 1);
        x_seq[0] = x0;

        for (size_t k = 0; k < N_; ++k)
        {
            Eigen::Vector2d u_k;
            u_k << u_flat(2 * k), u_flat(2 * k + 1);
            x_seq[k + 1] = backwardEuler(x_seq[k], u_k);
        }

        Eigen::VectorXd grad = Eigen::VectorXd::Zero(2 * N_);

        for (size_t k = 0; k < N_; ++k)
        {
            Eigen::Vector4d dx = x_seq[k] - x_ref[k];
            grad(2 * k) += 2 * (R_(0, 0) * u_flat(2 * k) + Q_(3, 3) * dx(3));
            grad(2 * k + 1) +=
                2 * (R_(1, 1) * u_flat(2 * k + 1) + Q_(2, 2) * dx(2) +
                     Q_(0, 0) * dx(0) + Q_(1, 1) * dx(1));
        }

        Eigen::Vector4d dxN = x_seq[N_] - x_ref[N_];
        grad(2 * (N_ - 1)) +=
            2 * (Qf_(3, 3) * dxN(3) + Qf_(0, 0) * dxN(0) + Qf_(1, 1) * dxN(1));
        grad(2 * (N_ - 1) + 1) +=
            2 * (Qf_(2, 2) * dxN(2) + Qf_(0, 0) * dxN(0) + Qf_(1, 1) * dxN(1));

        if (grad.norm() < tol)
            break;

        u_flat -= alpha * grad;

        for (size_t k = 0; k < N_; ++k)
        {
            u_flat(2 * k) =
                std::max(0.0, std::min(1.0, u_flat(2 * k))); // throttle [0, 1]
            u_flat(2 * k + 1) = std::max(
                -0.7854,
                std::min(
                    0.7854,
                    u_flat(2 * k + 1))); // steering [-45deg, +45deg] in radians
        }
    }

    publisher_->publishSteering(u_flat(1));
    desired_speed_ = u_flat(0);
    current_steering_ = u_flat(1);
}

// Forward Euler discretization
Eigen::Vector4d
ModelPredictiveController::backwardEuler(const Eigen::Vector4d& x,
                                         const Eigen::Vector2d& u)
{
    double v_next   = x(3) + Ts_ * u(0);
    double psi_next = x(2) + Ts_ * (v_next / L_) * std::tan(u(1));
    double Xf_next  = x(0) + Ts_ * v_next * std::cos(psi_next);
    double Yf_next  = x(1) + Ts_ * v_next * std::sin(psi_next);

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
    float manual_steering = xboxController_->getManualSteering();
    float manual_speed    = xboxController_->getManualSpeed();
    publisher_->publishSteering(manual_steering);
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
    double current_time   = getCurrentTime();
    float manual_steering = xboxController_->getManualSteering();
    float manual_speed    = xboxController_->getManualSpeed();

    if (std::abs(cameraError_) > lane_departure_threshold_ &&
        std::abs(cameraError_) < 1)
    {
        float direction = manual_steering +(steeringPID(cameraError_, current_time) - manual_steering) * 0.5f;
        // publisher_->publishAlert("Lane Departure");
        publisher_->publishSteering(direction);
    }
    else
    {
        publisher_->publishSteering(manual_steering);
    }
    if (!speed_lock_)
        publisher_->publishSpeed(manual_speed);
    else
        publisher_->publishSpeed(
            speedPidController_->speedPID(0 - current_speed_, current_time));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(fixed_delta_time_ * 1000)));
}

void ModelPredictiveController::run()
{
    while (true)
    {
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
    }
}