#include "MPController.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ModelPredictiveController::ModelPredictiveController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller)
{
    
    autonomousDrive_    = "SAE_0";
    xboxController_     = xbox_controller;
    fixed_delta_time_   = 0.02f;
    current_speed_      = 0.0f;
    current_steering_      = 0.0f;
    speedKp_ = 0.12f;
    speedKi_ = 1.3f;
    speedKd_ = 0.01f;
    speedPidController_ = new SpeedPidController();
    speedPidController_->init(speedKp_, speedKi_, speedKd_, fixed_delta_time_);
    speed_lock_         = false;
    
    session_ = session;
    
    publisher_ = std::make_unique<ControllerPublisher>(session_);
    
    coeffs_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Coeffs",
        [this](const zenoh::Sample& sample)
        {
            std::string coeffs_str = sample.get_payload().as_string();
            
            // Parse the comma-separated string into a vector of doubles
            std::vector<double> parsed_coeffs;
            std::stringstream ss(coeffs_str);
            std::string token;
            
            // Split by comma and convert to double
            while (std::getline(ss, token, ',')) {
                parsed_coeffs.push_back(std::stod(token));
            }
            
            // Make sure we have all 4 coefficients for a 3rd degree polynomial
            if (parsed_coeffs.size() == 4) {
                // Create the initial state vector
                Eigen::Vector4d x0;
                x0(0) = 0;
                x0(1) = 0;
                x0(2) = current_steering_;
                x0(3) = current_speed_;
                
                // Solve the MPC problem with the new coefficients
                this->solve(x0, parsed_coeffs);
                
                // Debug output
                std::cout << "Received coefficients: " 
                          << parsed_coeffs[0] << ", " 
                          << parsed_coeffs[1] << ", " 
                          << parsed_coeffs[2] << ", " 
                          << parsed_coeffs[3] << std::endl;
            } else {
                std::cerr << "Invalid number of coefficients: " 
                          << parsed_coeffs.size() << " (expected 4)" << std::endl;
            }
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

ModelPredictiveController::~ModelPredictiveController() {}

void ModelPredictiveController::init(size_t horizon, double wheelbase, double Ts,
                             const Eigen::Matrix4d& Q,
                             const Eigen::Matrix2d& R,
                             const Eigen::Matrix4d& Qf)
{
    // Replace initializer list with regular assignments
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

void ModelPredictiveController::setVehicleState(const Eigen::Vector4d& state) {
    this->currentState_ = state;
}

void ModelPredictiveController::solve(const Eigen::Vector4d& x0,
                                         const std::vector<double>& traj_coeffs)
{
    std::vector<Eigen::Vector4d> x_ref(N_ + 1);
    double y_ref_current = x0(1);

    // double v_start = x0(3);
    double v_target = target_velocity_;
    // double v_diff = v_target - v_start;

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

    // publisher_->publishSpeed(
    //         speedPidController_->speedPID(u_flat(0) - current_speed_, current_time));
    publisher_->publishSpeed(u_flat(0));
    publisher_->publishSteering(u_flat(1));
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

void ModelPredictiveController::setTargetVelocity(double velocity) {
    target_velocity_ = velocity;
}


void ModelPredictiveController::run()
{
    while (true)
    {
        // double current_time   = getCurrentTime();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // std::string sae_level = getAutonomousDriveState();
        // if (sae_level.find("SAE_5") != std::string::npos ||
        //     sae_level == "SAE_4")
        // {
        //     updateControl(cameraError_, current_time);
        //     if (!speed_lock_)
        //     {
        //         publisher_->publishSpeed(speedPidController_->speedPID(
        //             250 - current_speed_, current_time));
        //     }
        //     else
        //     {
        //         publisher_->publishSpeed(speedPidController_->speedPID(
        //             0 - current_speed_, current_time));
        //     }
        //     std::this_thread::sleep_for(std::chrono::milliseconds(
        //         static_cast<int>(fixed_delta_time_ * 1000)));
        // }
        // else
        // {
        //     float manual_steering = xboxController_->getManualSteering();
        //     float manual_speed    = xboxController_->getManualSpeed();
        //     if (sae_level.find("SAE_1_LKAS") != std::string::npos)
        //     {
        //         LKASControl(cameraError_, current_time, manual_steering,
        //                     manual_speed);
        //     }
        //     else if (sae_level == "SAE_1_ACC")
        //     {
        //         adaptiveCruiseControl(cameraError_, current_time,
        //                               manual_steering, manual_speed);
        //     }
        //     else if (sae_level == "SAE_2")
        //     {
        //         partialControl(cameraError_, current_time);
        //     }
        //     else if (sae_level == "SAE_3")
        //     {
        //         conditionalAutomation(cameraError_, current_time);
        //     }
        //     else
        //     {
        //         publisher_->publishSteering(manual_steering);
        //         if (!speed_lock_)
        //             publisher_->publishSpeed(manual_speed);
        //         else
        //         {
        //             if (manual_speed <= 0)
        //             {
        //                 publisher_->publishSpeed(manual_speed);
        //             }
        //             else
        //             {
        //                 publisher_->publishSpeed(speedPidController_->speedPID(
        //                     0 - current_speed_, current_time));
        //             }
        //         }
        //     }
        // }
    }
}

// static std::vector<Eigen::Vector2d>
// projectCurveToGround(const std::vector<double>& coeffs_uv, double focal_length,
//                      int image_width, int image_height, double camera_height,
//                      double pitch_rad)
// {
//     double cx = image_width / 2.0;
//     double cy = image_height / 2.0;

//     auto projectPoint = [&](double u, double v) -> Eigen::Vector2d
//     {
//         double x_cam = (u - cx) / focal_length;
//         double y_cam = (v - cy) / focal_length;
//         Eigen::Vector3d d_cam(x_cam, y_cam, 1.0);

//         Eigen::Matrix3d R_pitch;
//         R_pitch << 1, 0, 0, 0, cos(pitch_rad), -sin(pitch_rad), 0,
//             sin(pitch_rad), cos(pitch_rad);

//         Eigen::Vector3d d_world = R_pitch * d_cam;
//         double scale            = -camera_height / d_world.z();
//         double X                = scale * d_world.x();
//         double Y                = scale * d_world.y();
//         return Eigen::Vector2d(X, Y);
//     };

//     std::vector<Eigen::Vector2d> points;
//     for (int v = 0; v <= image_height; v += image_height / 5)
//     {
//         double u = coeffs_uv[0] + coeffs_uv[1] * v + coeffs_uv[2] * v * v +
//                    coeffs_uv[3] * v * v * v;
//         points.push_back(projectPoint(u, v));
//     }

//     return points;
// }

// static std::vector<double>
// fitThirdDegreePolynomial(const std::vector<Eigen::Vector2d>& points)
// {
//     Eigen::MatrixXd A(points.size(), 4);
//     Eigen::VectorXd b(points.size());

//     for (size_t i = 0; i < points.size(); ++i)
//     {
//         double y = points[i].y();
//         A(i, 0)  = 1.0;
//         A(i, 1)  = y;
//         A(i, 2)  = y * y;
//         A(i, 3)  = y * y * y;
//         b(i)     = points[i].x();
//     }

//     Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);

//     return {coeffs(0), coeffs(1), coeffs(2), coeffs(3)};
// }

// int main() {
//     size_t N = 10;
//     double L = 2.5, Ts = 0.1;
//     Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
//     Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
//     Eigen::Matrix4d Qf = Q;

//     MPController mpc(N, L, Ts, Q, R, Qf);

//     Eigen::Vector4d x0(0, 0, 0, 2);
//     std::vector<double> traj_coeffs = {0, 2, 0, 0};

//     auto control = mpc.solve(x0, traj_coeffs);
//     std::cout << "Next steering: " << control.delta
//               << ", throttle: " << control.throttle << std::endl;

//     std::vector<double> coeffs_uv = {100.0, -0.2, 0.0005, -0.000001};
//     auto ground_points = MPController::projectCurveToGround(coeffs_uv, 800.0,
//     1280, 720, 1.2, 0.1);

//     auto poly = MPController::fitThirdDegreePolynomial(ground_points);
//     std::cout << "Trajectory polynomial x(y) = "
//               << poly[0] << " + " << poly[1] << "*y + "
//               << poly[2] << "*y^2 + " << poly[3] << "*y^3" << std::endl;

//     return 0;
// }

// void ModelPredictiveController::setAutonomousDriveState(
//     std::string current_state)
// {
//     autonomousDrive_ = current_state;
// }

// std::string ModelPredictiveController::getAutonomousDriveState() const
// {
//     return autonomousDrive_;
// }