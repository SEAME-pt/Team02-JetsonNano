#include "PidController.hpp"

static double getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

PidController::PidController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller)
{
    prev_error_  = 0.0f;
    cameraError_ = 0.0f;
    integral_    = 0.0f;
    last_time_   = 0.0f;

    constant_speed_     = 0.0f;
    max_steering_angle_ = 90.0f;

    lane_departure_threshold_ = 0.1f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_   = 0.02f;
    autonomousDrive_    = "SAE_0";
    speed_lock_         = false;
    xboxController_     = xbox_controller;
    session_ = session;
    
    publisher_ = std::make_unique<ControllerPublisher>(session_);

    cameraError_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/LaneDetection/CameraError",
        [this](const zenoh::Sample& sample)
        {
            cameraError_ = std::stof(sample.get_payload().as_string());
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

            // std::cout << "Speed lock "
            //           << (lock_value ? "activated" : "deactivated")
            //           << std::endl;
        },
        zenoh::closures::none));

    std::cout << "PID controller created!" << std::endl;
}

PidController::~PidController()
{}

void PidController::init(float kp, float ki, float kd, float speed,
                         float delta_time)
{
    kp_               = kp;
    ki_               = ki;
    kd_               = kd;
    constant_speed_   = speed;
    fixed_delta_time_ = delta_time;

    prev_error_ = 0.0f;
    integral_   = 0.0f;

    std::cout << "PID Controller initialized with Kp=" << kp_ << ", Ki=" << ki_
              << ", Kd=" << kd_ << ", speed=" << constant_speed_
              << ", dt=" << fixed_delta_time_ << std::endl;
}

void PidController::setAutonomousDriveState(std::string current_state)
{
    autonomousDrive_ = current_state;
}

std::string PidController::getAutonomousDriveState() const
{
    return autonomousDrive_;
}

float PidController::steeringPID(float error, double current_time)
{
    // dt
    double dt = current_time - last_time_;

    // PID
    float p_term = kp_ * error;

    // Improved implementation with anti-windup
    integral_ += error * dt;
    // Limit integral term to prevent windup
    const float MAX_INTEGRAL = 10.0f; // Adjust based on your system
    integral_    = std::max(-MAX_INTEGRAL, std::min(integral_, MAX_INTEGRAL));
    float i_term = ki_ * integral_;

    float d_term = kd_ * (error - prev_error_) / dt;

    // Adjust steering
    float steering_correction = p_term + i_term + d_term;

    float direction = 90 + steering_correction;
    if (direction > 90.0f + max_steering_angle_)
    {
        direction = 90.0f + max_steering_angle_;
    }
    else if (direction < 90.0f - max_steering_angle_)
    {
        direction = 90.0f - max_steering_angle_;
    }

    prev_error_ = error;
    last_time_  = current_time;
    return direction;
}

// SAE_0
void PidController::manualControl()
{
    // double current_time   = getCurrentTime();
    float manual_steering = xboxController_->getManualSteering();
    // float manual_speed    = xboxController_->getManualSpeed();

    publisher_->publishSteering(manual_steering);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_1_LKAS
void PidController::LKASControl()
{
    double current_time   = getCurrentTime();
    float manual_steering = xboxController_->getManualSteering();
    float manual_speed    = xboxController_->getManualSpeed();
    static bool LKAS_enable = false;

    if (std::abs(cameraError_) > lane_departure_threshold_ &&
        std::abs(cameraError_) < 0.5)
    {
        float direction = manual_steering + (steeringPID(cameraError_, current_time) - manual_steering) * 1.2f;
        if (cameraError_ < 0)
        {
            LKAS_enable = true;
            publisher_->publishLaneAlert("Left");
        }
        else
        {
            LKAS_enable = true;
            publisher_->publishLaneAlert("Right");
        }
        publisher_->publishSteering(direction);
    }
    else
    {
        if (LKAS_enable)
        {
            publisher_->publishLaneAlert("Off");
            LKAS_enable = false;
        }
        publisher_->publishSteering(manual_steering);
    }
    publisher_->publishSpeed(manual_speed);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_1_ACC
void PidController::adaptiveCruiseControl()
{
    // double current_time   = getCurrentTime();
    // float manual_steering = xboxController_->getManualSteering();
    // float manual_speed    = xboxController_->getManualSpeed();
    publisher_->publishSpeed(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_2
void PidController::partialControl()
{
    // double current_time   = getCurrentTime();
    publisher_->publishSpeed(0);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_3
void PidController::conditionalAutomation()
{
    double current_time   = getCurrentTime();
    // float manual_steering = xboxController_->getManualSteering();
    float direction = steeringPID(cameraError_, current_time);

    // publisher_->publishSteering(manual_steering);
    publisher_->publishSteering(direction);
    // publisher_->publishSpeed(xboxController_->getManualSpeed());
    if (!this->speed_lock_)
    {
        publisher_->publishDesiredSpeed(constant_speed_);
        publisher_->publishCurrentGear(1);
    }
    else
    {
        publisher_->publishDesiredSpeed(0);
        publisher_->publishCurrentGear(0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(fixed_delta_time_ * 1000)));
}

// SAE_4
void PidController::autonomousControl()
{
    double current_time   = getCurrentTime();
    // float manual_steering = xboxController_->getManualSteering();
    float direction = steeringPID(cameraError_, current_time);

    // publisher_->publishSteering(manual_steering);
    publisher_->publishSteering(direction);
    // publisher_->publishSpeed(xboxController_->getManualSpeed());
    if (!this->speed_lock_)
    {
        publisher_->publishDesiredSpeed(constant_speed_);
        publisher_->publishCurrentGear(1);
    }
    else
    {
        publisher_->publishDesiredSpeed(0);
        publisher_->publishCurrentGear(0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(fixed_delta_time_ * 1000)));
}

void PidController::run()
{
    while (true)
    {
        if (xboxController_->getPidEnable()) {
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
