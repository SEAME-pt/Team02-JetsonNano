#include "PidController.hpp"

#ifdef TEST_MODE
// Define custom function names for testing
#define device_open custom_xbox_open
#define device_close custom_xbox_close
#define device_ioctl custom_xbox_ioctl
#define device_read custom_xbox_read
#define device_write custom_xbox_write
#define SESSION_OPEN zenoh::Session::open
#define ZENOH_CONFIG_FROM_FILE zenoh::Config::create_default()
#else
#define device_open open
#define device_close close
#define device_ioctl ioctl
#define device_read read
#define device_write write
#define SESSION_OPEN zenoh::Session::open
#define ZENOH_CONFIG_FROM_FILE zenoh::Config::from_file(configFile)
#endif

double getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

PidController::PidController(XboxController* xbox_controller)
{
    prev_error_  = 0.0f;
    cameraError_ = 0.0f;
    integral_    = 0.0f;
    last_time_   = 0.0f;

    constant_speed_     = 20.0f;
    max_steering_angle_ = 90.0f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_ = 0.02f;
    autonomousDrive_  = "SAE_0";
    xboxController_   = xbox_controller;
    current_speed_    = 0.0f;
    speed_lock_       = false;
    speedPidController_ = new SpeedPidController();
    speedPidController_->init(0.1f, 0.0005f, 0.005f,
                               fixed_delta_time_);

    auto config = zenoh::Config::create_default();
    session_ =
        std::make_shared<zenoh::Session>(SESSION_OPEN(std::move(config)));

    publisher_ = std::make_unique<ControllerPublisher>(session_);

    kp_subscriber.emplace(session_->declare_subscriber(
        "pid/kp",
        [this](const zenoh::Sample& sample)
        {
            float kp = std::stof(sample.get_payload().as_string());
            std::cout << "kp: " << kp << std::endl;
            kp_ = kp;
        },
        zenoh::closures::none));

    ki_subscriber.emplace(session_->declare_subscriber(
        "pid/ki",
        [this](const zenoh::Sample& sample)
        {
            float ki = std::stof(sample.get_payload().as_string());
            std::cout << "ki: " << ki << std::endl;
            ki_ = ki;
        },
        zenoh::closures::none));

    kd_subscriber.emplace(session_->declare_subscriber(
        "pid/kd",
        [this](const zenoh::Sample& sample)
        {
            float kd = std::stof(sample.get_payload().as_string());
            std::cout << "kd: " << kd << std::endl;
            kd_ = kd;
        },
        zenoh::closures::none));

    cameraError_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/LaneDetection/CameraError",
        [this](const zenoh::Sample& sample)
        {
            cameraError_ = std::stof(sample.get_payload().as_string());
            // std::cout << "Camera error: " << cameraError_ << std::endl;
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

            // Convert string to boolean
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
            float speed = std::stof(sample.get_payload().as_string());
            current_speed_ = speed;
        },
        zenoh::closures::none));

    std::cout << "PID controller created!" << std::endl;
}

PidController::PidController(const std::string& configFile,
                             XboxController* xbox_controller)
{
    prev_error_  = 0.0f;
    cameraError_ = 0.0f;
    integral_    = 0.0f;
    last_time_   = 0.0f;

    constant_speed_     = 20.0f;
    max_steering_angle_ = 90.0f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_ = 0.02f;
    autonomousDrive_  = "SAE_0";
    xboxController_   = xbox_controller;
    current_speed_    = 0.0f;
    speed_lock_       = false;
    speedPidController_ = new SpeedPidController();
    speedPidController_->init(0.000001f, 0.000f, 0.00005f,
                               fixed_delta_time_);

    auto config = zenoh::Config::from_file(configFile);
    session_ =
        std::make_shared<zenoh::Session>(SESSION_OPEN(std::move(config)));

    publisher_ = std::make_unique<ControllerPublisher>(session_);

    kp_subscriber.emplace(session_->declare_subscriber(
        "pid/kp",
        [this](const zenoh::Sample& sample)
        {
            float kp = std::stof(sample.get_payload().as_string());
            kp_      = kp;
        },
        zenoh::closures::none));

    ki_subscriber.emplace(session_->declare_subscriber(
        "pid/ki",
        [this](const zenoh::Sample& sample)
        {
            float ki = std::stof(sample.get_payload().as_string());
            ki_      = ki;
        },
        zenoh::closures::none));

    kd_subscriber.emplace(session_->declare_subscriber(
        "pid/kd",
        [this](const zenoh::Sample& sample)
        {
            float kd = std::stof(sample.get_payload().as_string());
            kd_      = kd;
        },
        zenoh::closures::none));

    cameraError_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/LaneDetection/CameraError",
        [this](const zenoh::Sample& sample)
        { cameraError_ = std::stof(sample.get_payload().as_string()); },
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

            // Convert string to boolean
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
            float speed = std::stof(sample.get_payload().as_string());
            current_speed_ = speed;
        },
        zenoh::closures::none));

    std::cout << "PID controller created!" << std::endl;
}

PidController::~PidController() {
    delete speedPidController_;
    session_->close();
}

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

    autonomousDrive_ = "SAE_0";
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

// SAE_1
void PidController::LKASControl(float lane_error, double current_time,
                                float manual_steering, float manual_speed)
{
    // Above threshold, the assistant adjusts slightly the direction
    if (std::abs(lane_error) > lane_departure_threshold_ &&
        std::abs(lane_error) < 1)
    {
        float direction =
            manual_steering +
            (steeringPID(lane_error, current_time) - manual_steering) * 0.5f;
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
        publisher_->publishSpeed(speedPidController_->speedPID(0 - current_speed_, current_time));

}

void PidController::adaptiveCruiseControl(float lane_error, double current_time,
                                          float manual_steering,
                                          float manual_speed)
{
    (void)lane_error;
    (void)current_time;
    (void)manual_steering;
    (void)manual_speed;

    return;
}

// SAE_2
void PidController::partialControl(float lane_error, double current_time)
{
    (void)lane_error;
    (void)current_time;

    return;
}

// SAE_3
void PidController::conditionalAutomation(float lane_error, double current_time)
{
    (void)lane_error;
    (void)current_time;

    return;
}

void PidController::updateControl(float error, double current_time)
{
    float direction = steeringPID(
        error, current_time); // float dynamicSpeed = speedAdjustment(error);

    publisher_->publishSteering(direction);
    // publisher_->publishSpeed(dynamicSpeed);
    // publisher_->publishCurrentGear(1);
}

void PidController::setAutonomousDriveState(std::string current_state)
{
    autonomousDrive_ = current_state;
}

std::string PidController::getAutonomousDriveState() const
{
    return autonomousDrive_;
}

void PidController::run()
{
    while (true)
    {
        double current_time   = getCurrentTime();
        std::string sae_level = getAutonomousDriveState();
        if (sae_level.find("SAE_5") != std::string::npos ||
            sae_level == "SAE_4")
        {
            std::cout << "Speed :" << current_speed_ << std::endl;
            updateControl(cameraError_, current_time);
            publisher_->publishSpeed(speedPidController_->speedPID(500 - current_speed_, current_time));
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(fixed_delta_time_ * 1000)));
        }
        else
        {
            float manual_steering = xboxController_->getManualSteering();
            float manual_speed    = xboxController_->getManualSpeed();
            if (sae_level.find("SAE_1_LKAS") != std::string::npos)
            {
                LKASControl(cameraError_, current_time, manual_steering,
                            manual_speed);
            }
            else if (sae_level == "SAE_1_ACC")
            {
                adaptiveCruiseControl(cameraError_, current_time,
                                      manual_steering, manual_speed);
            }
            else if (sae_level == "SAE_2")
            {
                partialControl(cameraError_, current_time);
            }
            else if (sae_level == "SAE_3")
            {
                conditionalAutomation(cameraError_, current_time);
            }
            else
            {
                publisher_->publishSteering(manual_steering);
                if (!speed_lock_)
                    publisher_->publishSpeed(manual_speed);
                else
                    publisher_->publishSpeed(speedPidController_->speedPID(0 - current_speed_, current_time));
            }
        }
    }
}