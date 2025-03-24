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

PidController::PidController()
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
    autonomousDrive_  = false;

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
            std::cout << "Camera error: " << cameraError_ << std::endl;
        },
        zenoh::closures::none));

    activeAutonomyLevel_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/ActiveAutonomyLevel",
        [this](const zenoh::Sample& sample)
        {
            std::string activeAutonomyLevel = sample.get_payload().as_string();
            if (activeAutonomyLevel == "SAE_5" &&
                getAutonomousDriveState() == false)
            {
                setAutonomousDriveState(true);
                std::cout << "Autonomous Sub True" << std::endl;
            }
            else if (getAutonomousDriveState() == true)
            {
                std::cout << "Autonomous Sub False" << std::endl;
                setAutonomousDriveState(false);
            }
            else
            {
                // empty
            }
        },
        zenoh::closures::none));

    std::cout << "PID controller created!" << std::endl;
}

PidController::PidController(const std::string& configFile)
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
            if (activeAutonomyLevel == "SAE_5" &&
                getAutonomousDriveState() == false)
            {
                setAutonomousDriveState(true);
                std::cout << "Autonomous Sub True" << std::endl;
            }
            else if (getAutonomousDriveState() == true)
            {
                std::cout << "Autonomous Sub False" << std::endl;
                setAutonomousDriveState(false);
            }
            else
            {
                // empty
            }
        },
        zenoh::closures::none));
    std::cout << "PID controller created!" << std::endl;
}

PidController::~PidController() {}

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

void PidController::updateControl(float error, double current_time)
{
    // dt
    double dt = current_time - last_time_;
    std::cout << "dt: " << dt << std::endl;

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

    // Dynamic speed adjustment based on error
    float error_magnitude = std::fabs(error);

    // Define speed control parameters
    const float BASE_SPEED =
        constant_speed_; // Maximum speed when error is minimal
    const float MIN_SPEED = BASE_SPEED * 0.5f; // Minimum speed (50% of max)
    const float ERROR_THRESHOLD =
        40.0f; // Error threshold where speed starts decreasing

    // Calculate dynamic speed
    float dynamic_speed;
    if (error_magnitude < ERROR_THRESHOLD)
    {
        // Linear interpolation between BASE_SPEED and slightly reduced speed
        float reduction_factor = error_magnitude / ERROR_THRESHOLD;
        dynamic_speed = BASE_SPEED - (reduction_factor * (BASE_SPEED * 0.2f));
    }
    else
    {
        // For larger errors, reduce speed more aggressively
        float excess_error = error_magnitude - ERROR_THRESHOLD;
        float reduction_factor =
            std::min(1.0f, excess_error / (ERROR_THRESHOLD * 2));
        dynamic_speed =
            BASE_SPEED * 0.8f - (reduction_factor * (BASE_SPEED * 0.2f));
    }

    // Ensure speed doesn't go below minimum
    dynamic_speed = std::max(MIN_SPEED, dynamic_speed) * 100;

    publisher_->publishSteering(direction);
    publisher_->publishSpeed(dynamic_speed);

    std::cout << "Direction: " << direction << ", Speed: " << dynamic_speed
              << std::endl;
    // publisher_->publishCurrentGear(1);

    prev_error_ = error;
    last_time_  = current_time;
}

void PidController::run()
{
    while (true)
    {
        if (getAutonomousDriveState())
        {
            double current_time = getCurrentTime();
            updateControl(cameraError_, current_time);
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(fixed_delta_time_ * 1000)));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(fixed_delta_time_ * 1000)));
        }
    }
}

void PidController::setAutonomousDriveState(bool toggle)
{
    autonomousDrive_ = toggle;
}

bool PidController::getAutonomousDriveState() const
{
    return autonomousDrive_;
}