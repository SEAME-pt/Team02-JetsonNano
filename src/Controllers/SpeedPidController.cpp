#include "SpeedPidController.hpp"

SpeedPidController::SpeedPidController()
{
    prev_error_  = 0.0f;
    cameraError_ = 0.0f;
    integral_    = 0.0f;
    last_time_   = 0.0f;

    max_speed_    = 90.0f;
    max_throttle_ = 100.0f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_ = 0.02f;

    std::cout << "SpeedPID controller created!" << std::endl;
}

SpeedPidController::~SpeedPidController() {}

void SpeedPidController::init(float kp, float ki, float kd, float delta_time, std::shared_ptr<zenoh::Session> session)
{
    session_ = session;

    kp_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/speedpid/kp",
        [this](const zenoh::Sample& sample)
        {
            kp_ = std::stof(sample.get_payload().as_string());
            std::cout << "Updated Kp: " << kp_ << std::endl;
        },
        zenoh::closures::none));
    
    ki_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/speedpid/ki",
        [this](const zenoh::Sample& sample)
        {
            ki_ = std::stof(sample.get_payload().as_string());
            std::cout << "Updated Ksadhaswdpofishi: " << ki_ << std::endl;
        },
        zenoh::closures::none));

    kd_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/speedpid/kd",
        [this](const zenoh::Sample& sample)
        {
            kd_ = std::stof(sample.get_payload().as_string());
            std::cout << "Updated Kd: " << kd_ << std::endl;
        },
        zenoh::closures::none));

    kp_               = kp;
    ki_               = ki;
    kd_               = kd;
    fixed_delta_time_ = delta_time;

    prev_error_ = 0.0f;
    integral_   = 0.0f;

    std::cout << "SpeedPID Controller initialized with Kp=" << kp_
              << ", Ki=" << ki_ << ", Kd=" << kd_
              << ", dt=" << fixed_delta_time_ << std::endl;
}

float SpeedPidController::speedPID(float error, double current_time)
{
    // std::cout << "Error in speed PID: " << error << std::endl;
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
    float outputThrottle = p_term + i_term + d_term;
    float throttle       = outputThrottle;
    if (throttle > max_throttle_)
    {
        throttle = max_throttle_;
    }
    else if (throttle < -max_throttle_)
    {
        throttle = -max_throttle_;
    }

    prev_error_ = error;
    last_time_  = current_time;
    // std::cout << "Throttle output: " << throttle << std::endl;
    return throttle;
}