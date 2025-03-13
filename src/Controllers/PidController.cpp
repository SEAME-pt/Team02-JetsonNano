#include "PidController.hpp"

PidController::PidController()
{
    prev_error_ = 0.0f;
    integral_ = 0.0f;
    last_time_ = 0.0f;

    constant_speed_ = 20.0f;
    max_steering_angle_ = 90.0f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_ = 0.02f;

    auto config = zenoh::Config::create_default();
    session_    = std::make_shared<zenoh::Session>(
        SESSION_OPEN(std::move(config)));

    publisher_ = std::make_unique<ControllerPublisher>(session_);

    std::cout << "PID controller created!" << std::endl;
}

PidController::PidController(onst std::string& configFile)
{
    prev_error_ = 0.0f;
    integral_ = 0.0f;
    last_time_ = 0.0f;

    constant_speed_ = 20.0f;
    max_steering_angle_ = 90.0f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_ = 0.02f;

    auto config = zenoh::Config::from_file(configFile);
    session_    = std::make_shared<zenoh::Session>(
        SESSION_OPEN(std::move(config)));

    publisher_ = std::make_unique<ControllerPublisher>(session_);

    std::cout << "PID controller created!" << std::endl;
}

PidController::~PidController()
{}

void PidLaneController::init(float kp, float ki, float kd, float speed, float delta_time) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    constant_speed_ = speed;
    fixed_delta_time_ = delta_time;
    
    previous_error_ = 0.0f;
    integral_ = 0.0f;
    
    std::cout << "PID Controller initialized with Kp=" << kp_ << ", Ki=" << ki_ 
              << ", Kd=" << kd_ << ", speed=" << constant_speed_
              << ", dt=" << fixed_delta_time_ << std::endl;
}

void PidLaneController::updateControl(float error, float current_time) {
    
    // dt
    float dt = current_time - last_time_;


    //PID
    float p_term = kp_ * error;

    integral_ += error * dt;
    float i_term = ki * integral_;
    
    float d_term = kd_ * (error - prev_error_) / dt;

    //Adjust steering
    float steering_correction = p_term + i_term + d_term;
    
    float direction = 90 + steering_correction;
    if (direction > 90.0f + max_steering_angle_) {
        direction = 90.0f + max_steering_angle_;
    } else if (direction < 90.0f - max_steering_angle_) {
        direction = 90.0f - max_steering_angle_;
    }

    publisher_->publishSteering(direction);
    publisher_->publishSpeed(constant_speed_);
    publisher_->publishCurrentGear(1);

    previous_error_ = error;
    last_time_ = current_time;

}

void PidLaneController::run() {

    while(true)
    {
        //float error = getLaneError(); // Get from camera/vision system
        float current_time = getCurrentTime();
        updateControl(error, current_time);
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(fixed_delta_time_ * 1000)));
    }
}