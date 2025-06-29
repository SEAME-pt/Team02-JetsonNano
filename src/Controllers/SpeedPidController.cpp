#include "SpeedPidController.hpp"

static double getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}


SpeedPidController::SpeedPidController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller)
{
    prev_error_  = 0.0f;
    integral_    = 0.0f;
    last_time_   = 0.0f;

    max_speed_    = 90.0f;
    max_throttle_ = 100.0f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_ = 0.02f;
    autonomousDrive_    = "SAE_0";
    speed_lock_         = false;
    xboxController_     = xbox_controller;

    std::cout << "SpeedPID controller created!" << std::endl;

    session_ = session;
    publisher_ = std::make_unique<ControllerPublisher>(session_);

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
            std::cout << "Updated Ki: " << ki_ << std::endl;
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

    currentSpeed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed",
        [this](const zenoh::Sample& sample)
        {
            float speed    = std::stof(sample.get_payload().as_string());
            current_speed_ = speed;
        },
        zenoh::closures::none));

    desiredSpeed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/speedPid/DesiredSpeed",
        [this](const zenoh::Sample& sample)
        {
            float speed    = std::stof(sample.get_payload().as_string());
            desired_speed_ = speed * 60.0 / (M_PI * 0.067);
        },
        zenoh::closures::none));

    currentYaw_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Chassis/SteeringWheel/Angle",
        [this](const zenoh::Sample& sample)
        {
            float steer    = std::stof(sample.get_payload().as_string());
            steer_ = (steer - 90.0) / 180;
        },
        zenoh::closures::none));


}

SpeedPidController::~SpeedPidController() {}

void SpeedPidController::init(float kp, float ki, float kd, float delta_time)
{

    kp_               = kp;
    ki_               = ki;
    kd_               = kd;
    fixed_delta_time_ = delta_time;

    prev_error_ = 0.0f;
    integral_   = 0.0f;
    desired_speed_ = 0.0f;

    std::cout << "SpeedPID Controller initialized with Kp=" << kp_
              << ", Ki=" << ki_ << ", Kd=" << kd_
              << ", dt=" << fixed_delta_time_ << std::endl;

}

void SpeedPidController::setAutonomousDriveState(std::string current_state)
{
    autonomousDrive_ = current_state;
}

std::string SpeedPidController::getAutonomousDriveState() const
{
    return autonomousDrive_;
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

void SpeedPidController::run()
{
    // while (true)
    // {
    //     std::string sae_level = getAutonomousDriveState();
    //     if (!speed_lock_)
    //     {
    //         if (sae_level.find("SAE_0") != std::string::npos) {
    //             float manual_speed    = xboxController_->getManualSpeed();
    //             publisher_->publishSpeed(manual_speed);

    //         } else if (sae_level.find("SAE_1_LKAS") != std::string::npos) {

    //         } else if (sae_level.find("SAE_1_ACC") != std::string::npos) {

    //         } else if (sae_level.find("SAE_2") != std::string::npos) {

    //         } else if (sae_level.find("SAE_3") != std::string::npos) {

    //         } else if (sae_level.find("SAE_4") != std::string::npos) {
    //             double current_time = getCurrentTime();
    //             float error = desired_speed_ - current_speed_;
    //             double throttle = speedPID(error, current_time);
    //             throttle = std::max(0.0, throttle); 
    //             publisher_->publishSpeed(throttle);
    //             std::this_thread::sleep_for(std::chrono::milliseconds(
    //                         static_cast<int>(fixed_delta_time_ * 10000)));
    //         } else {

    //         }
    //     }
    //     else
    //     {
    //         float manual_speed    = xboxController_->getManualSpeed();
    //         if (manual_speed <= 0)
    //         {
    //             publisher_->publishSpeed(manual_speed);
    //             std::this_thread::sleep_for(std::chrono::milliseconds(
    //                         static_cast<int>(fixed_delta_time_ * 1000)));
    //         }
    //         else
    //         {
    //             double current_time = getCurrentTime();
    //             float error = 0 - current_speed_;
    //             double throttle = speedPID(error, current_time);
    //             publisher_->publishSpeed(throttle);
    //             std::this_thread::sleep_for(std::chrono::milliseconds(
    //                         static_cast<int>(fixed_delta_time_ * 10000)));
    //         }
    //     }
    // }

    //calibration
    while (true){
        std::string sae_level = getAutonomousDriveState();
        if (sae_level.find("SAE_4") != std::string::npos)
            break;
    }

    double throttle = 0.15;
    publisher_->publishSpeed(throttle);
    
    // Wait for a trigger to increase throttle (could be a timer, button, or code logic)
    std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // Example: wait 2 seconds
    
    throttle = 0.25;
    publisher_->publishSpeed(throttle);
    
    // Start logging
    logging_ = true;
    log_start_time_ = getCurrentTime();
    log_file_.open("straight_speed_pid_log.csv");
    log_file_ << "time,speed,throttle\n";
    
    while (logging_) {
        double now = getCurrentTime();
        // Log speed and throttle
        log_file_ << (now - log_start_time_) << "," << current_speed_ << "," << throttle << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Log at 100 Hz
        
        // Stop after 2 seconds
        if (now - log_start_time_ > 2.0) {
            logging_ = false;
            log_file_.close();
            publisher_->publishSpeed(0);
        }
    }
}