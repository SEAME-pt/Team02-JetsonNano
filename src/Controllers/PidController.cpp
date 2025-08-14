#include "PidController.hpp"

static double getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

PidController::PidController(std::shared_ptr<zenoh::Session> session,
                             XboxController* xbox_controller)
{
    prev_error_  = 0.0f;
    cameraError_ = 0.0f;
    integral_    = 0.0f;
    last_time_   = 0.0f;

    desired_speed_      = 0.0f;
    acc_speed_          = 0.0f;
    speed_limit_        = 0.0f;
    max_steering_angle_ = 90.0f;

    last_acc_speed_receive_  = 0.0f;
    last_crosswalk_received_ = 0.0f;
    last_danger_received_    = 0.0f;
    last_yield_received_     = 0.0f;
    last_stop_received_      = 0.0f;
    last_red_received_       = 0.0f;
    last_green_received_     = 0.0f;
    last_yellow_received_    = 0.0f;

    lane_departure_threshold_ = 0.1f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_ = 0.02f;
    autonomousDrive_  = "SAE_0";
    emergency_brake_  = false;
    xboxController_   = xbox_controller;
    session_          = session;

    publisher_ = std::make_unique<ControllerPublisher>(session_);

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

    emergency_brake_subscriber_.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed/Emergency",
        [this](const zenoh::Sample& sample)
        {
            std::string value_str = sample.get_payload().as_string();

            bool emergency_value_ = false;
            if (value_str.find("1") != std::string::npos)
            {
                emergency_value_ = true;
            }

            emergency_brake_ = emergency_value_;

            // std::cout << "Emergency Brake "
            //           << (emergency_value_ ? "activated" : "deactivated")
            //           << std::endl;
        },
        zenoh::closures::none));

    LKAS_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/LKAS",
        [this](const zenoh::Sample& sample)
        {
            laneProximity_ = std::stof(sample.get_payload().as_string());
            lastLaneProximityMeasure_ = getCurrentTime();
            LKASon                    = true;
        },
        zenoh::closures::none));

    currentSpeed_subscriber_.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed",
        [this](const zenoh::Sample& sample)
        {
            float speed    = std::stof(sample.get_payload().as_string());
            current_speed_ = speed;
        },
        zenoh::closures::none));

    acc_speed_subscriber_.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/acc_speed",
        [this](const zenoh::Sample& sample)
        {
            float speed = std::stof(sample.get_payload().as_string());
            acc_speed_  = speed;
            last_acc_speed_receive_ = getCurrentTime();
            std::cout << "Acc speed received: " << acc_speed_ << std::endl;
        },
        zenoh::closures::none));

    trafficSign_subscriber_.emplace(session_->declare_subscriber(
        "Vehicle/1/TrafficSign",
        [this](const zenoh::Sample& sample)
        {
            std::string value_str = sample.get_payload().as_string();

            if (value_str.find("Speed 80km/h") != std::string::npos)
            {
                speed_limit_ = 0.30;
            }
            else if (value_str.find("Speed 50km/h") != std::string::npos)
            {
                speed_limit_ = 0.25;
            }
            else if (value_str.find("Danger") != std::string::npos)
            {
                last_danger_received_ = getCurrentTime();
            }
            else if (value_str.find("Crosswalk") != std::string::npos)
            {
                last_crosswalk_received_ = getCurrentTime();
            }
            else if (value_str.find("Yield") != std::string::npos)
            {
                last_yield_received_ = getCurrentTime();
            }
            else if (value_str.find("Stop") != std::string::npos)
            {
                last_stop_received_ = getCurrentTime();
            }
            else if (value_str.find("Traffic Green") != std::string::npos)
            {
                std::cout << "Green" << std::endl;
                green_active_        = true;
                last_green_received_ = getCurrentTime();
            }
            else if (value_str.find("Traffic Red") != std::string::npos)
            {
                red_active_        = true;
                last_red_received_ = getCurrentTime();
            }
            else if (value_str.find("Traffic Yellow") != std::string::npos)
            {
                yellow_active_        = true;
                last_yellow_received_ = getCurrentTime();
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
    desired_speed_    = speed;
    current_speed_    = 0.0f;
    speed_limit_      = speed;
    fixed_delta_time_ = delta_time;

    prev_error_ = 0.0f;
    integral_   = 0.0f;

    std::cout << "PID Controller initialized with Kp=" << kp_ << ", Ki=" << ki_
              << ", Kd=" << kd_ << ", speed=" << desired_speed_
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
    float manual_steering = xboxController_->getManualSteering();

    publisher_->publishSteering(manual_steering);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_1_LKAS
void PidController::LKASControl()
{
    static bool LKAS_enable = false;
    double current_time     = getCurrentTime();
    float manual_steering   = xboxController_->getManualSteering();
    float manual_speed      = xboxController_->getManualSpeed();
    double last_action      = 0.0;
    if (current_time - lastLaneProximityMeasure_ < 0.3)
    {
        if (LKASon && laneProximity_ < 0.35f && !LKAS_enable &&
            manual_speed > 10)
        {
            float direction = 110;
            publisher_->publishLaneAlert("Left");
            std::cout << "Lane Proximity: " << laneProximity_ << std::endl;
            publisher_->publishSteering(direction);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            LKAS_enable = true;
            LKASon      = false;
            last_action = getCurrentTime();
        }
        else if (LKASon && laneProximity_ > 0.65 && !LKAS_enable &&
                 manual_speed > 10)
        {
            float direction = 70;
            publisher_->publishLaneAlert("Right");
            std::cout << "Lane Proximity: " << laneProximity_ << std::endl;
            publisher_->publishSteering(direction);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            LKAS_enable = true;
            LKASon      = false;
            last_action = getCurrentTime();
        }
        else if (LKASon)
        {
            if (LKAS_enable)
            {
                if (getCurrentTime() - last_action < 1000)
                {
                    publisher_->publishLaneAlert("Off");
                    LKAS_enable = false;
                    LKASon      = false;
                }
            }
            publisher_->publishSteering(manual_steering);
        }
        publisher_->publishSpeed(manual_speed);
    }
    else
    {
        publisher_->publishSAELevelAttributionError("SAE_1_LKAS");
        std::cout << "SAE_1_LKAS: Lane Proximity measure timeout, LKAS control "
                     "not applied."
                  << std::endl;
        LKAS_enable = false;
        publisher_->publishLaneAlert("Off");
        manualControl();
        publisher_->publishSpeed(manual_speed);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_1_ACC
void PidController::adaptiveCruiseControl()
{
    float manual_steering = xboxController_->getManualSteering();

    publisher_->publishSteering(manual_steering);
    publisher_->publishDesiredSpeed(desired_speed_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// SAE_2
void PidController::partialControl()
{
    double current_time = getCurrentTime();

    float direction = steeringPID(cameraError_, current_time);

    publisher_->publishSteering(direction);

    if (!this->emergency_brake_)
    {
        publisher_->publishDesiredSpeed(desired_speed_);
        publisher_->publishCurrentGear(1);
    }
    else
    {
        publisher_->publishDesiredSpeed(0);
        publisher_->publishCurrentGear(0);
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(fixed_delta_time_ * 1000)));
}

// SAE_3
void PidController::conditionalAutomation()
{
    double current_time = getCurrentTime();

    float direction = steeringPID(cameraError_, current_time);

    publisher_->publishSteering(direction);

    if (!this->emergency_brake_)
    {
        publisher_->publishDesiredSpeed(desired_speed_);
        publisher_->publishCurrentGear(1);
    }
    else
    {
        publisher_->publishDesiredSpeed(0);
        publisher_->publishCurrentGear(0);
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(fixed_delta_time_ * 1000)));
}

// SAE_4
void PidController::autonomousControl()
{
    double current_time = getCurrentTime();

    float direction = steeringPID(cameraError_, current_time);

    publisher_->publishSteering(direction);

    if (!this->emergency_brake_)
    {
        publisher_->publishDesiredSpeed(desired_speed_);
        publisher_->publishCurrentGear(1);
    }
    else
    {
        publisher_->publishDesiredSpeed(0);
        publisher_->publishCurrentGear(0);
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(fixed_delta_time_ * 1000)));
}

void PidController::speedDefinition(void)
{
    double current_time   = getCurrentTime();
    double threshold      = 2.0;
    double red_threshold  = 5.0;
    double stop_threshold = 2.0;
    double active_speed   = speed_limit_;

    if (std::abs(current_time - last_acc_speed_receive_) < threshold)
    {
        if (acc_speed_ > speed_limit_)
            active_speed = speed_limit_;
        else
            active_speed = acc_speed_;
    }
    else
    {
        active_speed = speed_limit_;
    }

    static bool stopped = 0;
    static bool stop_signal = 0;
    if (std::abs(current_time - last_danger_received_) < threshold)
    {
        desired_speed_ = active_speed - 0.05;
    }
    else if (std::abs(current_time - last_crosswalk_received_) < threshold)
    {
        desired_speed_ = active_speed - 0.05;
    }
    else if (std::abs(current_time - last_yield_received_) < threshold)
    {
        desired_speed_ = active_speed - 0.05;
    }
    else if (std::abs(current_time - last_green_received_) < threshold)
    {
        desired_speed_ = active_speed;
    }
    else if (std::abs(current_time - last_yellow_received_ < threshold))
    {
        desired_speed_ = active_speed - 0.05;
    }
    else if (std::abs(current_time - last_red_received_) < red_threshold)
    {
        desired_speed_ = 0;
    }
    else if (std::abs(current_time - last_stop_received_) < stop_threshold)
    {
        static int counter = 1;

        // if (counter % 10 != 0)
        //     stop_active_ = !stop_active_;

        // counter++;

        // if (stop_active_ == true)
        //     desired_speed_ = 0;
        // else
        //     desired_speed_ = active_speed;

        
        //if stopped == 1 && speed == 0
        //go
        //else
        //desired_speed_ = 0;


        if (stopped && stop_signal && counter % 50 == 0)
        {
            desired_speed_ = active_speed;
            counter = 1;
        }
        else if (stopped && current_speed_ == 0)
        {
            desired_speed_ = active_speed;
            stop_signal = 1;
            counter++;
        }
        else
        {
            desired_speed_ = 0;
            stopped = 1;
        }

    }
    else
    {
        desired_speed_ = active_speed;
        stopped = 0;
        stop_signal = 0;
    }
}

void PidController::run()
{
    while (true)
    {
        if (xboxController_->getPidEnable())
        {
            std::string sae_level = getAutonomousDriveState();

            speedDefinition();

            if (sae_level.find("SAE_0") != std::string::npos)
            {
                manualControl();
            }
            else if (sae_level.find("SAE_1_LKAS") != std::string::npos)
            {
                LKASControl();
            }
            else if (sae_level.find("SAE_1_ACC") != std::string::npos &&
                     getCurrentTime() - lastLaneProximityMeasure_ < 0.3)
            {
                adaptiveCruiseControl();
            }
            else if (sae_level.find("SAE_2") != std::string::npos &&
                     getCurrentTime() - lastLaneProximityMeasure_ < 0.3)
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
                 std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}
