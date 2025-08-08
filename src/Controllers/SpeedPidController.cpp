#include "SpeedPidController.hpp"

static double getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

SpeedPidController::SpeedPidController(std::shared_ptr<zenoh::Session> session,
                                       XboxController* xbox_controller)
{
    prev_error_ = 0.0f;
    integral_   = 0.0f;
    last_time_  = 0.0f;

    max_speed_    = 90.0f;
    max_throttle_ = 40.0f;

    kp_ = 1.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;

    fixed_delta_time_ = 0.02f;
    autonomousDrive_  = "SAE_0";
    xboxController_   = xbox_controller;
    current_speed_    = 0.0f;

    std::cout << "SpeedPID controller created!" << std::endl;

    session_   = session;
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

    currentSpeed_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed",
        [this](const zenoh::Sample& sample)
        {
            float speed    = std::stof(sample.get_payload().as_string());
            current_speed_ = speed;

            if (logging_)
            {
                double now = getCurrentTime();
                log_file_ << (now - log_start_time_) << "," << current_speed_
                          << "," << "40" << "\n";

                if (now - log_start_time_ > 5.0)
                {
                    logging_ = false;
                    log_file_.close();
                }
            }

            last_measure_ = getCurrentTime();
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
            float steer = std::stof(sample.get_payload().as_string());
            steer_      = (steer - 90.0) / 90;
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

    prev_error_    = 0.0f;
    integral_      = 0.0f;
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
    // dt
    double dt = current_time - last_time_;

    // PID
    // Gain-scheduling index from steering (-1..1 -> 0..1)
    // float alpha = std::min(1.0f, std::fabs(steer_));
    // alpha = std::max(0.5f, alpha);
    // float alpha = 0;

    // Interpolate process model parameters
    // float Kp_model = (1.0f - alpha) * Kp_s_ + alpha * Kp_c_;
    // float tau_sch  = (1.0f - alpha) * tau_s_ + alpha * tau_c_;
    // float L_sch    = (1.0f - alpha) * L_s_   + alpha * L_c_;

    float Kp_model = Kp_c_;
    float tau_sch  = tau_c_;
    float L_sch    = L_c_;

    // IMC PID tuning
    float lambda = std::max(tau_sch * 0.5f, 2.0f * L_sch);
    float Kc     = tau_sch / (Kp_model * (L_sch + lambda));
    float Ti     = std::min(tau_sch, 4.0f * (L_sch + lambda));
    float Td     = tau_sch * L_sch / (2.0f * tau_sch + L_sch);

    // Update PID gains
    kp_ = Kc;
    ki_ = Kc / Ti;
    kd_ = Kc * Td;

    // std::cout << "ERROR : "<< error << std::endl;

    // PID terms with anti-windup
    float p_term = kp_ * error;
    integral_ += error * dt;
    // max_integral_ = 10 / ki_;
    // // std::cout << "Max Integral: " << max_integral_ << std::endl;
    // // Limit integral term to prevent windup
    // integral_   = std::clamp(integral_, -max_integral_, max_integral_);
    float i_term = ki_ * integral_;
    float d_term = kd_ * ((error - prev_error_) / dt);
    // std::cout << "ALPHA : " << alpha << std::endl;
    std::cout << "kp : " << kp_ << " | ki : " << ki_ << " | kd : " << kd_
              << std::endl;
    std::cout << "p_term : " << p_term << " | i_term : " << i_term
              << " | d_term : " << d_term << std::endl;

    float u_ff = a0_ + a1_ * desired_speed_;

    // Combine feed-forward and PID
    float throttle = u_ff + p_term + i_term + d_term;
    // float throttle = p_term + i_term + d_term;
    if (throttle >= 5)
    {
        throttle = std::max(throttle, a0_ - 5);
    }

    // std::cout << "dt : " << dt << " | P:" << p_term << " I:" << i_term << "
    // D:" << d_term
    //           << " FF:" << u_ff << " | Integral:" << integral_
    //           << " | Output:" << throttle << std::endl;

    last_time_ = current_time;
    throttle   = std::min(throttle, 55.0f);
    return throttle;
}

void SpeedPidController::run()
{
    while (true)
    {
        std::string sae_level = getAutonomousDriveState();
        // if (getCurrentTime() - last_measure_ > 0.3)
        // {
        //     publisher_->publishSpeed(0);
        //     std::cout << "Waiting for speed measurement..." << std::endl;
        //     std::this_thread::sleep_for(std::chrono::milliseconds(
        //         static_cast<int>(fixed_delta_time_ * 1000)));
        // } else {
        if (sae_level.find("SAE_0") != std::string::npos ||
            sae_level.find("SAE_1_LKAS") != std::string::npos)
        {
            float manual_speed = xboxController_->getManualSpeed();
            publisher_->publishSpeed(manual_speed);
            integral_  = 0;
            last_time_ = getCurrentTime();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        else
        {
            double current_time = getCurrentTime();
            float error         = desired_speed_ - current_speed_;
            double throttle     = speedPID(error, current_time);
            throttle            = std::max(0.0, throttle);
            std::cout << "Current Speed: " << current_speed_
                      << " | Desired Speed: " << desired_speed_
                      << " | Throttle: " << throttle << std::endl;
            publisher_->publishSpeed(throttle);
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(fixed_delta_time_ * 1000)));
            // runThrottleCalibration();
        }
        // }
    }

    // //calibration
    // while (true){
    //     std::string sae_level = getAutonomousDriveState();
    //     if (sae_level.find("SAE_4") != std::string::npos){
    //         break;
    //     }
    // }

    // double throttle = 30;
    // publisher_->publishSpeed(throttle);

    // // Wait for a trigger to increase throttle (could be a timer, button, or
    // code logic) std::this_thread::sleep_for(std::chrono::milliseconds(8000));

    // // Start logging
    // double now = getCurrentTime();
    // log_start_time_ = getCurrentTime();
    // log_file_.open("straight_speed_pid_log.csv");
    // log_file_ << "time,speed,throttle\n";
    // log_file_ << (now - log_start_time_) << "," << current_speed_ << "," <<
    // "30" << "\n"; logging_ = true; while (logging_) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(25)); // Log at
    //     40 Hz throttle = 40; publisher_->publishSpeed(throttle);
    // }
    // publisher_->publishSpeed(0);

    // calibration of minimum speed to make the wheels turn
    //  std::cout << "Starting minimum throttle calibration..." << std::endl;

    // double throttle = 0.0;
    // const double THROTTLE_INCREMENT = 1.0;  // Increase by 1% each step
    // const double TARGET_SPEED = 10.0;       // Target 10 RPM
    // const double MAX_THROTTLE = 50.0;       // Safety limit
    // const int WAIT_TIME_MS = 2000;          // Wait 2 seconds between
    // increments const int SPEED_CHECK_COUNT = 5;        // Check speed 5 times
    // before confirming

    // bool speed_achieved = false;

    // while (!speed_achieved && throttle <= MAX_THROTTLE) {
    //     // Set current throttle
    //     publisher_->publishSpeed(0);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     publisher_->publishSpeed(throttle);
    //     std::cout << "Testing throttle: " << throttle << "%" << std::endl;

    //     // Wait for system to stabilize
    //     std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME_MS));

    //     // Check if speed consistently above target
    //     int speed_above_target_count = 0;
    //     for (int i = 0; i < SPEED_CHECK_COUNT; i++) {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(300));
    //         if (current_speed_ > TARGET_SPEED) {
    //             speed_above_target_count++;
    //         }
    //         std::cout << "Speed check " << (i+1) << ": " << current_speed_ <<
    //         " RPM" << std::endl;
    //     }

    //     // If speed consistently above target, we found minimum throttle
    //     if (speed_above_target_count >= SPEED_CHECK_COUNT - 1) {
    //         speed_achieved = true;
    //         std::cout << "SUCCESS: Minimum throttle found at " << throttle
    //                   << "% for speed > " << TARGET_SPEED << " RPM" <<
    //                   std::endl;
    //         std::cout << "Final speed: " << current_speed_ << " RPM" <<
    //         std::endl;
    //     } else {
    //         // Increase throttle and try again
    //         throttle += THROTTLE_INCREMENT;
    //         std::cout << "Speed too low (" << current_speed_
    //                   << " RPM), increasing throttle..." << std::endl;
    //     }
    // }

    // if (!speed_achieved) {
    //     std::cout << "WARNING: Could not achieve " << TARGET_SPEED
    //               << " RPM with throttle up to " << MAX_THROTTLE << "%" <<
    //               std::endl;
    // }

    // // Stop the vehicle
    // publisher_->publishSpeed(0);
    // std::cout << "Calibration complete. Vehicle stopped." << std::endl;
}

void SpeedPidController::runThrottleCalibration()
{
    std::cout << "Starting throttle calibration..." << std::endl;

    // Throttle values to test
    std::vector<double> throttle_values = {15, 20, 21, 22, 23, 24, 25, 26,
                                           27, 28, 29, 30, 31, 32, 33, 34,
                                           35, 36, 37, 38, 39, 40};

    const int STABILIZATION_TIME_MS = 5000; // 5 seconds to stabilize
    const int LOGGING_TIME_MS       = 3000; // 3 seconds of data collection
    const int LOG_INTERVAL_MS       = 25;   // 40 Hz logging

    for (double test_throttle : throttle_values)
    {
        std::cout << "\n=== Testing throttle: " << test_throttle
                  << "% ===" << std::endl;

        // Stop vehicle first
        publisher_->publishSpeed(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        // Create filename for this throttle test
        std::string filename =
            "throttle_" + std::to_string((int)test_throttle) + "_speed_log.csv";
        std::ofstream log_file(filename);

        if (!log_file.is_open())
        {
            std::cerr << "Failed to open log file: " << filename << std::endl;
            continue;
        }

        // Write CSV header
        log_file << "time,speed,throttle\n";

        // Apply throttle
        publisher_->publishSpeed(test_throttle);
        std::cout << "Applied throttle: " << test_throttle
                  << "%, stabilizing..." << std::endl;

        // Wait for stabilization
        std::this_thread::sleep_for(
            std::chrono::milliseconds(STABILIZATION_TIME_MS));

        // Start data collection
        std::cout << "Starting data collection for " << LOGGING_TIME_MS / 1000
                  << " seconds..." << std::endl;
        double log_start_time   = getCurrentTime();
        double collection_start = getCurrentTime();

        while ((getCurrentTime() - collection_start) * 1000 < LOGGING_TIME_MS)
        {
            double now     = getCurrentTime();
            double elapsed = now - log_start_time;

            // Log current data
            log_file << elapsed << "," << current_speed_ << "," << test_throttle
                     << "\n";

            // Maintain throttle
            publisher_->publishSpeed(test_throttle);

            std::this_thread::sleep_for(
                std::chrono::milliseconds(LOG_INTERVAL_MS));
        }

        log_file.close();
        std::cout << "Data collection complete. Saved to: " << filename
                  << std::endl;
        std::cout << "Final speed: " << current_speed_ << " RPM" << std::endl;
    }

    // Stop vehicle
    publisher_->publishSpeed(0);
    std::cout << "\n=== Throttle calibration complete ===" << std::endl;
    std::cout << "All data files saved. Vehicle stopped." << std::endl;
}