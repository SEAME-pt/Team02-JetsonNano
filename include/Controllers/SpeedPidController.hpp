#pragma once

#include <memory>
#include <zenoh.hxx>
#include "ControllerPublisher.hpp"
#include "IVehicleController.hpp"
#include "XboxController.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <sys/time.h>

//calibratrion
#include <fstream>

#ifdef TEST_MODE
  // Declare your custom functions
  extern "C" int custom_xbox_open(const char* path, int flags);
  extern "C" int custom_xbox_close(int fd);
  extern "C" int custom_xbox_ioctl(int fd, unsigned long request, int* arg);
  extern "C" ssize_t custom_xbox_read(int fd, void* buf, size_t count);
  extern "C" ssize_t custom_xbox_write(int fd, const void* buf, size_t count);
#endif

# ifndef M_PI
# define M_PI 3.14159265358979323846
# endif

class SpeedPidController
{
private:
    
    std::shared_ptr<zenoh::Session> session_;
    std::unique_ptr<ControllerPublisher> publisher_;
    std::optional<zenoh::Subscriber<void>> kp_subscriber;
    std::optional<zenoh::Subscriber<void>> ki_subscriber;
    std::optional<zenoh::Subscriber<void>> kd_subscriber;
    std::optional<zenoh::Subscriber<void>> activeAutonomyLevel_subscriber;
    std::optional<zenoh::Subscriber<void>> currentSpeed_subscriber;
    std::optional<zenoh::Subscriber<void>> desiredSpeed_subscriber;
    std::optional<zenoh::Subscriber<void>> currentYaw_subscriber;
    std::optional<zenoh::Subscriber<void>> adaptiveCruiseControlSpeed_subscriber;

    //calibration parameters
    // Straight-line FOPDT model
    float Kp_s_= 5.97;
    float tau_s_ = 0.50;
    float L_s_ = 0.00;
    // Cornering FOPDT model
    float Kp_c_ = 100;
    float tau_c_ = 0.5574;
    float L_c_ = 0.1750;

    // // float a0_ = 12.93f;
    // float a0_ = 14.93f;
    // float a1_ = 0.130f;
    // // Saturation limits
    
    // PID constants
    float kp_; // Proportional gain
    float ki_; // Integral gain
    float kd_; // Derivative gain
    float desired_speed_;
    float current_speed_;
    float acc_speed_;
    
    // PID variables
    float prev_error_;
    float speedError_;
    float integral_;
    double last_time_;
    
    // Control parameters
    float max_speed_;
    float max_throttle_;
    float max_integral_ = 10.0;
    float fixed_delta_time_;

    std::string autonomousDrive_;
    XboxController* xboxController_;


    //calibration measurement parameters
    std::ofstream log_file_;
    bool logging_ = false;
    double log_start_time_ = 0.0;

    float steer_ = 0.0;

    float prev_throttle_ = 0.0f;
    const float MAX_THROTTLE_RATE = 20.0f;
    float u_stiction = 30.0;

    double last_measure_ = 0.0;




public:
    SpeedPidController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller);
    ~SpeedPidController();
    
    void init(float kp, float ki, float kd, float delta_time);

    void setAutonomousDriveState(std::string current_state);
    std::string getAutonomousDriveState() const;
    
    float speedPID(float error, double current_time);

    void runThrottleCalibration();

    void run();
};
