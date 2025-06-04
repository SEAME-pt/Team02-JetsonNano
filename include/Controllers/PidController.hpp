#pragma once

#include <memory>
#include <zenoh.hxx>
#include "ControllerPublisher.hpp"
#include "IVehicleController.hpp"
#include "SpeedPidController.hpp"
#include "XboxController.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <sys/time.h>

#ifdef TEST_MODE
  // Declare your custom functions
  extern "C" int custom_xbox_open(const char* path, int flags);
  extern "C" int custom_xbox_close(int fd);
  extern "C" int custom_xbox_ioctl(int fd, unsigned long request, int* arg);
  extern "C" ssize_t custom_xbox_read(int fd, void* buf, size_t count);
  extern "C" ssize_t custom_xbox_write(int fd, const void* buf, size_t count);
#endif

class PidController
{
private:
    std::shared_ptr<zenoh::Session> session_;
    std::unique_ptr<ControllerPublisher> publisher_;
    std::optional<zenoh::Subscriber<void>> kp_subscriber;
    std::optional<zenoh::Subscriber<void>> ki_subscriber;
    std::optional<zenoh::Subscriber<void>> kd_subscriber;
    std::optional<zenoh::Subscriber<void>> cameraError_subscriber;
    std::optional<zenoh::Subscriber<void>> activeAutonomyLevel_subscriber;
    std::optional<zenoh::Subscriber<void>> currentSpeed_subscriber;
    std::optional<zenoh::Subscriber<void>> speed_lock_subscriber;
    
    // PID constants
    float kp_; // Proportional gain
    float ki_; // Integral gain
    float kd_; // Derivative gain
    
    SpeedPidController* speedPidController_;
    float speedKp_;
    float speedKi_;
    float speedKd_;
    float current_speed_;

    // PID variables
    float prev_error_;
    float cameraError_;
    float integral_;
    double last_time_;
    
    // Control parameters
    float constant_speed_; // Constant speed for the car
    float max_steering_angle_; // Maximum steering angle

    float fixed_delta_time_;

    std::string autonomousDrive_;
    XboxController* xboxController_;

    bool speed_lock_;

    //SAE Levels
    //SAE1
      //For LKAS, if the error is above a value, PID will take control
    float lane_departure_threshold_ = 0.1f;


public:
    PidController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller);
    ~PidController();
    
    void init(float kp, float ki, float kd, float speed, float delta_time);
    
    float steeringPID(float error, double current_time);
    float speedAdjustment(float error);

    //SAE_1
    void LKASControl(float lane_error, double current_time, float manual_steering, float manual_speed);
    void adaptiveCruiseControl(float lane_error, double current_time, float manual_steering, float manual_speed);
    
    //SAE_2
    void partialControl(float lane_error, double current_time);

    //SAE_3
    void conditionalAutomation(float lane_error, double current_time);

    //SAE_4
    void updateControl(float lane_error, double current_time);

    void setAutonomousDriveState(std::string current_state);
    std::string getAutonomousDriveState() const;

    void run(); // Main control loop
};
