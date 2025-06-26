#pragma once

#include <memory>
#include <zenoh.hxx>
#include "ControllerPublisher.hpp"
#include "IVehicleController.hpp"
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

class SpeedPidController
{
private:
    
    std::shared_ptr<zenoh::Session> session_;
    std::optional<zenoh::Subscriber<void>> kp_subscriber;
    std::optional<zenoh::Subscriber<void>> ki_subscriber;
    std::optional<zenoh::Subscriber<void>> kd_subscriber;

    // PID constants
    float kp_; // Proportional gain
    float ki_; // Integral gain
    float kd_; // Derivative gain
    
    // PID variables
    float prev_error_;
    float cameraError_;
    float integral_;
    double last_time_;
    
    // Control parameters
    float max_speed_;
    float max_throttle_;

    float fixed_delta_time_;


public:
    SpeedPidController();
    ~SpeedPidController();
    
    void init(float kp, float ki, float kd, float delta_time, std::shared_ptr<zenoh::Session> session);
    
    float speedPID(float error, double current_time);
    float speedAdjustment(float error);

    void updateControl(float lane_error, double current_time);

    void setAutonomousDriveState(std::string current_state);
    std::string getAutonomousDriveState() const;

    void run(); // Main control loop
};
