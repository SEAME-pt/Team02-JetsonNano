#pragma once

#include <memory>
#include <zenoh.hxx>
#include "ControllerPublisher.hpp"
#include "IVehicleController.hpp"
#include <iostream>
#include <chrono>
#include <thread>
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
    
    // PID constants
    float kp_; // Proportional gain
    float ki_; // Integral gain
    float kd_; // Derivative gain
    
    // PID variables
    float prev_error_;
    float cameraError_;
    float integral_;
    float last_time_;
    
    // Control parameters
    float constant_speed_; // Constant speed for the car
    float max_steering_angle_; // Maximum steering angle

    float fixed_delta_time_;

    bool autonomousDrive_;

public:
    PidController();
    PidController(const std::string& configFile);
    ~PidController();
    
    void init(float kp, float ki, float kd, float speed, float delta_time);
    void updateControl(float lane_error, float current_time);

    void setAutonomousDriveState(bool toggle);
    bool getAutonomousDriveState() const;

    void run(); // Main control loop
};
