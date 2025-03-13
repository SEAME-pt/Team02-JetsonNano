#pragma once

#include <memory>
#include <zenoh.hxx>
#include "ControllerPublisher.hpp"
#include <iostream>
#include <chrono>
#include <thread>

class PidController {
private:
    std::shared_ptr<zenoh::Session> session_;
    std::unique_ptr<ControllerPublisher> publisher_;
    
    // PID constants
    float kp_; // Proportional gain
    float ki_; // Integral gain
    float kd_; // Derivative gain
    
    // PID variables
    float prev_error_;
    float integral_;
    float last_time_;
    
    // Control parameters
    float constant_speed_; // Constant speed for the car
    float max_steering_angle_; // Maximum steering angle

    float fixed_delta_time_;
    
public:
    PidController();
    PidController(const std::string& configFile);
    ~PidController();
    
    void init(float kp, float ki, float kd, float speed, float delta_time);
    void updateControl(float lane_error, float current_time);
    void run(); // Main control loop
};
