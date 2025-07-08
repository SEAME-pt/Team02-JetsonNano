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

class PidController
{
private:
    std::shared_ptr<zenoh::Session> session_;
    std::unique_ptr<ControllerPublisher> publisher_;
    std::optional<zenoh::Subscriber<void>> cameraError_subscriber;
    std::optional<zenoh::Subscriber<void>> activeAutonomyLevel_subscriber;
    std::optional<zenoh::Subscriber<void>> emergency_brake_subscriber_;
    
    float kp_;
    float ki_;
    float kd_;
    

    float prev_error_;
    float cameraError_;
    float integral_;
    double last_time_;
    
    float desired_speed_;
    float max_steering_angle_;

    float fixed_delta_time_;

    std::string autonomousDrive_;
    XboxController* xboxController_;

    bool emergency_brake_;

    float lane_departure_threshold_;

private:
    //SAE_0
    void manualControl();

    //SAE_1_LKAS
    void LKASControl();
    
    // SAE_1_ACC
    void adaptiveCruiseControl();
    
    //SAE_2
    void partialControl();

    //SAE_3
    void conditionalAutomation();

    //SAE_4
    void autonomousControl();

public:
    PidController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller);
    ~PidController();
    
    void init(float kp, float ki, float kd, float speed, float delta_time);
    
    float steeringPID(float error, double current_time);

    void setAutonomousDriveState(std::string current_state);
    std::string getAutonomousDriveState() const;

    void run();
};
