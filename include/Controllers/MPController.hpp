#pragma once
 
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <thread>
#include "XboxController.hpp"
#include "SpeedPidController.hpp"
#include "zenoh.hxx"

# ifndef M_PI
# define M_PI 3.14159265358979323846
# endif

class ModelPredictiveController
{
private:
    std::shared_ptr<zenoh::Session> session_;
    std::unique_ptr<ControllerPublisher> publisher_;
    std::optional<zenoh::Subscriber<void>> coeffs_subscriber;
    std::optional<zenoh::Subscriber<void>> activeAutonomyLevel_subscriber;
    std::optional<zenoh::Subscriber<void>> speed_lock_subscriber;
    std::optional<zenoh::Subscriber<void>> currentSpeed_subscriber;

    XboxController* xboxController_;
    bool speed_lock_;
    float fixed_delta_time_;

    float lane_departure_threshold_;

    SpeedPidController* speedPidController_;
    float speedKp_;
    float speedKi_;
    float speedKd_;
    float current_speed_;
    float desired_speed_;
    float current_steering_;

    Eigen::Vector4d currentState_;
    std::string autonomousDrive_;

    double target_velocity_ = 8.0;

    float initial_v; //x and y start at zero and psi starts at 90 degrees
    size_t N_; //steps
    double L_; //distance between axis
    double Ts_; // time between control actions
    Eigen::Matrix4d Q_; //trajectory error costs
    Eigen::Matrix2d R_; //changes in control costs
    Eigen::Matrix4d Qf_; //terminal cost (outside steps)

    std::vector<double> trajectoryCoeffs;
    Eigen::VectorXd last_u_flat_;    // size = 2*N_
    double tau_v_ = 1.0;
    
    double w_ddelta_base_ = 250.0; // base weight for steering changes

public:
    ModelPredictiveController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller);
    ~ModelPredictiveController();

    void init(size_t horizon, double wheelbase, double Ts,
        const Eigen::Matrix4d& Q, const Eigen::Matrix2d& R, const Eigen::Matrix4d& Qf);

    void run();

    std::vector<Eigen::Vector4d> predicted_trajectory_;

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

    void solve(const Eigen::Vector4d& x0, const std::vector<double>& traj_coeffs);

    Eigen::Vector4d backwardEuler(const Eigen::Vector4d& x, const Eigen::Vector2d& u);

    void setAutonomousDriveState(std::string current_state);
    std::string getAutonomousDriveState() const;
};
