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
 
class ModelPredictiveController {
    public:

        ModelPredictiveController(std::shared_ptr<zenoh::Session> session, XboxController* xbox_controller);
        ~ModelPredictiveController();

        void init(size_t horizon, double wheelbase, double Ts,
            const Eigen::Matrix4d& Q, const Eigen::Matrix2d& R, const Eigen::Matrix4d& Qf);

        void run();
        void solve(const Eigen::Vector4d& x0, const std::vector<double>& traj_coeffs);
        void setVehicleState(const Eigen::Vector4d& state);
        Eigen::Vector4d getVehicleState() const { return this->currentState_; }

        void setTargetVelocity(double velocity);

    private:

        std::shared_ptr<zenoh::Session> session_;
        std::unique_ptr<ControllerPublisher> publisher_;
        std::optional<zenoh::Subscriber<void>> coeffs_subscriber;
        std::optional<zenoh::Subscriber<void>> currentSpeed_subscriber;

        XboxController* xboxController_;
        bool speed_lock_;
        float fixed_delta_time_;
    
        SpeedPidController* speedPidController_;
        float speedKp_;
        float speedKi_;
        float speedKd_;
        float current_speed_;
        float current_steering_;

        Eigen::Vector4d currentState_; // [x, y, psi, v]
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

        //Backward Euler  discretization - dynamics(front axle development, considering no slip)
        Eigen::Vector4d backwardEuler(const Eigen::Vector4d& x, const Eigen::Vector2d& u);

};