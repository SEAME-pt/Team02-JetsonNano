#pragma once
 
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>

 
class ModelPredictiveController {
    public:
    
        struct Control
        {
            double steering;
            double throttle;
        }

        ModelPredictiveController(XboxController* xbox_controller);
        ModelPredictiveController(const std::string& configFile, XboxController* xbox_controller);
        ~ModelPredictiveController();

        void init(size_t horizon, double wheelbase, double Ts,
            const Eigen::Matrix4d& Q, const Eigen::Matrix2d& R, const Eigen::Matrix4d& Qf);

        void run(); // Main control loop

    private:

        std::shared_ptr<zenoh::Session> session_;
        std::unique_ptr<ControllerPublisher> publisher_;
        std::optional<zenoh::Subscriber<void>> trajectory_subscriber;
        std::optional<zenoh::Subscriber<void>> activeAutonomyLevel_subscriber;
        std::optional<zenoh::Subscriber<void>> speed_subscriber;

        std::string autonomousDrive_;

        float initial_v; //x and y start at zero and psi starts at 90 degrees
        size_t N_; //steps
        double L_; //distance between axis
        double Ts_; // time between control actions
        Eigen::Matrix4d Q_; //trajectory error costs
        Eigen::Matrix2d R_; //changes in control costs
        Eigen::Matrix4d Qf_; //terminal cost (outside steps)

        std::vector<double> trajectoryCoeffs;

        std::string autonomousDrive_;

        Control solve(const Eigen::Vector4d& x0, const std::vector<double>& traj_coeffs);

        //Backward Euler  discretization - dynamics(front axle development, considering no slip)
        Eigen::Vector4d backwardEuler(const Eigen::Vector4d& x, const Eigen::Vector2d& u);


        double computeCostconst std::vector<Eigen::Vector4d>& xTraj,
            const std::vector<Eigen::Vector2d>& uSeq,
            const std::vector<Eigen::Vector4d>& xRef,
            const std::vector<Eigen::Vector2d>& uRef;
            
        void computeControlInputs();

};
 
#endif