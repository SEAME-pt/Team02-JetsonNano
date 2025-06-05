#include "XboxController.hpp"
#include "MPController.hpp"
#include "PidController.hpp"

int main(int argc, char** argv)
{
    try
    {
        std::shared_ptr<zenoh::Session> session;
        if (argc == 2)
        {
            auto config = zenoh::Config::from_file(std::string(argv[1]));
            session     = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }
        else
        {
            auto config = zenoh::Config::create_default();
            session     = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }

        XboxController manualController(session);
        PidController pidController(session, &manualController);
        ModelPredictiveController MPController(session, &manualController);

        // PID controller values
        // float kp                = 130;
        // float ki                = 0.000001;
        // float kd                = 10;
        float kp                = 15;
        float ki                = 0.1;
        float kd                = 3.0;
        float constant_throttle = 250;
        float delta_time        = 0.05; // ms

        // MPC controller values
        size_t N  = 10;  // steps
        double L  = 2.9; // distance between axis
        double Ts = 0.1; // time between control actions

        Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
        Q(0,0) = 100.0;  // x position error weight
        Q(1,1) = 100.0;  // y position error weight
        Q(2,2) = 10.0;   // heading error weight
        Q(3,3) = 1.0;    // velocity error weight

        // Control input weight matrix (R)
        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        R(0,0) = 0.1;    // steering input weight
        R(1,1) = 10.0;       
        
        Eigen::Matrix4d Qf = Q;

        pidController.init(kp, ki, kd, constant_throttle, delta_time);
        MPController.init(N, L, Ts, Q, R, Qf);

        std::thread manualThread(&XboxController::run, &manualController);
        std::thread pidThread(&PidController::run, &pidController);
        std::thread MPCThread(&ModelPredictiveController::run, &MPController);
        manualThread.join();
        pidThread.join();
        MPCThread.join();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}