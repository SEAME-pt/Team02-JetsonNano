#include "XboxController.hpp"
#include "MPController.hpp"
// #include "PidController.hpp"

int main(int argc, char** argv)
{
    try
    {
        XboxController* manualController;
        // PidController* pidController;
        ModelPredictiveController* MPController;

        /*both controllers need to run with config files,
         otherwise no config file will be considered */
        if (argc == 3)
        {
            manualController = new XboxController(argv[1]);
            // pidController    = new PidController(argv[2], manualController);
            MPController =
                new ModelPredictiveController(argv[2], manualController);
        }
        else
        {
            manualController = new XboxController();
            // pidController    = new PidController(manualController);
            MPController = new ModelPredictiveController(manualController);
        }
        // PID controller values
        // float kp                = 130;
        // float ki                = 0.000001;
        // float kd                = 10;
        // float constant_throttle = 0.2;
        // float delta_time        = 0.05; // ms

        // MPC controller values
        size_t N  = 10;  // steps
        double L  = 2.5; // distance between axis
        double Ts = 0.1; // time between control actions
        Eigen::Matrix4d Q =
            Eigen::Matrix4d::Identity(); // trajectory error costs
        Eigen::Matrix2d R =
            Eigen::Matrix2d::Identity(); // changes in control costs
        Eigen::Matrix4d Qf = Q;

        // pidController->init(kp, ki, kd, constant_throttle, delta_time);
        MPController->init(N, L, Ts, Q, R, Qf);

        std::thread manualThread(&XboxController::run, manualController);
        // std::thread pidThread(&PidController::run, pidController);
        std::thread MPCThread(&ModelPredictiveController::run, MPController);
        manualThread.join();
        // pidThread.join();
        MPController.join();

        delete manualController;
        // delete pidController;
        delete MPController;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}