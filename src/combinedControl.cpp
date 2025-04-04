#include "XboxController.hpp"
#include "PidController.hpp"

int main(int argc, char** argv)
{
    try
    {
        /*both cointrollers need to run with config files,
         otherwise no config file will be considered */
        std::unique_ptr<XboxController> manualController;
        std::unique_ptr<PidController> pidController;
        
        if (argc == 3)
        {
            auto manualController = std::make_unique<XboxController>(argv[1]);
            auto pidController = std::make_unique<PidController>(argv[2], manualController.get());
        }
        else
        {
            auto manualController = std::make_unique<XboxController>();
            auto pidController = std::make_unique<PidController> (manualController.get());
        }
        // PID controller values
        float kp                = 250;
        float ki                = 0.000001;
        float kd                = 20;
        float constant_throttle = 0.3;
        float delta_time        = 0.05; // ms

        pidController->init(kp, ki, kd, constant_throttle, delta_time);

        std::thread manualThread(&XboxController::run, manualController.get());
        std::thread pidThread(&PidController::run, pidController.get());
        manualThread.join();
        pidThread.join();

        // delete manualController;
        // delete pidController;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}