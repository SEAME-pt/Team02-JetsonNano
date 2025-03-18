#include "XboxController.hpp"
#include "PidController.hpp"

int main(int argc, char** argv)
{
    try
    {
        XboxController* manualController;
        PidController* pidController;

        /*both cointrollers need to run with config files,
         otherwise no config file will be considered */
        if (argc == 3)
        {
            manualController = new XboxController(argv[1]);
            pidController = new PidController(argv[2]);
        }
        else
        {
            manualController = new XboxController();
            pidController = new PidController();
        }
        float kp = 250;
        float ki = 0;
        float kd = 2;
        float constant_throttle = 0.25;
        float delta_time = 0.01; //ms
        pidController->init(kp, ki, kd, constant_throttle, delta_time);

        std::thread manualThread(&XboxController::run, manualController);
        std::thread pidThread(&PidController::run, pidController);
        manualThread.join();
        pidThread.join();

        delete manualController;
        delete pidController;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}