#include "XboxController.hpp"
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

        // PID controller values
        // float kp                = 130;
        // float ki                = 0.000001;
        // float kd                = 10;
        float kp                = 15;
        float ki                = 0.1;
        float kd                = 3.0;
        float constant_throttle = 0.2;
        float delta_time        = 0.05; // ms

        pidController.init(kp, ki, kd, constant_throttle, delta_time);

        std::thread manualThread(&XboxController::run, &manualController);
        std::thread pidThread(&PidController::run, &pidController);
        manualThread.join();
        pidThread.join();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}