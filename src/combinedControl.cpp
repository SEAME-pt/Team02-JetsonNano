#include "XboxController.hpp"
#include "MPController.hpp"
#include "PidController.hpp"
#include "SpeedPidController.hpp"
#include "utils.hpp"

int main(int argc, char** argv)
{
    try
    {
        std::string configFile;
        std::string mode;

        if (!parseParameters(argc, argv, configFile, mode))
        {
            return -1;
        }

        double Qx = 100.0, Qy = 600.0, Qpsi = 300.0, Qv = 250.0,
               Rthrottle = 20.0, Rsteer = 40.0;
        for (int i = 1; i < argc; ++i)
        {
            if (std::string(argv[i]) == "--Qx" && i + 1 < argc)
                Qx = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Qy" && i + 1 < argc)
                Qy = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Qpsi" && i + 1 < argc)
                Qpsi = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Qv" && i + 1 < argc)
                Qv = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Rthrottle" && i + 1 < argc)
                Rthrottle = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Rsteer" && i + 1 < argc)
                Rsteer = std::stod(argv[++i]);
        }

        std::shared_ptr<zenoh::Session> session;
        if (!configFile.empty())
        {
            std::cout << "Using configuration from file: " << configFile
                      << std::endl;
            auto config = zenoh::Config::from_file(configFile);
            session     = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }
        else
        {
            std::cout << "Using default configuration" << std::endl;
            auto config = zenoh::Config::create_default();
            // config.insert_json5("listen/endpoints",
            // "[\"udp/100.117.122.95:7450\"]");
            // config.insert_json5("connect/endpoints",
            // "[\"udp/100.117.122.95:7447\"]");
            // config.insert_json5("listen/endpoints",
            // "[\"udp/127.0.0.1:7450\"]");
            // config.insert_json5("connect/endpoints",
            // "[\"udp/127.0.0.1:7447\"]");
            session = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }

        XboxController manualController(session);
        PidController pidController(session, &manualController);
        ModelPredictiveController MPController(session, &manualController);
        SpeedPidController speedPidController(session, &manualController);

        if (mode == "local")
        {
            std::cout << "Running in LOCAL mode" << std::endl;
            // PID controller values
            float kp             = 130;
            float ki             = 0.000001;
            float kd             = 10;
            float constant_speed = 0.33; // m/s
            float delta_time     = 0.05;

            int screen_height = 480;
            int screen_width  = 640;

            // MPC controller values
            size_t N  = 5;
            double L  = 0.15;
            double Ts = 0.2;

            Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
            Q(0, 0)           = Qx;
            Q(1, 1)           = Qy;
            Q(2, 2)           = Qpsi;
            Q(3, 3)           = Qv;

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0, 0)           = Rthrottle;
            R(1, 1)           = Rsteer;

            Eigen::Matrix4d Qf = 5 * Q;
            pidController.init(kp, ki, kd, constant_speed, delta_time);
            MPController.init(N, L, Ts, Q, R, Qf, screen_height, screen_width,
                              constant_speed, false);
            speedPidController.init(0.25, 0.005, 0.0005, delta_time);
        }
        else
        {
            std::cout << "Running in CARLA mode" << std::endl;
            // PID controller values
            float kp             = 20;
            float ki             = 0.2;
            float kd             = 4.0;
            float constant_speed = 10;
            float delta_time     = 0.05;

            int screen_height = 512;
            int screen_width  = 1024;
            // MPC controller values
            size_t N  = 5;
            double L  = 2.9;
            double Ts = 0.05;

            Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
            Q(0, 0)           = Qx;
            Q(1, 1)           = Qy;
            Q(2, 2)           = Qpsi;
            Q(3, 3)           = Qv;

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0, 0)           = Rthrottle;
            R(1, 1)           = Rsteer;

            Eigen::Matrix4d Qf = Q * 5; // Terminal cost, more aggressive
            pidController.init(kp, ki, kd, constant_speed, delta_time);
            MPController.init(N, L, Ts, Q, R, Qf, screen_height, screen_width,
                              constant_speed, true);
            speedPidController.init(0.25, 0.005, 0.0005, delta_time);
        }

        std::thread manualThread(&XboxController::run, &manualController);
        std::thread pidThread(&PidController::run, &pidController);
        // std::thread MPCThread(&ModelPredictiveController::run, &MPController);
        std::thread speedPidThread(&SpeedPidController::run,
                                   &speedPidController);

        if (manualThread.joinable())
        {
            manualThread.join();
        }

        if (pidThread.joinable())
        {
            pidThread.join();
        }

        // if (MPCThread.joinable())
        // {
        //     MPCThread.join();
        // }

        if (speedPidThread.joinable())
        {
            speedPidThread.join();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}