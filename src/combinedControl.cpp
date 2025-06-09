#include "XboxController.hpp"
#include "MPController.hpp"
#include "PidController.hpp"
#include "utils.hpp"

int main(int argc, char** argv)
{
    try
    {
        std::string configFile;
        std::string mode;
        
        if (!parseParameters(argc, argv, configFile, mode)) {
            return -1;
        }

        std::shared_ptr<zenoh::Session> session;
        if (!configFile.empty()) {
            std::cout << "Using configuration from file: " << configFile << std::endl;
            auto config = zenoh::Config::from_file(configFile);
            session = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }
        else
        {
            std::cout << "Using default configuration" << std::endl;
            auto config = zenoh::Config::create_default();
            // config.insert_json5("listen/endpoints", "[\"udp/100.117.122.95:7450\"]");
            // config.insert_json5("connect/endpoints", "[\"udp/100.117.122.95:7447\"]");
            session = std::make_shared<zenoh::Session>(
                zenoh::Session::open(std::move(config)));
        }

        XboxController manualController(session);
        PidController pidController(session, &manualController);
        ModelPredictiveController MPController(session, &manualController);

        if (mode == "local") {
            std::cout << "Running in LOCAL mode" << std::endl;
            // PID controller values
            float kp                = 130;
            float ki                = 0.000001;
            float kd                = 10;
            float constant_throttle = 100;
            float delta_time        = 0.05;

            // MPC controller values
            size_t N  = 10;
            double L  = 2.9;
            double Ts = 0.1;

            Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
            Q(0,0) = 100.0;
            Q(1,1) = 100.0;
            Q(2,2) = 10.0;
            Q(3,3) = 1.0;

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0,0) = 0.1;
            R(1,1) = 10.0;       
            
            Eigen::Matrix4d Qf = Q;
            pidController.init(kp, ki, kd, constant_throttle, delta_time);
            MPController.init(N, L, Ts, Q, R, Qf);
        } else {
            std::cout << "Running in CARLA mode" << std::endl;
            // PID controller values
            float kp                = 15;
            float ki                = 0.1;
            float kd                = 3.0;
            float constant_throttle = 100;
            float delta_time        = 0.05;

            // MPC controller values
            size_t N  = 10;
            double L  = 2.9;
            double Ts = 0.1;

            Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
            Q(0,0) = 100.0;
            Q(1,1) = 100.0;
            Q(2,2) = 10.0;
            Q(3,3) = 1.0;

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0,0) = 0.1;
            R(1,1) = 10.0;       
            
            Eigen::Matrix4d Qf = Q;
            pidController.init(kp, ki, kd, constant_throttle, delta_time);
            MPController.init(N, L, Ts, Q, R, Qf);
        }

        std::thread manualThread(&XboxController::run, &manualController);
        std::thread pidThread(&PidController::run, &pidController);
        std::thread MPCThread(&ModelPredictiveController::run, &MPController);

        if (manualThread.joinable()) {
            manualThread.join();
        }

        if (pidThread.joinable()) {
            pidThread.join();
        }

        if (MPCThread.joinable()) {
            MPCThread.join();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}