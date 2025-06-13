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

        double Qx = 0.6268, Qy = 0.1604, Qpsi = 5.4687, Qv = 0.1999, Rthrottle = 0.0050, Rsteer = 0.1681;
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--Qx" && i+1 < argc) Qx = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Qy" && i+1 < argc) Qy = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Qpsi" && i+1 < argc) Qpsi = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Qv" && i+1 < argc) Qv = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Rthrottle" && i+1 < argc) Rthrottle = std::stod(argv[++i]);
            if (std::string(argv[i]) == "--Rsteer" && i+1 < argc) Rsteer = std::stod(argv[++i]);
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
            Q(0,0) = Qx;
            Q(1,1) = Qy;
            Q(2,2) = Qpsi;
            Q(3,3) = Qv;

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0,0) = Rthrottle;
            R(1,1) = Rsteer;
            
            Eigen::Matrix4d Qf = 2 * Q;
            pidController.init(kp, ki, kd, constant_throttle, delta_time);
            MPController.init(N, L, Ts, Q, R, Qf);
        } else {
            std::cout << "Running in CARLA mode" << std::endl;
            // PID controller values
            float kp                = 20;
            float ki                = 0.2;
            float kd                = 4.0;
            float constant_throttle = 30;
            float delta_time        = 0.05;

            // MPC controller values
            size_t N  = 10;
            double L  = 2.9;
            double Ts = 0.1;

            Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
            Q(0,0) = 1.0 / (25.0 * 25.0);   // x error, expect up to 25 units
            Q(1,1) = 1.0 / (10.0 * 10.0);   // y error, expect up to 10 units
            Q(2,2) = 1.0 / (0.05 * 0.05);     // psi error, expect up to 0.5 rad
            Q(3,3) = 1.0 / (8.0 * 8.0);     // v error, expect up to 8 m/s

            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0,0) = 1.0 / (1.0 * 1.0);     // throttle, expect up to 1.0
            R(1,1) = 1.0 / (0.2 * 0.2);     // steering, expect up to 0.2 rad

            Eigen::Matrix4d Qf = Q * 10;       // Terminal cost, more aggressive
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