#include "pidCalibrator.hpp"

PidCalibrator::PidCalibrator(std::shared_ptr<zenoh::Session> session)
    : session_(session), kp_(0.0f), ki_(0.0f), kd_(0.0f),
      fixed_delta_time_(0.1f)
{
    publisher_ = std::make_unique<Publisher>(session_);
}

void PidCalibrator::run()
{
    std::cout << "PID Calibrator is running..." << std::endl;
    std::cout << "Enter PID values (kp, ki, kd) or 'exit' to quit:"
              << std::endl;
    std::string input;
    while (true)
    {
        std::cout << "PID> ";
        std::getline(std::cin, input);
        if (input == "exit")
        {
            break;
        }
        try
        {
            size_t pos1 = input.find(',');
            size_t pos2 = input.find(',', pos1 + 1);
            if (pos1 == std::string::npos || pos2 == std::string::npos)
            {
                throw std::invalid_argument(
                    "Invalid input format. Use: kp,ki,kd");
            }
            kp_ = std::stof(input.substr(0, pos1));
            ki_ = std::stof(input.substr(pos1 + 1, pos2 - pos1 - 1));
            kd_ = std::stof(input.substr(pos2 + 1));
            std::cout << "Setting PID values: kp=" << kp_ << ", ki=" << ki_
                      << ", kd=" << kd_ << std::endl;
            publisher_->publishSpeedKp(kp_);
            publisher_->publishSpeedKi(ki_);
            publisher_->publishSpeedKd(kd_);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
            std::cout << "Please enter valid PID values." << std::endl;
        }
    }
    std::cout << "PID Calibrator stopped." << std::endl;
}