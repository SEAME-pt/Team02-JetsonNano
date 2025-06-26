#pragma once

#include <zenoh.hxx>
#include <string>
#include <optional>

#include <memory>
#include <iostream>
#include <Publisher.hpp>

class PidCalibrator
{
    private:
        std::shared_ptr<zenoh::Session> session_;
        std::unique_ptr<Publisher> publisher_;

        float kp_;
        float ki_;
        float kd_;

        float fixed_delta_time_;
    public:
        PidCalibrator(std::shared_ptr<zenoh::Session> session);
        // ~PidCalibrator();
        void run();
};