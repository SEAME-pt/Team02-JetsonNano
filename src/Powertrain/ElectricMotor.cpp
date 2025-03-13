#include "ElectricMotor.hpp"

std::uint16_t ElectricMotor::get_max_power() const
{
    return max_power;
}

void ElectricMotor::set_max_power(const std::uint16_t value)
{
    this->max_power = value;
}

std::int32_t ElectricMotor::get_speed() const
{
    return speed;
}

void ElectricMotor::set_speed(const std::int32_t value)
{
    if (speed != value)
    {
        speed = value;
        notifySpeedChanged(value);
    }
}

float ElectricMotor::get_time_in_use() const
{
    return time_in_use;
}

void ElectricMotor::set_time_in_use(const float value)
{
    this->time_in_use = value;
}

void ElectricMotor::addObserver(std::shared_ptr<IVehicleObserver> observer)
{
    observers_.push_back(observer);
}

void ElectricMotor::notifySpeedChanged(int32_t speed)
{
    for (auto& observer : observers_)
    {
        observer->onSpeedChanged(speed);
    }
}