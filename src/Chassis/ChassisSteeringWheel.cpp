#include "ChassisSteeringWheel.hpp"

int16_t ChassisSteeringWheel::get_angle() const
{
    return angle;
}

void ChassisSteeringWheel::set_angle(const int16_t value)
{
    if (angle != value)
    {
        angle = value;
        notifyAngleChanged(value);
    }
}

void ChassisSteeringWheel::addObserver(
    std::shared_ptr<IVehicleObserver> observer)
{
    observers_.push_back(observer);
}

void ChassisSteeringWheel::notifyAngleChanged(float angle)
{
    for (auto& observer : observers_)
    {
        observer->onSteeringAngleChanged(angle);
    }
}