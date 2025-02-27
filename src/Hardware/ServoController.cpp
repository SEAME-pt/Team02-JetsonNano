#include "ServoController.hpp"

ServoController::ServoController(std::shared_ptr<I2C> i2c)
{
    m_currAngle = 90;
    m_ServoPCA  = std::make_unique<PCA9685>();

    m_ServoPCA->init(i2c, 0x40);
    m_ServoPCA->setPWMFreq(50);
}

ServoController::~ServoController() {}

int16_t ServoController::getAngle(void) const
{
    return m_currAngle;
}

void ServoController::setAngle(int16_t angle)
{
    angle = std::clamp(static_cast<int>(angle), 0, 180);

    if (m_currAngle == angle)
    {
        return;
    }
    else
    {
        m_currAngle = angle;
    }

    uint16_t pulseWidth = static_cast<uint16_t>(205 + (angle * 205) / 180);
    m_ServoPCA->setPWM(0, 0, pulseWidth);
}
