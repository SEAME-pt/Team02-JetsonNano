#include "../include/Motor.hpp"

Motor::Motor() : m_currThrottle(0) {}

Motor::~Motor() {}

PCA9685* Motor::getMotorPCA(void) const
{
    return m_MotorPCA;
}

int16_t Motor::getThrottle(void) const
{
    return m_currThrottle;
}

Motor::Side Motor::getSide(void) const
{
    return m_side;
}

void Motor::setThrottle(int throttle)
{
    throttle = std::clamp(throttle, -100, 100);

    if (throttle == m_currThrottle)
    {
        return;
    }
    else
    {
        m_currThrottle = throttle;
    }
    if (throttle < 0)
    {
        uint16_t pulseWidth = -throttle * 4095 / 100;
        if (this->m_side == RIGHT)
        {
            m_MotorPCA->setDutyCicle(0, pulseWidth);
            m_MotorPCA->setGPIO(1, true);
            m_MotorPCA->setGPIO(2, false);
        }
        else if (this->m_side == LEFT)
        {
            m_MotorPCA->setGPIO(5, false);
            m_MotorPCA->setGPIO(6, true);
            m_MotorPCA->setDutyCicle(7, pulseWidth);
        }
        else
        {
        }
    }
    else
    {
        uint16_t pulseWidth = throttle * 4095 / 100;
        if (this->m_side == RIGHT)
        {
            m_MotorPCA->setDutyCicle(0, pulseWidth);
            m_MotorPCA->setGPIO(1, false);
            m_MotorPCA->setGPIO(2, true);
        }
        else if (this->m_side == LEFT)
        {
            m_MotorPCA->setGPIO(5, true);
            m_MotorPCA->setGPIO(6, false);
            m_MotorPCA->setDutyCicle(7, pulseWidth);
        }
        else
        {
        }
    }
}

void Motor::init(PCA9685* motorPCA, Side side)
{
    this->m_MotorPCA = motorPCA;
    this->m_side     = side;
}
