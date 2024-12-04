#include "../include/Motor.hpp"

Motor::Motor() {}

Motor::~Motor() {}

void Motor::init(PCA9685* motorPCA)
{
    this->m_MotorPCA = motorPCA;
}

void Motor::setSpeed(int speed)
{
    if (speed < 0)
    {
        uint16_t pulseWidth = -speed * 4095 / 100;
        m_MotorPCA->setDutyCicle(0, pulseWidth);
        m_MotorPCA->setGPIO(1, true);
        m_MotorPCA->setGPIO(2, false);
        m_MotorPCA->setGPIO(5, false);
        m_MotorPCA->setGPIO(6, true);
        m_MotorPCA->setDutyCicle(7, pulseWidth);
    }
    else
    {
        uint16_t pulseWidth = speed * 4095 / 100;
        m_MotorPCA->setDutyCicle(0, pulseWidth);
        m_MotorPCA->setGPIO(1, false);
        m_MotorPCA->setGPIO(2, true);
        m_MotorPCA->setGPIO(5, true);
        m_MotorPCA->setGPIO(6, false);
        m_MotorPCA->setDutyCicle(7, pulseWidth);
    }
}