#include "../include/RaceCar.hpp"

RaceCar::RaceCar()
{
    this->m_I2c   = new I2C;
    this->m_Motor = new PCA9685;
    this->m_Servo = new PCA9685;
}

RaceCar::~RaceCar()
{
    if (this->m_I2c)
    {
        delete this->m_I2c;
    }
    else
    {
        // nothing
    }

    if (this->m_Motor)
    {
        delete this->m_Motor;
    }
    else
    {
        // nothing
    }

    if (this->m_Servo)
    {
        delete this->m_Servo;
    }
    else
    {
        // nothing
    }
}

void RaceCar::init(const std::string& i2cDevice, uint8_t motorAddress,
                   uint8_t servoAddress)
{
    this->m_I2c->init(i2cDevice);
    this->m_Motor->init(m_I2c, motorAddress);
    this->m_Servo->init(m_I2c, servoAddress);
    this->m_Motor->setPWMFreq(1600);
    this->m_Servo->setPWMFreq(50);
}

void RaceCar::setDirection(uint8_t angle)
{
    uint16_t pulseWidth = static_cast<uint16_t>(205 + (angle * 205) / 180);
    m_Servo->setPWM(0, 0, pulseWidth);
}

void RaceCar::setSpeed(int speed)
{
    if (speed < 0)
    {
        uint16_t pulseWidth = -speed * 4095 / 100;
        m_Motor->setDutyCicle(0, pulseWidth);
        m_Motor->setGPIO(1, true);
        m_Motor->setGPIO(2, false);
        m_Motor->setGPIO(5, false);
        m_Motor->setGPIO(6, true);
        m_Motor->setDutyCicle(7, pulseWidth);
    }
    else
    {
        uint16_t pulseWidth = speed * 4095 / 100;
        m_Motor->setDutyCicle(0, pulseWidth);
        m_Motor->setGPIO(1, false);
        m_Motor->setGPIO(2, true);
        m_Motor->setGPIO(5, true);
        m_Motor->setGPIO(6, false);
        m_Motor->setDutyCicle(7, pulseWidth);
    }
}