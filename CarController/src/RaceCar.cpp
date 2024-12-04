#include "RaceCar.hpp"

RaceCar::RaceCar()
{
    this->m_I2c      = new I2C;
    this->m_motorPCA = new PCA9685;
    this->m_ServoPCA = new PCA9685;
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

    if (this->m_motorPCA)
    {
        delete this->m_motorPCA;
    }
    else
    {
        // nothing
    }

    if (this->m_ServoPCA)
    {
        delete this->m_ServoPCA;
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
    this->m_motorPCA->init(m_I2c, motorAddress);
    this->m_ServoPCA->init(m_I2c, servoAddress);

    this->motorLeft.init(m_motorPCA);
    this->motorRight.init(m_motorPCA);
    this->servo.init(m_ServoPCA);
}

void RaceCar::setDirection(uint8_t angle)
{
    servo.setDirection(angle);
}

void RaceCar::setSpeed(int speed)
{
    motorLeft.setSpeed(speed);
    motorRight.setSpeed(speed);
}