#include "MotorController.hpp"

MotorController::MotorController(std::shared_ptr<I2C> i2c)
{
    m_currSpeed = 0;
    m_MotorPCA  = std::make_unique<PCA9685>();

    m_MotorPCA->init(i2c, 0x60);
    m_MotorPCA->setPWMFreq(1600);
}

MotorController::~MotorController() {}

int32_t MotorController::getSpeed(void) const
{
    return m_currSpeed;
}

void MotorController::setSpeed(int32_t speed)
{
    speed = std::clamp(speed, -100, 100);

    if (speed == m_currSpeed)
    {
        return;
    }
    else
    {
        m_currSpeed = speed;
    }
    if (speed < 0)
    {
        uint16_t pulseWidth = static_cast<uint16_t>(-speed * 4095 / 100);

        m_MotorPCA->setDutyCicle(0, pulseWidth);
        m_MotorPCA->setGPIO(1, true);
        m_MotorPCA->setGPIO(2, false);

        m_MotorPCA->setGPIO(5, false);
        m_MotorPCA->setGPIO(6, true);
        m_MotorPCA->setDutyCicle(7, pulseWidth);
    }
    else
    {
        uint16_t pulseWidth = static_cast<uint16_t>(speed * 4095 / 100);

        m_MotorPCA->setDutyCicle(0, pulseWidth);
        m_MotorPCA->setGPIO(1, false);
        m_MotorPCA->setGPIO(2, true);

        m_MotorPCA->setGPIO(5, true);
        m_MotorPCA->setGPIO(6, false);
        m_MotorPCA->setDutyCicle(7, pulseWidth);
    }
}
