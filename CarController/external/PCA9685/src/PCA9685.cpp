#include "../include/PCA9685.hpp"

PCA9685::PCA9685() {}

PCA9685::~PCA9685()
{
    _i2c->writeByte(_deviceAddress, MODE1, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_ON_L, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_ON_H, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_OFF_L, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_OFF_H, 0x10);
}

PCA9685::PCA9685(const PCA9685& originalPCA9685)
{
    (void)originalPCA9685;
}

PCA9685& PCA9685::operator=(const PCA9685& originalPCA9685)
{
    (void)originalPCA9685;
    return *this;
}

void PCA9685::init(I2C* m_i2c, uint8_t deviceAddress)
{
    this->_i2c           = m_i2c;
    this->_deviceAddress = deviceAddress;
    _i2c->writeByte(_deviceAddress, MODE1, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_ON_L, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_ON_H, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_OFF_L, 0x00);
    _i2c->writeByte(_deviceAddress, ALL_LED_OFF_H, 0x10);
}

void PCA9685::setPWMFreq(float freq_hz)
{
    uint8_t prescale =
        static_cast<uint8_t>((25000000.0 / (4096 * freq_hz)) - 1.0);
    _i2c->writeByte(_deviceAddress, MODE1, 0x10);
    _i2c->writeByte(_deviceAddress, PRE_SCALE, prescale);
    _i2c->writeByte(_deviceAddress, MODE1, 0x80);
    usleep(5000);
}

void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    _i2c->writeByte(_deviceAddress, LED0_ON_L + 4 * channel, on & 0xFF);
    _i2c->writeByte(_deviceAddress, LED0_ON_H + 4 * channel, on >> 8);
    _i2c->writeByte(_deviceAddress, LED0_OFF_L + 4 * channel, off & 0xFF);
    _i2c->writeByte(_deviceAddress, LED0_OFF_H + 4 * channel, off >> 8);
}

void PCA9685::setDutyCicle(uint8_t channel, uint16_t pulseWidth)
{
    this->setPWM(channel, 0, pulseWidth % 0x1000);
}

void PCA9685::setGPIO(uint8_t channel, uint16_t on)
{
    this->setPWM(channel, 0x1000 * on, 0x1000 * !on);
}
