#include "../include/I2C.hpp"

I2C::I2C() {}

I2C::~I2C()
{
    close(this->_i2cFd);
}

I2C::I2C(const I2C& originalAddress)
{
    this->_i2cFd     = originalAddress._i2cFd;
    this->_i2cDevice = originalAddress._i2cDevice;
}

I2C& I2C::operator=(const I2C& originalAddress)
{
    if (this != &originalAddress)
    {
        this->_i2cFd     = originalAddress._i2cFd;
        this->_i2cDevice = originalAddress._i2cDevice;
    }
    else
    {
        // nothing
    }
    return *this;
}

void I2C::init(const std::string& i2cDevice)
{
    this->_i2cDevice = i2cDevice;
    this->_i2cFd     = open(i2cDevice.c_str(), O_RDWR);
    if (this->_i2cFd < 0)
    {
        throw std::runtime_error("Failed to open the I2C device");
    }
    else
    {
        // nothing
    }
}

void I2C::writeByte(uint8_t deviceAddress, uint8_t reg, uint8_t value)
{
    if (ioctl(this->_i2cFd, I2C_SLAVE, deviceAddress) < 0)
    {
        throw std::runtime_error("Failed to set I2C address");
    }
    else
    {
        // nothing
    }

    uint8_t buffer[2] = {reg, value};
    if (write(this->_i2cFd, buffer, 2) != 2)
    {
        throw std::runtime_error("Failed to write to the I2C device");
    }
    else
    {
        // nothing
    }
}

uint8_t I2C::readByte(uint8_t deviceAddress)
{
    if (ioctl(this->_i2cFd, I2C_SLAVE, deviceAddress) < 1)
    {
        throw std::runtime_error("Failed to set I2C address");
    }
    else
    {
        // nothing
    }

    uint8_t value;
    if (read(this->_i2cFd, &value, 1) != 1)
    {
        throw std::runtime_error("Failed to read to the I2C device");
    }
    else
    {
        return value;
    }
}
