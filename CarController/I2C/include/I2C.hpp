#pragma once

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

class I2C
{
  private:
    const std::string _i2cDevice;
    int _i2cFd;

  public:
    I2C(const std::string& i2cDevice);
    ~I2C();
    I2C(const I2C& originalI2C);
    I2C& operator=(const I2C& originalI2C);

    void writeByte(const std::string& deviceAddress, uint8_t reg,
                   uint8_t value);
};
