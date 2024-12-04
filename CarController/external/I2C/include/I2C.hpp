#pragma once

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

class I2C
{
  private:
    std::string _i2cDevice;
    int _i2cFd;

  public:
    I2C();
    ~I2C();
    I2C(const I2C& originalI2C);
    I2C& operator=(const I2C& originalI2C);

    void init(const std::string& i2cDevice);
    void writeByte(int deviceAddress, uint8_t reg,
                   uint8_t value);
};
