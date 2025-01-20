#pragma once

#include "Motor.hpp"
#include "Servo.hpp"
#include "PCA9685.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "zenoh.hxx"

extern int signalTo;

using namespace zenoh;

class RaceCar
{
  private:
    I2C* m_I2c;
    PCA9685* m_ServoPCA;
    PCA9685* m_motorPCA;
    Motor m_motorRight;
    Motor m_motorLeft;
    Servo m_servo;
    Session m_session;
    Subscriber<void> m_subThrottle;
    Subscriber<void> m_subDirection;

  private:
    void setDirection(uint8_t angle);
    void setThrottle(int speed);

  public:
    RaceCar(Session& session);
    ~RaceCar();

    void init(const std::string& i2cDevice, uint8_t motorAddress,
              uint8_t servoAddress);
    void run(void);
};