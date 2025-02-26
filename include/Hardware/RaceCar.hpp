#pragma once

#include "Motor.hpp"
#include "Servo.hpp"
#include "PCA9685.hpp"
#include <unistd.h>
#include <iostream>
#include "zenoh.hxx"

using namespace zenoh;

class RaceCar
{
  private:
    I2C* m_I2c;
    PCA9685* m_servoPCA;
    PCA9685* m_motorPCA;
    Motor m_motorRight;
    Motor m_motorLeft;
    Servo m_servo;
    Session m_session;
    Subscriber<void> m_subThrottle;
    Subscriber<void> m_subDirection;

  public:
    RaceCar();
    ~RaceCar();

    I2C* getI2C(void) const;
    PCA9685* getServoPCA(void) const;
    PCA9685* getMotorPCA(void) const;
    Motor& getMotorRight(void);
    Motor& getMotorLeft(void);
    Servo& getServo(void);

    void init(const std::string& i2cDevice, uint8_t motorAddress,
              uint8_t servoAddress);
    void run(void);
};