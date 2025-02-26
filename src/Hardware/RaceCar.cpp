#include "RaceCar.hpp"

RaceCar::RaceCar()
    : m_session(Session::open(std::move(Config::create_default()))),
      m_subThrottle(m_session.declare_subscriber(
          "seame/car/1/throttle",
          [this](const Sample& sample)
          {
              int throttle = std::stoi(sample.get_payload().as_string());
              std::cout << "Sub throttle: " << throttle << std::endl;
              this->getMotorLeft().setThrottle(throttle);
              this->getMotorRight().setThrottle(throttle);
          },
          closures::none)),
      m_subDirection(m_session.declare_subscriber(
          "seame/car/1/direction",
          [this](const Sample& sample)
          {
              int direction = std::stoi(sample.get_payload().as_string());
              std::cout << "Sub direction: " << direction << std::endl;
              this->getServo().setDirection(direction);
          },
          closures::none))
{
    this->m_I2c      = new I2C;
    this->m_motorPCA = new PCA9685;
    this->m_servoPCA = new PCA9685;

    std::cout << "Car created!" << std::endl;
}

RaceCar::~RaceCar()
{
    if (this->m_motorPCA)
    {
        delete this->m_motorPCA;
    }
    else
    {
        // nothing
    }

    if (this->m_servoPCA)
    {
        delete this->m_servoPCA;
    }
    else
    {
        // nothing
    }
    if (this->m_I2c)
    {
        delete this->m_I2c;
    }
    else
    {
        // nothing
    }
    this->m_session.close();
}

I2C* RaceCar::getI2C(void) const
{
    return m_I2c;
}

PCA9685* RaceCar::getServoPCA(void) const
{
    return m_servoPCA;
}

PCA9685* RaceCar::getMotorPCA(void) const
{
    return m_motorPCA;
}

Motor& RaceCar::getMotorLeft(void)
{
    return m_motorLeft;
}

Motor& RaceCar::getMotorRight(void)
{
    return m_motorRight;
}

Servo& RaceCar::getServo(void)
{
    return m_servo;
}

void RaceCar::init(const std::string& i2cDevice, uint8_t motorAddress,
                   uint8_t servoAddress)
{
    this->m_I2c->init(i2cDevice);
    this->m_motorPCA->init(this->m_I2c, motorAddress);
    this->m_servoPCA->init(this->m_I2c, servoAddress);

    this->m_motorPCA->setPWMFreq(1600);
    this->m_servoPCA->setPWMFreq(50);

    this->m_motorLeft.init(this->m_motorPCA, Motor::LEFT);
    this->m_motorRight.init(this->m_motorPCA, Motor::RIGHT);
    this->m_servo.init(this->m_servoPCA);
}

void RaceCar::run(void)
{
    std::cout << "Car running!" << std::endl;

    while (1)
    {
        usleep(10);
    }
}
