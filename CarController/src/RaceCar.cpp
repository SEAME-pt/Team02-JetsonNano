#include "RaceCar.hpp"

RaceCar::RaceCar()
    : m_session(Session::open(std::move(Config::create_default()))),
      m_subThrottle(m_session.declare_subscriber(
          "seame/car/1/throttle",
          [this](const Sample& sample)
          {
              int throttle = std::stoi(sample.get_payload().as_string());
              std::cout << "Sub throttle: " << throttle << std::endl;
              this->setThrottle(throttle);
          },
          closures::none)),
      m_subDirection(m_session.declare_subscriber(
          "seame/car/1/direction",
          [this](const Sample& sample)
          {
              int direction = std::stoi(sample.get_payload().as_string());
              std::cout << "Sub direction: " << direction << std::endl;
              this->setDirection(direction);
          },
          closures::none))
{
    this->m_I2c      = new I2C;
    this->m_motorPCA = new PCA9685;
    this->m_ServoPCA = new PCA9685;

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

    if (this->m_ServoPCA)
    {
        delete this->m_ServoPCA;
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

void RaceCar::init(const std::string& i2cDevice, uint8_t motorAddress,
                   uint8_t servoAddress)
{
    this->m_I2c->init(i2cDevice);
    this->m_motorPCA->init(this->m_I2c, motorAddress);
    this->m_ServoPCA->init(this->m_I2c, servoAddress);

    this->m_motorPCA->setPWMFreq(1600);
    this->m_ServoPCA->setPWMFreq(50);

    this->m_motorLeft.init(this->m_motorPCA, Motor::LEFT);
    this->m_motorRight.init(this->m_motorPCA, Motor::RIGHT);
    this->m_servo.init(this->m_ServoPCA);
}

void RaceCar::setDirection(uint8_t angle)
{
    m_servo.setDirection(angle);
}

void RaceCar::setThrottle(int speed)
{
    m_motorLeft.setThrottle(speed);
    m_motorRight.setThrottle(speed);
}

void RaceCar::run(void)
{
    std::cout << "Car running!" << std::endl;

    while (1)
    {
        usleep(10);
    }
}