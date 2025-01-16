#include "RaceCar.hpp"

RaceCar::RaceCar(Session &session) : m_session(session)
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

    this->motorLeft.init(this->m_motorPCA, Motor::LEFT);
    this->motorRight.init(this->m_motorPCA, Motor::RIGHT);
    this->servo.init(this->m_ServoPCA);
}

void RaceCar::setDirection(uint8_t angle)
{
    servo.setDirection(angle);
}

void RaceCar::setThrottle(int speed)
{
    motorLeft.setThrottle(speed);
    motorRight.setThrottle(speed);
}

void RaceCar::run(void)
{
    std::cout << "Car running!" << std::endl;

    auto throttle_handler = [this](const Sample& sample)
    {
        int throttle = std::stoi(sample.get_payload().as_string());
        std::cout << "Sub throttle: " << throttle << std::endl;
        this->setThrottle(throttle);
    };

    auto direction_handler = [this](const Sample& sample)
    {
        int direction = std::stoi(sample.get_payload().as_string());
        std::cout << "Sub direction: " << direction << std::endl;
        this->setDirection(direction);
    };

    auto subThrottle = this->m_session.declare_subscriber(
        "seame/car/1/throttle", throttle_handler, closures::none);

    auto subDirection = this->m_session.declare_subscriber(
        "seame/car/1/direction", direction_handler, closures::none);

    while (signalTo)
    {
        usleep(10);
    }
}