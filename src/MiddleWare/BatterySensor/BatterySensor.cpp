#include "BatterySensor.hpp"
#include <sys/stat.h>
#include <cmath>

BatterySensor::BatterySensor(std::shared_ptr<SensoringPublisher> publisher)
{
    publisher_ = publisher;
}

BatterySensor::~BatterySensor()
{
    if (this->batteryINA)
    {
        delete (batteryINA);
    }
    if (this->m_I2c)
    {
        delete (m_I2c);
    }
    if (this->canBus)
    {
        delete this->canBus;
    }
}

void BatterySensor::initLocalEnv(const std::string& i2cDevice,
                                 uint8_t sensorAddress,
                                 const std::string& canDevice)
{
    try
    {
        struct stat buffer;
        if (stat(i2cDevice.c_str(), &buffer) != 0)
        {
            std::cerr << "I2C device " << i2cDevice << " does not exist!"
                      << std::endl;
            this->m_I2c = NULL;
            throw std::runtime_error("Error on I2C device");
        }
        this->m_I2c = new I2C();
        this->m_I2c->init(i2cDevice);
    }
    catch (...)
    {
        std::cerr << "Error on initializing i2c" << std::endl;
        this->m_I2c = NULL;
    }

    try
    {
        if (m_I2c)
        {
            this->batteryINA = new INA219();
            this->batteryINA->init(m_I2c, sensorAddress);
        }
        else
        {
            throw std::runtime_error("Error on ina219 device");
        }
    }
    catch (...)
    {
        std::cerr << "Error on initializing ina" << std::endl;
        this->batteryINA = NULL;
    }

    try
    {
        // struct stat buffer;
        // if (stat(canDevice.c_str(), &buffer) != 0) {
        //     std::cerr << "Can device " << canDevice << " does not exist!" <<
        //     std::endl; this->canBus = NULL; throw std::runtime_error("Error
        //     on can device");
        // }
        this->canBus = new CAN();
        this->canBus->init(canDevice);
    }
    catch (...)
    {
        std::cerr << "Error on initializing can" << std::endl;
        this->canBus = NULL;
    }
}

void BatterySensor::initCarlaEnv()
{
    this->canBus     = NULL;
    this->m_I2c      = NULL;
    this->batteryINA = NULL;
}

void BatterySensor::run(void)
{
    double voltage_ema              = 0.0;
    const double alpha              = 0.1;
    float last_published_percentage = -1.0f;
    const float publish_threshold   = 1.0f;
    while (1)
    {
        usleep(100000);

        if (this->batteryINA)
        {
            double voltage = this->batteryINA->readVoltage(0x02);

            // Exponential moving average
            if (voltage_ema == 0.0)
                voltage_ema = voltage;
            else
                voltage_ema = alpha * voltage + (1 - alpha) * voltage_ema;

            if (this->canBus)
            {
                uint8_t value[8];
                memcpy(value, &voltage_ema, sizeof(value));
                this->canBus->writeMessage(0x02, value, sizeof(value));
            }

            float percentage = ((voltage_ema - 9.5f) / (12.6f - 9.5f)) * 100.0f;
            percentage       = std::min(100.0f, std::max(0.0f, percentage));

            // Only publish if change is significant
            if (fabs(percentage - last_published_percentage) >
                publish_threshold)
            {
                last_published_percentage = percentage;
                publisher_->publishStateOfCharge(percentage);
            }
        }
    }
    return;
}