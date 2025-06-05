#include "BatterySensor.hpp"
#include <sys/stat.h>

BatterySensor::BatterySensor(std::shared_ptr<SensoringPublisher> publisher)
{   
    publisher_ = publisher;
}

BatterySensor::~BatterySensor()
{
    if (this->batteryINA) {
        delete (batteryINA);
    }
    if (this->m_I2c) {
        delete (m_I2c);
    }
    if (this->canBus) {
        delete this->canBus;
    }
}

void BatterySensor::initLocalEnv(const std::string& i2cDevice, uint8_t sensorAddress,
                         const std::string& canDevice) {
    try {
        struct stat buffer;
        if (stat(i2cDevice.c_str(), &buffer) != 0) {
            std::cerr << "I2C device " << i2cDevice << " does not exist!" << std::endl;
            this->m_I2c = NULL;
            throw std::runtime_error("Error on I2C device");
        }
        this->m_I2c      = new I2C();
        this->m_I2c->init(i2cDevice);
    } catch (...) {
        std::cerr << "Error on initializing i2c" << std::endl;
        this->m_I2c = NULL;
    }

    try {
        if (m_I2c) {
            this->batteryINA = new INA219();
            this->batteryINA->init(m_I2c, sensorAddress);
        } else {
            throw std::runtime_error("Error on ina219 device");
        }
    } catch (...) {
        std::cerr << "Error on initializing ina" << std::endl;
        this->batteryINA = NULL;
    }

    try {
        struct stat buffer;
        if (stat(canDevice.c_str(), &buffer) != 0) {
            std::cerr << "Can device " << canDevice << " does not exist!" << std::endl;
            this->canBus = NULL;
            throw std::runtime_error("Error on can device");
        }
        this->canBus     = new CAN();
        this->canBus->init(canDevice);
    } catch (...) {
        std::cerr << "Error on initializing can" << std::endl;
        this->canBus = NULL;
    }
}

void BatterySensor::initCarlaEnv() {
    this->canBus = NULL;
    this->m_I2c = NULL;
    this->batteryINA = NULL;
}

void BatterySensor::run(void)
{
    double prev_voltage = 0;
    while (1)
    {
        usleep(100000);

        if (this->batteryINA) {
            double voltage = this->batteryINA->readVoltage(0x02);
            if (prev_voltage > 0 && abs(prev_voltage - voltage) > 0.04)
                voltage = prev_voltage;
    
            float alpha            = 0.01f;
            double smoothedVoltage = alpha * voltage + (1 - alpha) * voltage;

            if (this->canBus) {
                uint8_t value[8];
                memcpy(value, &smoothedVoltage, sizeof(value));

                this->canBus->writeMessage(0x02, value, sizeof(value));
            }

            float percentage = ((smoothedVoltage - 9.5f) / (12.6f - 9.5f)) * 100.0f;
            percentage       = std::min(100.0f, std::max(0.0f, percentage));
            std::string battery_str = std::to_string(percentage);
            publisher_->publishStateOfCharge(std::stof(battery_str));
            prev_voltage = voltage;
        }
    }
    return;
}