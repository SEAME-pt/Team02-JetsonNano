#include "BatterySensor.hpp"

BatterySensor::BatterySensor()
{
    this->m_I2c      = new I2C();
    this->batteryINA = new INA219();
    this->canBus     = new CAN();

    publisher_ = std::make_unique<SensoringPublisher>();
}

BatterySensor::~BatterySensor()
{
    delete (batteryINA);
    delete (m_I2c);
    delete this->canBus;
}

void BatterySensor::init(const std::string& i2cDevice, uint8_t sensorAddress,
                         const std::string& canDevice)
{
    this->m_I2c->init(i2cDevice);
    this->batteryINA->init(m_I2c, sensorAddress);
    this->canBus->init(canDevice);
}

void BatterySensor::run(void)
{
    double prev_voltage = 0;
    while (1)
    {
        usleep(100000);
        double voltage = this->batteryINA->readVoltage(0x02);
        if (prev_voltage > 0 && abs(prev_voltage - voltage) > 0.04)
            voltage = prev_voltage;

        float alpha            = 0.01f;
        double smoothedVoltage = alpha * voltage + (1 - alpha) * voltage;

        uint8_t value[8];
        memcpy(value, &smoothedVoltage, sizeof(value));

        this->canBus->writeMessage(0x02, value, sizeof(value));
        float percentage = ((smoothedVoltage - 9.5f) / (12.6f - 9.5f)) * 100.0f;
        percentage       = std::min(100.0f, std::max(0.0f, percentage));
        std::string battery_str = std::to_string(percentage);
        publisher_->publishStateOfCharge(std::stof(battery_str));
        prev_voltage = voltage;
    }
    return;
}