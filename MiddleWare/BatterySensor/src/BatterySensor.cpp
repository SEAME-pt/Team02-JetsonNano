#include "BatterySensor.hpp"

BatterySensor::BatterySensor()
    : m_session(Session::open(std::move(Config::create_default()))),
      m_pubBattery(
          m_session.declare_publisher(KeyExpr("seame/car/1/batterySensor")))
{
    this->m_I2c      = new I2C();
    this->batteryINA = new INA219();
    this->canBus     = new CAN();
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
    static const size_t BUFFER_SIZE                      = 50;
    static std::array<double, BUFFER_SIZE> voltageBuffer = {0};
    static size_t bufferIndex                            = 0;

    while (1)
    {
        usleep(100000);
        double voltage = this->batteryINA->readVoltage(0x02);

        voltageBuffer[bufferIndex] = voltage;
        bufferIndex                = (bufferIndex + 1) % BUFFER_SIZE;

        double sum = 0.0;
        for (const auto& v : voltageBuffer)
        {
            sum += v;
        }
        double smoothedVoltage = sum / BUFFER_SIZE;

        std::cout << "Battery: " << smoothedVoltage << std::endl;

        uint8_t value[8];
        memcpy(value, &smoothedVoltage, sizeof(value));

        this->canBus->writeMessage(0x02, value, sizeof(value));
        float percentage = ((smoothedVoltage - 9.5f) / (12.6f - 9.5f)) * 100.0f;
        percentage       = std::min(100.0f, std::max(0.0f, percentage));
        std::string battery_str = std::to_string(percentage);
        this->m_pubBattery.put(battery_str);
    }
    return;
}