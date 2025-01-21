#include "BatterySensor.hpp"

BatterySensor::BatterySensor() : m_session(Session::open(std::move(Config::create_default()))),
      m_pubBattery(
          m_session.declare_publisher(KeyExpr("seame/car/1/batterySensor")))
{
    this->m_I2c = new I2C();
    this->batteryINA = new INA219();
    this->canBus = new CAN();

}

BatterySensor::~BatterySensor()
{
    delete (batteryINA);
    delete (m_I2c);
    delete this->canBus;

}

void BatterySensor::init(const std::string& i2cDevice, uint8_t sensorAddress, const std::string& canDevice)
{
    this->m_I2c->init(i2cDevice);
    this->batteryINA->init(m_I2c, sensorAddress);
    this->canBus->init(canDevice);
}

void BatterySensor::run( void )
{
    while(1)
    {
        usleep(10000);
        int buffer = this->canBus->checkReceive();
        if (buffer != -1) {
            this->canBus->readMessage(buffer);
        }

        usleep(1000); // Add a small delay to avoid busy waiting

        double status = this->batteryINA->readVoltage(0x02);
        char buf[sizeof(status)];
        memcpy(buf, &status, sizeof(status));
        std::cout << "Battery: " << status << std::endl;

        uint8_t value[8];
        memcpy(value, &status, sizeof(status));
        this->canBus->writeMessage(0x02, value, sizeof(tx));


        char tx[8];
        memcpy(tx, &status, sizeof(status));
        this->m_pubBattery.put(tx);
    }
    return;
}