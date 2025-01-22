
#include "Signals.hpp"

Signals::Signals()
    : m_session(Session::open(std::move(Config::create_default()))),
      m_pubSpeed(
          m_session.declare_publisher(KeyExpr("seame/car/1/speedSensor"))),
      m_subLights(m_session.declare_subscriber(
          "seame/car/1/lights",
          [this](const Sample& sample)
          {
              uint8_t data =
                  static_cast<uint8_t>(sample.get_payload().as_string()[0]);
              this->canBus->writeMessage(0x03, &data, sizeof(data));
          },
          closures::none)),
      m_subGear(m_session.declare_subscriber(
          "seame/car/1/gear",
          [this](const Sample& sample)
          {
              uint8_t gear =
                  static_cast<uint8_t>(sample.get_payload().as_string()[0]);
              uint8_t data = gear & 0x0F;
              this->canBus->writeMessage(0x04, &data, sizeof(data));
          },
          closures::none))
{
    this->canBus = new CAN();
}

Signals::~Signals() {}

void Signals::init(const std::string& canDevice)
{
    this->canBus->init(canDevice);
}

void Signals::run()
{
    while (1)
    {
        usleep(100000);
        int buffer = this->canBus->checkReceive();
        if (buffer != -1)
        {
            uint32_t can_id;
            uint8_t data[8];
            this->canBus->readMessage(buffer, can_id, data);
            if (can_id == 0x01)
            {
                int speed;
                double wheelDiame = 0.067;

                memcpy(&speed, &data[1], 4);

                speed                 = ntohl(speed);
                speed                 = wheelDiame * 3.14 * speed * 10 / 60;
                std::string speed_str = std::to_string(speed);

                printf("Publishing speed: '%d'\n", speed);
                m_pubSpeed.put(speed_str.c_str());
            }
        }
    }
}
