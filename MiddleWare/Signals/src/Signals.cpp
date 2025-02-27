
#include "Signals.hpp"

Signals::Signals()
    : m_session(Session::open(std::move(Config::create_default()))),
      m_pubSpeed(
          m_session.declare_publisher(KeyExpr("seame/car/1/speedSensor"))),
      m_subLights(m_session.declare_subscriber(
          "seame/car/1/lights",
          [this](const Sample& sample)
          {
              uint8_t data[1];
              data[0] =
                  static_cast<uint8_t>(sample.get_payload().as_string()[0]);
              printf("Can send lights: ");
              for (int i = 7; i >= 0; i--)
              {
                  printf("%d", (data[0] >> i) & 0x01);
              }
              printf("\n");
              this->canBus->writeMessage(0x03, data, sizeof(data));
          },
          closures::none)),
      m_subGear(m_session.declare_subscriber(
          "seame/car/1/gear",
          [this](const Sample& sample)
          {
              uint8_t gear[1];
              uint8_t data[1];
              gear[0] =
                  static_cast<uint8_t>(sample.get_payload().as_string()[0]);
              data[0] = gear[0] & 0x0F;
              this->canBus->writeMessage(0x04, data, sizeof(data));
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
        // usleep(1000);
        int buffer = this->canBus->checktheReceive();
        if (buffer != -1)
        {
            uint32_t can_id = 0;
            int size        = 0;
            uint8_t data[8];
            size = this->canBus->readMessage(buffer, can_id, data);
            if (can_id == 0x01 && data[0] == 0x86)
            {
                int speed;
                double wheelDiame = 0.067;

                memcpy(&speed, &data[1], 4);

                speed = ntohl(speed);
                speed = wheelDiame * 3.14 * speed * 10 / 60;
                printf("Publishing speed: '%d'\n", speed);
                if (speed < 0 || speed > 100)
                    speed = 0;
                std::string speed_str = std::to_string(speed);

                // printf("Publishing speed: '%d'\n", speed);
                // m_pubSpeed.put(speed_str.c_str());
            }
        }
    }
}
