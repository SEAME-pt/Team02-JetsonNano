
#include "Signals.hpp"


Signals::Signals(): m_session(Session::open(std::move(Config::create_default()))),
      m_pubSpeed(
          m_session.declare_publisher(KeyExpr("seame/car/1/speedSensor")))
{
    this->canBus = new CAN();
}

Signals::~Signals()
{}


void Signals::init(const std::string& canDevice)
{
    this->canBus->init(canDevice);
}

void Signals::run()
{
    while(1)
    {
        usleep(100000);
        int buffer = this->canBus->checkReceive();
        if (buffer != -1)
        {
            uint32_t can_id;
            uint8_t *data;
            data = this->canBus->readMessage(buffer, can_id);
            if (frame.can_id == 0x01)
            {
                int speed;
                double wheelDiame = 0.067;

                memcpy(&speed, data, sizeof(data));
                speed = ntohl(speed);
                speed = wheelDiame * 3.14 * speed * 10 / 60;
                std::string speed_str = std::to_string(speed);

                printf("Publishing speed: '%d'\n", speed);
                pubSpeed.put(speed_str.c_str());
            }
        }
    }
}
