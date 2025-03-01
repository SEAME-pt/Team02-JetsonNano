
#include "Signals.hpp"

Signals::Signals()
{
    this->canBus = new CAN();
    vehicle_ = VehicleFactory::createDefaultVehicle();
    publisher_ = std::make_unique<SensoringPublisher>();
    vss_subscriber_   = std::make_unique<VSSSubscriber>(vehicle_);

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
        usleep(300000);
        int buffer = this->canBus->checktheReceive();
        if (buffer != -1)
        {
            uint32_t can_id = 0;
            int size        = 0;
            uint8_t data[8];
            size = this->canBus->readMessage(buffer, can_id, data);
            if (can_id == 0x01)
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
                publisher_->publishSpeed(std::stof(speed_str));
            }
        }
    }
}
