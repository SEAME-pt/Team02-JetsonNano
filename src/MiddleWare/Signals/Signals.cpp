
#include "Signals.hpp"
#define SESSION_OPEN zenoh::Session::open

Signals::Signals(std::shared_ptr<SensoringPublisher> publisher)
{
    this->canBus = new CAN();
    publisher_   = publisher;

    auto config = zenoh::Config::create_default();
    session_ =
        std::make_shared<zenoh::Session>(SESSION_OPEN(std::move(config)));

    // publisher_ = std::make_unique<ControllerPublisher>(session_);

    activeAutonomyLevel_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/ActiveAutonomyLevel",
        [this](const zenoh::Sample& sample)
        {
            std::string activeAutonomyLevel = sample.get_payload().as_string();
            if (activeAutonomyLevel.find("SAE_5") != std::string::npos)
            {
                uint8_t value[8];
                memcpy(value, &activeAutonomyLevel, sizeof(value));
                this->canBus->writeMessage(0x405, value, sizeof(value));
            }
            else if (activeAutonomyLevel.find("SAE_0") != std::string::npos)
            {
                uint8_t value[8];
                memcpy(value, &activeAutonomyLevel, sizeof(value));
                this->canBus->writeMessage(0x400, value, sizeof(value));
            }
        },
        zenoh::closures::none));
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
        usleep(15);
        int buffer = this->canBus->checktheReceive();
        if (buffer != -1)
        {
            uint32_t can_id = 0;
            // int size        = 0;
            uint8_t data[8];
            this->canBus->readMessage(buffer, can_id, data);
            if (can_id == 0x01)
            {
                int speed;
                // double wheelDiame = 0.067;

                memcpy(&speed, &data[1], 4);

                speed = ntohl(speed);
                // speed = wheelDiame * 3.14 * speed * 10 / 60;
                if (speed < 0 || speed > 2000)
                    speed = 0;
                printf("Publishing speed: '%d'\n", speed);
                std::string speed_str = std::to_string(speed);
                publisher_->publishSpeed(std::stof(speed_str));
            }
        }
    }
}
