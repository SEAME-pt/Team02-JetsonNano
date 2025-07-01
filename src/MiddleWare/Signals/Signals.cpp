
#include "Signals.hpp"
#include <sys/stat.h>
#define SESSION_OPEN zenoh::Session::open

Signals::Signals(std::shared_ptr<zenoh::Session> session, std::shared_ptr<SensoringPublisher> publisher)
{
    publisher_   = publisher;

    session_ = session;
    
    activeAutonomyLevel_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/ActiveAutonomyLevel",
        [this](const zenoh::Sample& sample)
        {
            std::string activeAutonomyLevel = sample.get_payload().as_string();
            if (this->canBus) {
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
            }
        },
        zenoh::closures::none));

    carlaSpeed_subscriber.emplace(session_->declare_subscriber(
        "carla/speed",
        [this](const zenoh::Sample& sample)
        {
            std::string speed_str = sample.get_payload().as_string();
            publisher_->publishSpeed(std::stof(speed_str));
        },
        zenoh::closures::none));
}

Signals::~Signals() {}

void Signals::initLocalEnv(const std::string& canDevice) {
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

void Signals::initCarlaEnv() {
    this->canBus = NULL;
}

void Signals::run()
{
    while (1)
    {
        usleep(250);  

        if (this->canBus) {
            int buffer = this->canBus->checktheReceive();
            // printf("Buffer: %d\n", buffer);

            if (buffer != -1)
            {
                uint32_t can_id = 0;
                // int size        = 0;
                uint8_t data[8] = {0};
                this->canBus->readMessage(buffer, can_id, data);
                std::cout << "Received CAN ID: 0x" << std::hex << std::setw(3) << std::setfill('0') << can_id 
                          << ", Data: ";
                for (int i = 0; i < 8; i++) {
                    std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
                }
                std::cout << std::dec << std::endl;
                if (can_id == 0x01)
                {
                    int speed;
                    // double wheelDiame = 0.067;
    
                    memcpy(&speed, data, 4);
    
                    speed = ntohl(speed);
                    // speed = wheelDiame * 3.14 * speed * 10 / 60;
                    if (speed < 0 || speed > 2000)
                        speed = 0;

                    bool validReading = true;
                    // int speedChange = speed - lastValidSpeed;
                    // if (!isFirstReading) {
                    //     if (std::abs(speedChange) > MAX_SPEED_CHANGE && speedChange < MAX_SPEED_CHANGE * 2) {
                    //         printf("Rejecting speed reading %d (change of %d from last valid %d)\n", 
                    //                speed, speedChange, lastValidSpeed);
                    //         validReading = false;
                    //     }
                    // }
                    
                    if (validReading) {
                        lastValidSpeed = speed;
                        isFirstReading = false;
                        printf("Publishing speed: '%d'\n", speed);
                        std::string speed_str = std::to_string(speed);
                        publisher_->publishSpeed(std::stof(speed_str));
                    }
                    // else
                    // {
                    //     std::string speed_str = std::to_string(speed + speedChange / 3);
                    //     publisher_->publishSpeed(std::stof(speed_str));
                    // }
                    // printf("Publishing speed: '%d'\n", speed);
                    // std::string speed_str = std::to_string(speed);
                    // publisher_->publishSpeed(std::stof(speed_str));
                }
            }
        }
    }
}
