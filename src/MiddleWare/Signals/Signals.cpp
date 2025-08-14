
#include "Signals.hpp"
#include <sys/stat.h>
#define SESSION_OPEN zenoh::Session::open

// // Add this helper method to get current time (like candump timestamp)
// double static getCurrentTime() {
//     struct timespec ts;
//     clock_gettime(CLOCK_REALTIME, &ts);
//     return ts.tv_sec + (ts.tv_nsec / 1.0e9);
// }

Signals::Signals(std::shared_ptr<zenoh::Session> session,
                 std::shared_ptr<SensoringPublisher> publisher)
{
    publisher_ = publisher;

    session_ = session;

    activeAutonomyLevel_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/ActiveAutonomyLevel",
        [this](const zenoh::Sample& sample)
        {
            std::string activeAutonomyLevel = sample.get_payload().as_string();
            if (this->canBus)
            {
                if (activeAutonomyLevel.find("SAE_4") != std::string::npos)
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

    trafficSign_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/TrafficSign",
        [this](const zenoh::Sample& sample)
        {
            std::string trafficSign = sample.get_payload().as_string();
            if (this->canBus)
            {
                uint8_t value[8];
                if (trafficSign.find("Speed 50km/h") != std::string::npos)
                {
                    std::cout << "\033[32mSpeed 50\033[0m" << std::endl;
                    int speed = 50;
                    memcpy(value, &speed, sizeof(value));
                    this->canBus->writeMessage(0x500, value, sizeof(value));
                }
                else if (trafficSign.find("Speed 80km/h") != std::string::npos)
                {
                    std::cout << "\033[36mSpeed 80\033[0m" << std::endl;
                    int speed = 80;
                    memcpy(value, &speed, sizeof(value));
                    this->canBus->writeMessage(0x505, value, sizeof(value));
                }
                else if (trafficSign.find("Yield") != std::string::npos)
                {
                    std::cout << "\033[33mYield\033[0m" << std::endl;
                    memcpy(value, &trafficSign, sizeof(value));
                    this->canBus->writeMessage(0x502, value, sizeof(value));
                }
                else if (trafficSign.find("Stop") != std::string::npos)
                {
                    std::cout << "\033[31mStop\033[0m" << std::endl;
                    memcpy(value, &trafficSign, sizeof(value));
                    this->canBus->writeMessage(0x501, value, sizeof(value));
                }
                else if (trafficSign.find("Danger") != std::string::npos)
                {
                    std::cout << "\033[35mDanger\033[0m" << std::endl;
                    memcpy(value, &trafficSign, sizeof(value));
                    this->canBus->writeMessage(0x504, value, sizeof(value));
                }
                else if (trafficSign.find("Crosswalk") != std::string::npos)
                {
                    std::cout << "\033[34mCrosswalk\033[0m" << std::endl;
                    memcpy(value, &trafficSign, sizeof(value));
                    this->canBus->writeMessage(0x503, value, sizeof(value));
                }
                else if (trafficSign.find("Traffic Yellow") !=
                         std::string::npos)
                {
                    std::cout << "\033[37mTraffic Yellow\033[0m" << std::endl;
                    memcpy(value, &trafficSign, sizeof(value));
                    this->canBus->writeMessage(0x600, value, sizeof(value));
                }
                else if (trafficSign.find("Traffic Green") != std::string::npos)
                {
                    std::cout << "\033[38mTraffic Green\033[0m" << std::endl;
                    memcpy(value, &trafficSign, sizeof(value));
                    this->canBus->writeMessage(0x601, value, sizeof(value));
                }
                else if (trafficSign.find("Traffic Red") != std::string::npos)
                {
                    std::cout << "\033[39mTraffic Red\033[0m" << std::endl;
                    memcpy(value, &trafficSign, sizeof(value));
                    this->canBus->writeMessage(0x602, value, sizeof(value));
                }
                else
                {
                    std::cout << "\033[37mUnknown traffic sign!\033[0m"
                              << std::endl;
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

    laneAlert_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/LaneAlert",
        [this](const zenoh::Sample& sample)
        {
            std::string lane = sample.get_payload().as_string();
            if (this->canBus)
            {
                if (lane.find("Left") != std::string::npos)
                {
                    uint8_t value[8];
                    memcpy(value, &lane, sizeof(value));
                    this->canBus->writeMessage(0x301, value, sizeof(value));
                    std::cout << "Lane Alert: Left" << std::endl;
                }
                else if (lane.find("Right") != std::string::npos)
                {
                    uint8_t value[8];
                    memcpy(value, &lane, sizeof(value));
                    this->canBus->writeMessage(0x303, value, sizeof(value));
                    std::cout << "Lane Alert: Right" << std::endl;
                }
                else if (lane.find("Off") != std::string::npos)
                {
                    uint8_t value[8];
                    memcpy(value, &lane, sizeof(value));
                    this->canBus->writeMessage(0x302, value, sizeof(value));
                    std::cout << "Lane Alert: Off" << std::endl;
                }
            }
        },
        zenoh::closures::none));

    emergency_brake_subscriber_.emplace(session_->declare_subscriber(
        "Vehicle/1/Speed/Emergency",
        [this](const zenoh::Sample& sample)
        {
            std::string emergency = sample.get_payload().as_string();
            if (emergency.find("1") != std::string::npos)
            {
                uint8_t value[8];
                memcpy(value, "DANGER", sizeof(value));

                this->canBus->writeMessage(0x200, value, sizeof(value));
            }
        },
        zenoh::closures::none));

    beamLow_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Beam/Low",
        [this](const zenoh::Sample& sample)
        {
            static bool state = 0;
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            (void)isOn;

            std::cout << "Low" << std::endl;
            state         = !state;
            uint8_t onOff = static_cast<uint8_t>(state);
            this->canBus->writeMessage(0x702, &onOff, sizeof(uint8_t));
        },
        zenoh::closures::none));

    beamHigh_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Beam/High",
        [this](const zenoh::Sample& sample)
        {
            static bool state = 0;
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            (void)isOn;

            std::cout << "High" << std::endl;
            state         = !state;
            uint8_t onOff = static_cast<uint8_t>(state);
            this->canBus->writeMessage(0x703, &onOff, sizeof(uint8_t));
        },
        zenoh::closures::none));

    running_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Running",
        [this](const zenoh::Sample& sample)
        {
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            (void)isOn;
        },
        zenoh::closures::none));

    parking_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Parking",
        [this](const zenoh::Sample& sample)
        {
            static bool state = 0;
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            (void)isOn;

            std::cout << "Parking" << std::endl;
            state         = !state;
            uint8_t onOff = static_cast<uint8_t>(state);
            this->canBus->writeMessage(0x707, &onOff, sizeof(uint8_t));
        },
        zenoh::closures::none));

    fogRear_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Fog/Rear",
        [this](const zenoh::Sample& sample)
        {
            static bool state = 0;
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            (void)isOn;

            std::cout << "RearFog" << std::endl;
            state         = !state;
            uint8_t onOff = static_cast<uint8_t>(state);
            this->canBus->writeMessage(0x705, &onOff, sizeof(uint8_t));
        },
        zenoh::closures::none));

    fogFront_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Fog/Front",
        [this](const zenoh::Sample& sample)
        {
            static bool state = 0;
            StaticLights value;
            bool isOn = std::stoi(sample.get_payload().as_string());

            (void)isOn;

            std::cout << "FrontFog" << std::endl;
            state         = !state;
            uint8_t onOff = static_cast<uint8_t>(state);
            this->canBus->writeMessage(0x704, &onOff, sizeof(uint8_t));
        },
        zenoh::closures::none));

    brake_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Brake",
        [this](const zenoh::Sample& sample)
        {
            BrakeLights value;
            bool isActive = std::stoi(sample.get_payload().as_string());

            (void)isActive;
        },
        zenoh::closures::none));

    hazard_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/Hazard",
        [this](const zenoh::Sample& sample)
        {
            static bool state = 0;
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            (void)isSignaling;

            std::cout << "Hazard" << std::endl;
            state         = !state;
            uint8_t onOff = static_cast<uint8_t>(state);
            this->canBus->writeMessage(0x706, &onOff, sizeof(uint8_t));
        },
        zenoh::closures::none));

    directionIndicatorLeft_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/DirectionIndicator/Left",
        [this](const zenoh::Sample& sample)
        {
            static bool state = 0;
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            (void)isSignaling;

            std::cout << "Left" << std::endl;
            state         = !state;
            uint8_t onOff = static_cast<uint8_t>(state);
            this->canBus->writeMessage(0x700, &onOff, sizeof(uint8_t));
        },
        zenoh::closures::none));

    directionIndicatorRight_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Body/Lights/DirectionIndicator/Right",
        [this](const zenoh::Sample& sample)
        {
            static bool state = 0;
            SignalingLights value;
            bool isSignaling = std::stoi(sample.get_payload().as_string());

            (void)isSignaling;

            std::cout << "Right" << std::endl;
            state         = !state;
            uint8_t onOff = static_cast<uint8_t>(state);
            this->canBus->writeMessage(0x701, &onOff, sizeof(uint8_t));
        },
        zenoh::closures::none));

    currentGear_subscriber.emplace(session_->declare_subscriber(
        "Vehicle/1/Powertrain/Transmission/CurrentGear",
        [this](const zenoh::Sample& sample)
        {
            int8_t currentGear = std::stoi(sample.get_payload().as_string());

            uint8_t gearData[1];
            gearData[0] = static_cast<uint8_t>(currentGear);
            this->canBus->writeMessage(0x04, gearData, sizeof(gearData));
        },
        zenoh::closures::none));
}

Signals::~Signals() {}

void Signals::initLocalEnv(const std::string& canDevice)
{
    try
    {
        // struct stat buffer;
        // if (stat(canDevice.c_str(), &buffer) != 0) {
        //     std::cerr << "Can device " << canDevice << " does not exist!" <<
        //     std::endl; this->canBus = NULL; throw std::runtime_error("Error
        //     on can device");
        // }
        this->canBus = new CAN();
        this->canBus->init(canDevice);
    }
    catch (...)
    {
        std::cerr << "Error on initializing can" << std::endl;
        this->canBus = NULL;
    }
}

void Signals::initCarlaEnv()
{
    this->canBus = NULL;
}

void Signals::run()
{
    int canSocket = this->canBus->getCanFd();
    while (1)
    {
        struct can_frame frame;

        int nbytes = read(canSocket, &frame, sizeof(struct can_frame));
        if (nbytes < 0)
        {
            std::cerr << "Error reading CAN frame!" << std::endl;
            continue;
        }
        if (frame.can_id == 0x01)
        {
            int speed;

            memcpy(&speed, frame.data, sizeof(int));
            speed = ntohl(speed);

            float fspeed = static_cast<float>(speed);
            std::cout << fspeed << std::endl;

            this->publisher_->publishSpeed(fspeed);
        }
        usleep(10);
    }
}
