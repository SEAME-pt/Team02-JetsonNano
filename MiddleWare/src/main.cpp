#include <stdio.h>
#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "zenoh.hxx"

using namespace zenoh;

int main(int argc, char** argv)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    int canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (canSocket < 0)
    {
        std::cerr << "Cannot create CAN socket!" << std::endl;
        exit(-1);
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(canSocket, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(canSocket, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "Cannot bind CAN socket!" << std::endl;
        close(canSocket);
        exit(1);
    }

    std::cout << "CAN socket bound to can0 interface successfully."
              << std::endl;

    Config config = Config::create_default();
    auto session  = Session::open(std::move(config));

    auto pubSpeed =
        session.declare_publisher(KeyExpr("seame/car/1/speedSensor"));
    auto pubBattery =
        session.declare_publisher(KeyExpr("seame/car/1/batterySensor"));

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
            double wheelDiame = 0.067;

            memcpy(&speed, frame.data, sizeof(int));
            speed                 = ntohl(speed);
            speed                 = wheelDiame * 3.14 * speed * 10 / 60;
            std::string speed_str = std::to_string(speed);

            printf("Publishing speed: '%d'\n", speed);
            pubSpeed.put(speed_str.c_str());
        }
        else if (frame.can_id == 0x02)
        {
            int battery;
            memcpy(&battery, frame.data, sizeof(int));
            battery                 = ntohl(battery);
            std::string battery_str = std::to_string(battery);

            printf("Publishing battery: '%d\n", battery);
            pubBattery.put(battery_str.c_str());
        }
        usleep(10);
    }
    return 0;
}
