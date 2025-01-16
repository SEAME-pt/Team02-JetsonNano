#include "../include/XboxController.hpp"

#define JS_EVENT_BUTTON 0x01 /* button pressed/released */
#define JS_EVENT_AXIS 0x02   /* joystick moved */
#define JS_EVENT_INIT 0x80   /* initial state of device */

int main(void)
{
    Config config = Config::create_default();
    auto session  = Session::open(std::move(config));

    XboxController player(session);
    size_t axis;
    size_t button;

    auto pubThrottle =
        session.declare_publisher(KeyExpr("seame/car/1/throttle"));
    auto pubDirection =
        session.declare_publisher(KeyExpr("seame/car/1/direction"));
    auto pubLights = session.declare_publisher(KeyExpr("seame/car/1/lights"));

    while (player.readEvent() == 0)
    {
        switch (player.event.type)
        {
            case JS_EVENT_BUTTON:
            {
                button = player.event.number;
                switch (button)
                {
                    case BUTTON_RB:
                    {
                        bool rightBlinker = true;
                        pubLights.put(std::to_string(rightBlinker).c_str());
                        std::cout << "RightBlinker" << std::endl;
                        break;
                    }
                    case BUTTON_LB:
                    {
                        bool leftBlinker = true;
                        pubLights.put(std::to_string(leftBlinker).c_str());
                        std::cout << "LeftBlinker" << std::endl;
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case JS_EVENT_AXIS:
            {
                axis = player.getAxisState();
                switch (axis)
                {
                    case (AXIS_LEFT_STICK):
                    {
                        int speed = -player.axes[axis]->y * 100 / 32767;
                        std::string speedStr = std::to_string(speed);
                        pubThrottle.put(speedStr.c_str());
                        std::cout << "Speed" << std::endl;
                        break;
                    }
                    case (AXIS_RIGHT_STICK):
                    {
                        uint8_t direction =
                            90 + player.axes[axis]->x * 90 / 32767;
                        std::string dirStr = std::to_string(direction);
                        pubDirection.put(dirStr.c_str());
                        std::cout << "Direction" << std::endl;
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            default:
                break;
        }
        fflush(stdout);
    }
    return 0;
}