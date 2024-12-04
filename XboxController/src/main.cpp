

#include "../include/XboxController.hpp"

#define JS_EVENT_BUTTON		0x01	/* button pressed/released */
#define JS_EVENT_AXIS		0x02	/* joystick moved */
#define JS_EVENT_INIT		0x80	/* initial state of device */

int main(void)
{
    XboxController player;
    size_t axis;
    std::cout << player.getButtonCount() <<std::endl;

    while (player.readEvent() == 0)
    {
        switch (player.event.type)
        {
            case JS_EVENT_BUTTON:
                switch (player.event.number)
                {
                    case BUTTON_A:
                        std::cout << "Button A" << std::endl;
                        break;
                    case BUTTON_B:
                        std::cout << "Button B" << std::endl;
                        break;
                    case BUTTON_X:
                        std::cout << "Button X" << std::endl;
                        break;
                    case BUTTON_Y:
                        std::cout << "Button Y" << std::endl;
                        break;
                    case BUTTON_LB:
                        std::cout << "Button LB" << std::endl;
                        break;
                    case BUTTON_RB:
                        std::cout << "Button RB" << std::endl;
                        break;
                    case BUTTON_BACK:
                        std::cout << "Button Back" << std::endl;
                        break;
                    case BUTTON_UNKNOWN:
                        std::cout << "Button Start" << std::endl;
                        break;
                    case BUTTON_L2:
                        std::cout << "Button L2" << std::endl;
                        break;
                    case BUTTON_R2:
                        std::cout << "Button R2" << std::endl;
                        break;
                    case BUTTON_SELECT:
                        std::cout << "Button Select" << std::endl;
                        break;
                    case BUTTON_START:
                        std::cout << "Button Start" << std::endl;
                        break;
                    case BUTTON_HOME:
                        std::cout << "Button HOME" << std::endl;
                        break;
                    case BUTTON_LEFT_STICK:
                        std::cout << "Button Left Stick" << std::endl;
                        break;
                    case BUTTON_RIGHT_STICK:
                        std::cout << "Button Right Stick" << std::endl;
                        break;
                    default:
                        std::cout << "Unknown button" << std::endl;
                        break;
                }
                break;
            case JS_EVENT_AXIS:
                axis = player.getAxisState();
                switch (axis)
                {
                    case (AXIS_LEFT_STICK):
                        printf("Axis %zu at (%6d, %6d)\n", axis, player.axes[axis]->x, -player.axes[axis]->y);
                        break;
                    case (AXIS_RIGHT_STICK):
                        printf("Axis %zu at (%6d, %6d)\n", axis, player.axes[axis]->x, -player.axes[axis]->y);
                        break;
                    default:
                        break;
                }
            default:
                break;
        }
        fflush(stdout);
    }
    return 0;
}