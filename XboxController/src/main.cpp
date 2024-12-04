

#include "../include/XboxController.hpp"

#define JS_EVENT_BUTTON		0x01	/* button pressed/released */
#define JS_EVENT_AXIS		0x02	/* joystick moved */
#define JS_EVENT_INIT		0x80	/* initial state of device */

#define SHM_NAME "/xbox_shm"
#define SHM_SIZE 1024

int main(void)
{
    XboxController player;
    size_t axis;

    /* This loop will exit if the controller is unplugged. */
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
                    case BUTTON_START:
                        std::cout << "Button Start" << std::endl;
                        break;
                    case BUTTON_HOME:
                        std::cout << "Button Home" << std::endl;
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
            case JS_EVENT_AXIS:
                axis = player.getAxisState();
                if (axis )
                    printf("Axis %zu at (%6d, %6d)\n", axis, player.axes[axis]->x, player.axes[axis]->y);
                break;
            default:
                break;
        }
        fflush(stdout);
    }
    return 0;
}