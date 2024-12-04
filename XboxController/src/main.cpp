

#include "../include/XboxController.hpp"

#define JS_EVENT_BUTTON		0x01	/* button pressed/released */
#define JS_EVENT_AXIS		0x02	/* joystick moved */
#define JS_EVENT_INIT		0x80	/* initial state of device */

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
                std::cout << "Button " << player.event.number << " , " << player.event.value << std::endl;
                break;
            case JS_EVENT_AXIS:
                axis = player.getAxisState();
                if (axis < 4)
                    printf("Axis %zu at (%6d, %6d)\n", axis, player.axes[axis]->x, player.axes[axis]->y);
                break;
            default:
                break;
        }
        fflush(stdout);
    }
    return 0;
}