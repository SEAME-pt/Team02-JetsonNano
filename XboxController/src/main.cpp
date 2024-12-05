

#include "../include/XboxController.hpp"

#define JS_EVENT_BUTTON		0x01	/* button pressed/released */
#define JS_EVENT_AXIS		0x02	/* joystick moved */
#define JS_EVENT_INIT		0x80	/* initial state of device */

int main(void)
{
    XboxController player;
    size_t axis;
    //std::cout << player.getButtonCount() <<std::endl;

    //std::thread runThread(&XboxController::test, player);
    while (player.readEvent() == 0)
    {
        std::cout << player.event.type << std::endl;
        switch (player.event.type)
        {
            case JS_EVENT_AXIS:
                axis = player.getAxisState();
                switch (axis)
                {
                    case (AXIS_LEFT_STICK):
                        {
                            std::lock_guard<std::mutex> lock(player.sharedData->mtx);
                            std::cout << "ESTOU AQUI" << std::endl;
                            player.sharedData->speed = -player.axes[axis]->y* 100 / 32767;
                        }
                        break;
                    case (AXIS_RIGHT_STICK):
                        {
                            std::lock_guard<std::mutex> lock(player.sharedData->mtx);
                                player.sharedData->direction = player.axes[axis]->x * 100 / 32767;
                        }
                        break;
                    default:
                        break;
                }
            default:
                break;
        }
        fflush(stdout);
    }
    //runThread.join();
    return 0;
}