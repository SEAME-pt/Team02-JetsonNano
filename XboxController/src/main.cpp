

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
        switch (player.event.type)
        {
            case JS_EVENT_BUTTON:
                //  std::cout << "BUTTON" << std::endl;
            case JS_EVENT_AXIS:
                axis = player.getAxisState();
                switch (axis)
                {
                    case (AXIS_LEFT_STICK):
                        {
                            pthread_mutex_lock(&player.sharedData->mtx_speed);
                            player.sharedData->speed = -player.axes[axis]->y* 100 / 32767;
                            pthread_mutex_unlock(&player.sharedData->mtx_speed);

                        }
                        //sleep(1);
                        break;
                    case (AXIS_RIGHT_STICK):
                        {
                            pthread_mutex_lock(&player.sharedData->mtx_direction);
                            uint8_t x = 90 + player.axes[axis]->x * 90 / 32767;
                            player.sharedData->direction = x;
                            pthread_mutex_unlock(&player.sharedData->mtx_direction);
                            
                        }
                        //sleep(1);
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