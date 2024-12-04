
#include <linux/joystick.h>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


int main(void)
{
    std::string deviceRoute = "/dev/input/js0";
    int js = open(deviceRoute.c_str(), O_RDONLY);

    if (js == -1)
        throw std::exception();

    struct js_event event;
    while (read(js, &event, sizeof(event)) > 0)
    {
        switch (event.type)
        {
            case JS_EVENT_BUTTON:
                std::cout << "Button event: ";
                switch (event.number)
                {
                    case 0:
                        std::cout << "Button 0 (A)" << std::endl;
                        break;
                    case 1:
                        std::cout << "Button 1 (B)" << std::endl;
                        break;
                    case 2:
                        std::cout << "Button 2 (X)" << std::endl;
                        break;
                    case 3:
                        std::cout << "Button 3 (Y)" << std::endl;
                        break;
                    case 4:
                        std::cout << "Button 4 (LB)" << std::endl;
                        break;
                    case 5:
                        std::cout << "Button 5 (RB)" << std::endl;
                        break;
                     case 6:
                        std::cout << "Button 6 (Back)" << std::endl;
                        break;
                    case 7:
                        std::cout << "Button 7 (Start)" << std::endl;
                        break;
                    case 8:
                        std::cout << "Button 8 (Home)" << std::endl;
                        break;
                    case 9:
                        std::cout << "Button 9 (Left Stick)" << std::endl;
                        break;
                    case 10:
                        std::cout << "Button 10 (Right Stick)" << std::endl;
                        break;
                    default:
                        std::cout << "Unknown button" << std::endl;
                        break;
                }
                break;

            case JS_EVENT_AXIS:
                std::cout << "Axis event: ";
                switch (event.number)
                {
                    case 0:
                        std::cout << "Left Stick X Axis " << event.value << std::endl;
                        break;
                    case 1:
                        std::cout << "Left Stick Y Axis " << event.value << std::endl;
                        break;
                    case 2:
                        std::cout << "Right Stick X Axis " << event.value << std::endl;
                        break;
                    case 3:
                        std::cout << "Right Stick Y Axis " << event.value << std::endl;
                        break;
                    case 4:
                        std::cout << "Left Trigger " << event.value << std::endl;
                        break;
                    case 5:
                        std::cout << "Right Trigger " << event.value << std::endl;
                        break;
                    case 6:
                        std::cout << "D-Pad X Axis " << event.value << std::endl;
                        break;
                    case 7:
                        std::cout << "D-Pad Y Axis " << event.value << std::endl;
                        break;
                    default:
                        std::cout << "Unknown axis" << std::endl;
                        break;
                }
                break;

            default:
                break;
        }
    }

    close(js);
    return 0;
}