
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
    read(js, &event, sizeof(event));
    switch (event.type)
    {
        case JS_EVENT_BUTTON:
            std::cout << "Carrega-me nos botoes" << std::endl;
        
        case JS_EVENT_AXIS:
            if (event.number == 0)
            {
                if (event.number % 2 == 0)
                {
                    std::cout << "Isso, mexe-me nos guizos" << std::endl;
                    std::cout << "X_Axis " << event.value << std::endl;
                }
                else
                    std::cout << "Isso, mexe-me nos guizos" << std::endl;
                    std::cout << "Y_Axis " << event.value << std::endl;
            }
        default:
            break;

    }

}