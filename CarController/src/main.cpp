#include "RaceCar.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Function to set terminal to non-blocking mode
void setNonBlockingInput(bool enable)
{
    struct termios tty;
    tcgetattr(STDIN_FILENO, &tty);
    if (enable)
    {
        tty.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
        tty.c_cc[VMIN] = 1;
    }
    else
    {
        tty.c_lflag |= (ICANON | ECHO); // Enable canonical mode and echo
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

// Function to check if a key has been pressed
int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main(void)
{
    // Set terminal to non-blocking mode
    setNonBlockingInput(true);

    try
    {
        RaceCar raceCar;
        raceCar.init("/dev/i2c-1", 0x60, 0x40);

        int ch;
        int angle = 90;
        int speed = 0;

        raceCar.setSpeed(speed);

        std::cout
            << "Use arrow keys to change direction and 'w' to increase speed."
            << std::endl;
        std::cout << "Press 'q' to quit." << std::endl;

        while (true)
        {
            if (kbhit())
            {
                ch = getchar();
                if (ch == 'q')
                    break;

                switch (ch)
                {
                    case 'w':
                        speed += 10;
                        if (speed > 100)
                            speed = 100;
                        raceCar.setSpeed(speed);
                        std::cout << "Speed: " << speed << std::endl;
                        break;
                    case 's':
                        speed -= 10;
                        if (speed < -100)
                            speed = -100;
                        raceCar.setSpeed(speed);
                        std::cout << "Speed: " << speed << std::endl;
                        break;
                    case 'a':
                        angle -= 10;
                        if (angle < 0)
                            angle = 0;
                        raceCar.setDirection(angle);
                        std::cout << "Angle: " << angle << std::endl;
                        break;
                    case 'd':
                        angle += 10;
                        if (angle > 180)
                            angle = 180;
                        raceCar.setDirection(angle);
                        std::cout << "Angle: " << angle << std::endl;
                        break;
                    default:
                        break;
                }
            }
            usleep(100000); // Sleep for 100ms
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    // Restore terminal to blocking mode
    setNonBlockingInput(false);
    return 0;
}