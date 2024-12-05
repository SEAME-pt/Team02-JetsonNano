
#pragma once

#include "SharedMemory.hpp"
#include <linux/joystick.h>
#include <thread>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <map>
#include <sys/mman.h>

enum Button
{
    BUTTON_A = 0,
    BUTTON_B = 1,
    BUTTON_BACK = 2,
    BUTTON_X = 3,
    BUTTON_Y = 4,
    BUTTON_UNKNOWN= 5,
    BUTTON_LB = 6,
    BUTTON_RB = 7,
    BUTTON_L2 = 8,
    BUTTON_R2 = 9,
    BUTTON_SELECT = 10,
    BUTTON_START = 11,
    BUTTON_HOME = 12,
    BUTTON_LEFT_STICK = 13,
    BUTTON_RIGHT_STICK = 14
};

enum Axis
{
    AXIS_RIGHT_STICK = 0,
    AXIS_LEFT_STICK = 1,
};

struct axis_state {
    int x = 0;
    int y = 0;
};


class XboxController
{
    private:
        std::map<int, int> buttons;
        int js;

    public:
        std::vector<struct axis_state*> axes;
        struct js_event event;
        SharedMemory* sharedData;
        
        XboxController();
        ~XboxController();
        int readEvent( void );
        int getButtonCount( void );
        int getAxisCount( void );
        int getAxisState( void );
        void test( void );



};