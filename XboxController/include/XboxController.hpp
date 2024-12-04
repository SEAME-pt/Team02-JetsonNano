
#pragma once

#include <linux/joystick.h>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <map>

enum Button
{
    BUTTON_A = 0,
    BUTTON_B,
    BUTTON_X,
    BUTTON_Y,
    BUTTON_LB,
    BUTTON_RB,
    BUTTON_BACK,
    BUTTON_START,
    BUTTON_HOME,
    BUTTON_LEFT_STICK,
    BUTTON_RIGHT_STICK
};

enum Axis
{
    AXIS_LEFT_STICK_X = 0,
    AXIS_LEFT_STICK_Y,
    AXIS_RIGHT_STICK_X,
    AXIS_RIGHT_STICK_Y,
    AXIS_LEFT_TRIGGER,
    AXIS_RIGHT_TRIGGER,
    AXIS_DPAD_X,
    AXIS_DPAD_Y
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
        
        XboxController();
        ~XboxController();
        int readEvent( void );
        int getButtonCount( void );
        int getAxisCount( void );
        int getAxisState( void );



};