
#pragma once

#include <linux/joystick.h>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <map>
#include <string>
#include <cstring>
#include "ControllerPublisher.hpp"
#include "zenoh.hxx"

#define JS_EVENT_BUTTON 0x01 /* button pressed/released */
#define JS_EVENT_AXIS 0x02   /* joystick moved */
#define JS_EVENT_INIT 0x80   /* initial state of device */

enum Button
{
    BUTTON_A           = 0,
    BUTTON_B           = 1,
    BUTTON_BACK        = 2,
    BUTTON_X           = 3,
    BUTTON_Y           = 4,
    BUTTON_UNKNOWN     = 5,
    BUTTON_LB          = 6,
    BUTTON_RB          = 7,
    BUTTON_L2          = 8,
    BUTTON_R2          = 9,
    BUTTON_SELECT      = 10,
    BUTTON_START       = 11,
    BUTTON_HOME        = 12,
    BUTTON_LEFT_STICK  = 13,
    BUTTON_RIGHT_STICK = 14
};

enum Axis
{
    AXIS_RIGHT_STICK = 0,
    AXIS_LEFT_STICK  = 1,
};

struct axis_state
{
    int x = 0;
    int y = 0;
};

/**
 * @brief Xbox controller interface for vehicle control and light management
 *
 * @details Implements game controller interface for vehicle control including:
 *          - Speed and steering control via analog sticks
 *          - Light system control via buttons
 *          - Gear selection
 *          - State publishing via Zenoh
 *
 * Control mappings:
 * Buttons:
 * - RB: Right direction indicator
 * - LB: Left direction indicator
 * - A: Low beam lights
 * - B: High beam lights
 * - X: Rear fog lights
 * - Y: Front fog lights
 * - L2: Hazard lights
 * - R2: Parking lights
 *
 * Analog sticks:
 * - Left stick (Y-axis): Speed control (-100 to +100)
 *   - Up: Forward gear
 *   - Down: Reverse gear
 *   - Center (±5): Neutral
 * - Right stick (X-axis): Steering (0 to 180 degrees)
 *   - Center: 90° (straight)
 *   - Left: 0° (full left)
 *   - Right: 180° (full right)
 *
 * @note Requires Linux joystick driver (/dev/input/js0)
 * @note Axis values normalized from hardware range (-32767 to +32767)
 * @see ControllerPublisher
 * @see zenoh::Session
 */
class XboxController
{
  private:
    int js;
    std::shared_ptr<zenoh::Session> session_;
    std::unique_ptr<ControllerPublisher> publisher_;

  public:
    std::vector<struct axis_state*> axes;
    struct js_event event;

    XboxController();
    XboxController(const std::string& configFile);
    ~XboxController();

    int readEvent(void);
    int getButtonCount(void);
    int getAxisCount(void);
    int getAxisState(void);
    void run();
};