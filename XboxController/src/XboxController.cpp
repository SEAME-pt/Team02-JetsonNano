#include "../include/XboxController.hpp"

XboxController::XboxController()
    : m_session(Session::open(std::move(Config::create_default()))),
      m_pubThrottle(
          m_session.declare_publisher(KeyExpr("seame/car/1/throttle"))),
      m_pubDirection(
          m_session.declare_publisher(KeyExpr("seame/car/1/direction"))),
      m_pubLights(m_session.declare_publisher(KeyExpr("seame/car/1/lights")))
{
    const char* device = "/dev/input/js0";
    js                 = open(device, O_RDONLY);

    if (js == -1)
        throw std::exception();

    int numAxes = this->getAxisCount();
    for (int i = 0; i < numAxes; i++)
    {
        struct axis_state* axis = new struct axis_state();
        axes.push_back(axis);
    }

    std::cout << "Remote controller created!" << std::endl;
}

XboxController::~XboxController()
{
    for (unsigned int i = 0; i < axes.size(); i++)
    {
        delete axes[i];
    }

    close(js);
    this->m_session.close();
}
int XboxController::readEvent(void)
{
    int bytes;

    bytes = read(js, &event, sizeof(event));

    if (bytes == sizeof(event))
        return 0;

    return -1;
}

int XboxController::getButtonCount(void)
{
    int buttons;
    if (ioctl(js, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

int XboxController::getAxisCount(void)
{
    int axes;
    if (ioctl(js, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

int XboxController::getAxisState(void)
{
    int axis = event.number / 2;

    if (axis < 3)
    {
        if (event.number % 2 == 0)
            axes[axis]->x = event.value;
        else
            axes[axis]->y = event.value;
    }
    return axis;
}

void XboxController::run()
{
    size_t axis;
    size_t button;
    LightStatus test;
    char buffer[1];

    while (this->readEvent() == 0)
    {
        switch (this->event.type)
        {
            case JS_EVENT_BUTTON:
            {
                button = this->event.number;
                if (this->event.value == 1) // Check if the button is pressed
                {
                    switch (button)
                    {
                        case BUTTON_RB:
                        {
                            
                            if (buffer[0] & 1 == 1)
                                buffer[0] &= (0 << 0);
                            else
                                buffer[0] |= (1 << 0);
                            this->m_pubLights.put(buffer);
                            std::cout << "RightBlinker" << std::endl;
                            break;
                        }
                        case BUTTON_LB:
                        {
                            if ((buffer[0] >> 1) & 1 == 1)
                                buffer[0] &= (1 << 1);
                            else
                                buffer[0] |= (1 << 1);
                            bool leftBlinker = true;
                            this->m_pubLights.put(buffer));
                            std::cout << "LeftBlinker" << std::endl;
                            break;
                        }
                        case BUTTON_A:
                        {
                            
                            if ((buffer[0] >> 2) & 1 == 1)
                                buffer[0] &= (0 << 2);
                            else
                                buffer[0] |= (1 << 2);
                            this->m_pubLights.put(buffer);
                            std::cout << "lowBeam" << std::endl;
                            break;
                        }
                        case BUTTON_B:
                        {
                            if ((buffer[0] >> 3) & 1 == 1)
                                buffer[0] &= (1 << 3);
                            else
                                buffer[0] |= (1 << 3);
                            bool leftBlinker = true;
                            this->m_pubLights.put(buffer));
                            std::cout << "highBeam" << std::endl;
                            break;
                        }
                        case BUTTON_X:
                        {
                            
                            if ((buffer[0] >> 4) & 1 == 1)
                                buffer[0] &= (0 << 4);
                            else
                                buffer[0] |= (1 << 4);
                            this->m_pubLights.put(buffer);
                            std::cout << "frontFogLight" << std::endl;
                            break;
                        }
                        case BUTTON_Y:
                        {
                            if ((buffer[0] >> 5) & 1 == 1)
                                buffer[0] &= (1 << 5);
                            else
                                buffer[0] |= (1 << 5);
                            bool leftBlinker = true;
                            this->m_pubLights.put(buffer));
                            std::cout << "rearFogLight" << std::endl;
                            break;
                        }
                        case BUTTON_L2:
                        {
                            if ((buffer[0] >> 3) & 1 == 1)
                                buffer[0] &= (1 << 3);
                            else
                                buffer[0] |= (1 << 3);
                            bool leftBlinker = true;
                            this->m_pubLights.put(buffer));
                            std::cout << "highBeam" << std::endl;
                            break;
                        }
                        case BUTTON_R2:
                        {
                            
                            if ((buffer[0] >> 4) & 1 == 1)
                                buffer[0] &= (0 << 4);
                            else
                                buffer[0] |= (1 << 4);
                            this->m_pubLights.put(buffer);
                            std::cout << "frontFogLight" << std::endl;
                            break;
                        }
                        case BUTTON_START:
                        {
                            if ((buffer[0] >> 5) & 1 == 1)
                                buffer[0] &= (1 << 5);
                            else
                                buffer[0] |= (1 << 5);
                            bool leftBlinker = true;
                            this->m_pubLights.put(buffer));
                            std::cout << "parkingLight" << std::endl;
                            break;
                        }

                        default:
                            break;
                    }
                }
                break;
            }
            case JS_EVENT_AXIS:
            {
                axis = this->getAxisState();
                switch (axis)
                {
                    case (AXIS_LEFT_STICK):
                    {
                        int speed = -this->axes[axis]->y * 100 / 32767;
                        this->m_pubThrottle.put(std::to_string(speed));
                        std::cout << "Speed" << std::endl;
                        break;
                    }
                    case (AXIS_RIGHT_STICK):
                    {
                        uint8_t direction =
                            90 + this->axes[axis]->x * 90 / 32767;
                        this->m_pubDirection.put(std::to_string(direction));
                        std::cout << "Direction" << std::endl;
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            default:
                break;
        }
        fflush(stdout);
    }
}