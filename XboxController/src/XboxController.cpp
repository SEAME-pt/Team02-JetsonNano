#include "XboxController.hpp"

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

    while (this->readEvent() == 0)
    {
        switch (this->event.type)
        {
            case JS_EVENT_BUTTON:
            {
                button = this->event.number;
                switch (button)
                {
                    case BUTTON_RB:
                    {
                        LightsInfo test;
                        char buffer[sizeof(LightsInfo)];
                        memcpy(buffer, &test, sizeof(LightsInfo));
                        this->m_pubLights.put(buffer);
                        std::cout << "RightBlinker" << std::endl;
                        break;
                    }
                    case BUTTON_LB:
                    {
                        bool leftBlinker = true;
                        this->m_pubLights.put(std::to_string(leftBlinker));
                        std::cout << "LeftBlinker" << std::endl;
                        break;
                    }
                    default:
                        break;
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