
#include "../include/XboxController.hpp"

XboxController::XboxController(Session &session) : m_session(session)
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
