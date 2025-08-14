#include "XboxController.hpp"

#ifdef TEST_MODE
// Define custom function names for testing
#define device_open custom_xbox_open
#define device_close custom_xbox_close
#define device_ioctl custom_xbox_ioctl
#define device_read custom_xbox_read
#define device_write custom_xbox_write
#else
#define device_open open
#define device_close close
#define device_ioctl ioctl
#define device_read read
#define device_write write
#endif

XboxController::XboxController(std::shared_ptr<zenoh::Session> session)
{
    const char* device = "/dev/input/js0";
    js                 = device_open(device, O_RDONLY);

    if (js == -1)
        throw std::exception();

    int numAxes = getAxisCount();
    axes.clear();
    axes.resize(numAxes);

    session_ = session;

    publisher_ = std::make_unique<ControllerPublisher>(session_);

    autonomy_env_enable_subscriber_.emplace(session_->declare_subscriber(
        "Vehicle/1/ADAS/Enable",
        [this](const zenoh::Sample& sample)
        {
            std::string autonomyEnvEnable = sample.get_payload().as_string();
            if (autonomyEnvEnable.find("true") != std::string::npos)
            {
                autonomyEnvEnable_ = true;
            }
            else
            {
                autonomyEnvEnable_ = false;
            }
        },
        zenoh::closures::none));

    std::cout << "Remote controller created!" << std::endl;
}

XboxController::~XboxController()
{
    device_close(js);
}

int XboxController::readEvent()
{
    std::memset(&event, 0, sizeof(event));
    ssize_t bytes = device_read(js, &event, sizeof(event));
    if (bytes == sizeof(event)) return 0;
    if (bytes < 0 && (errno == EAGAIN || errno == EINTR)) return -2;
    return -1;
}

int XboxController::getButtonCount(void)
{
    int buttons;
    if (device_ioctl(js, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

int XboxController::getAxisCount(void)
{
    int axes = 0;
    if (device_ioctl(js, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

int XboxController::getAxisState(void)
{
    int axis = event.number / 2;

    if (axis >= 0 && static_cast<size_t>(axis) < axes.size()) {
        if (event.number % 2 == 0)
            axes[axis].x = event.value;
        else
            axes[axis].y = event.value;
    }
    return axis;
}

void XboxController::run()
{
    size_t axis;
    size_t button;

    float last_speed = std::numeric_limits<float>::quiet_NaN();
    float last_dir   = std::numeric_limits<float>::quiet_NaN();

    while (this->readEvent() == 0)
    {
        switch (this->event.type)
        {
            case JS_EVENT_BUTTON:
            {
                if (event.value != 1) break;
                button = event.number;
                switch (button)
                {
                    case BUTTON_RB:
                    {
                        publisher_->publishDirectionIndicatorRight(true);

                        std::cout << "RightBlinker" << std::endl;
                        break;
                    }
                    case BUTTON_LB:
                    {
                        publisher_->publishDirectionIndicatorLeft(true);
                        std::cout << "LeftBlinker" << std::endl;
                        break;
                    }
                    case BUTTON_A:
                    {
                        publisher_->publishBeamLow(true);
                        std::cout << "lowBeam" << std::endl;
                        break;
                    }
                    case BUTTON_B:
                    {
                        publisher_->publishBeamHigh(true);
                        std::cout << "highBeam" << std::endl;
                        break;
                    }
                    case BUTTON_X:
                    {
                        publisher_->publishFogRear(true);
                        std::cout << "rearFogLight" << std::endl;
                        break;
                    }
                    case BUTTON_Y:
                    {
                        publisher_->publishFogFront(true);
                        std::cout << "frontFogLight" << std::endl;
                        break;
                    }
                    case BUTTON_L2:
                    {
                        publisher_->publishHazard(true);
                        std::cout << "hazardLight" << std::endl;
                        break;
                    }
                    case BUTTON_R2:
                    {
                        publisher_->publishParking(true);
                        std::cout << "parkingLight" << std::endl;
                        break;
                    }
                    case BUTTON_START:
                    {
                        if (sae_4 == true)
                        {
                            publisher_->publishActiveAutonomyLevel("SAE_0");
                            std::cout << "SAE_0 Driving" << std::endl;
                            sae_4 = false;
                        }
                        else
                        {
                            if (autonomyEnvEnable_)
                            {
                                publisher_->publishActiveAutonomyLevel(
                                    "SAE_4");
                                std::cout << "SAE_4 Driving" << std::endl;
                                sae_4      = true;
                                sae_3      = false;
                                sae_2      = false;
                                sae_1_LKAS = false;
                                sae_1_ACC  = false;
                            }
                            else
                            {
                                std::cout << "Autonomy Env not ready"
                                            << std::endl;
                            }
                        }
                        break;
                    }
                    case BUTTON_SELECT:
                    {
                        pidEnable_.load() ? pidEnable_.store(false)
                                            : pidEnable_.store(true);
                        std::cout << "Control Type Switch" << std::endl;
                        break;
                    }

                    default:
                        break;
                }
                break;   
            }
            case JS_EVENT_AXIS:
            {
                axis   = this->getAxisState();
                button = this->event.number;

                if (axis == AXIS_LEFT_STICK)
                {
                    float speed = -static_cast<float>(axes[axis].y) * 50.0f / 32767.0f;
                    if (speed != last_speed)
                    {
                        if (speed < -5)      publisher_->publishCurrentGear(-1);
                        else if (speed > 5)  publisher_->publishCurrentGear(1);
                        else                 publisher_->publishCurrentGear(0);
                        manual_speed_.store(speed);
                        last_speed = speed;
                    }
                }
                else if (axis == AXIS_RIGHT_STICK)
                {
                    float direction = 90.0f + static_cast<float>(axes[axis].x) * 90.0f / 32767.0f;
                    if (direction != last_dir)
                    {
                        if (sae_2 || sae_3 || sae_4)
                        {
                            publisher_->publishActiveAutonomyLevel("SAE_0");
                            sae_2 = sae_3 = sae_4 = false;
                        }
                        manual_steering_.store(direction);
                        last_dir = direction;
                    }
                }

                switch (button)
                {
                    case (BUTTON_CLICK_LEFT_RIGHT):
                    {
                        if (this->event.value == 32767)
                        {
                            if (sae_1_LKAS == true)
                            {
                                publisher_->publishActiveAutonomyLevel("SAE_0");
                                sae_1_LKAS = false;
                            }
                            else
                            {
                                if (autonomyEnvEnable_)
                                {
                                    publisher_->publishActiveAutonomyLevel(
                                        "SAE_1_LKAS");
                                    std::cout << "SAE_1_LKAS Driving"
                                              << std::endl;
                                    sae_4      = false;
                                    sae_3      = false;
                                    sae_2      = false;
                                    sae_1_LKAS = true;
                                    sae_1_ACC  = false;
                                }
                                else
                                {
                                    std::cout << "Autonomy Env not ready"
                                              << std::endl;
                                }
                            }
                        }
                        else if (this->event.value == -32767)
                        {
                            if (sae_1_ACC == true)
                            {
                                publisher_->publishActiveAutonomyLevel("SAE_0");
                                sae_1_ACC = false;
                            }
                            else
                            {
                                if (autonomyEnvEnable_)
                                {
                                    publisher_->publishActiveAutonomyLevel(
                                        "SAE_1_ACC");
                                    std::cout << "SAE_1_ACC Driving"
                                              << std::endl;
                                    sae_4      = false;
                                    sae_3      = false;
                                    sae_2      = false;
                                    sae_1_LKAS = false;
                                    sae_1_ACC  = true;
                                }
                                else
                                {
                                    std::cout << "Autonomy Env not ready"
                                              << std::endl;
                                }
                            }
                        }

                        break;
                    }
                    case (BUTTON_CLICK_UP_DOWN):
                    {
                        if (this->event.value == 32767)
                        {
                            if (sae_2 == true)
                            {
                                publisher_->publishActiveAutonomyLevel("SAE_0");
                                sae_2 = false;
                            }
                            else
                            {
                                if (autonomyEnvEnable_)
                                {
                                    publisher_->publishActiveAutonomyLevel(
                                        "SAE_2");
                                    std::cout << "SAE_2 Driving" << std::endl;
                                    sae_4      = false;
                                    sae_3      = false;
                                    sae_2      = true;
                                    sae_1_LKAS = false;
                                    sae_1_ACC  = false;
                                }
                                else
                                {
                                    std::cout << "Autonomy Env not ready"
                                              << std::endl;
                                }
                            }
                        }
                        else if (this->event.value == -32767)
                        {
                            if (sae_3 == true)
                            {
                                publisher_->publishActiveAutonomyLevel("SAE_0");
                                sae_3 = false;
                            }
                            else
                            {
                                if (autonomyEnvEnable_)
                                {
                                    publisher_->publishActiveAutonomyLevel(
                                        "SAE_3");
                                    std::cout << "SAE_3 Driving" << std::endl;
                                    sae_4      = false;
                                    sae_3      = true;
                                    sae_2      = false;
                                    sae_1_LKAS = false;
                                    sae_1_ACC  = false;
                                }
                                else
                                {
                                    std::cout << "Autonomy Env not ready"
                                              << std::endl;
                                }
                            }
                        }

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
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

float XboxController::getManualSteering() const
{
    return manual_steering_.load();
}

float XboxController::getManualSpeed() const
{
    return manual_speed_.load();
}

bool XboxController::getPidEnable() const
{
    return pidEnable_.load();
}
