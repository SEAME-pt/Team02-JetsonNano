
#include "../include/XboxController.hpp"

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

XboxController::XboxController()
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

    int shm_fd = shm_open("/shared_memory", O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to create shared memory segment");
    }

    if (ftruncate(shm_fd, sizeof(SharedMemory)) == -1)
    {
        throw std::runtime_error("Failed to set size of shared memory segment");
    }

    this->sharedData = static_cast<SharedMemory*>(
        mmap(nullptr, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED,
             shm_fd, 0));
    if (this->sharedData == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory segment");
    }

    // Initialize the mutex
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&sharedData->mtx_speed, &attr);
    pthread_mutex_init(&sharedData->mtx_direction, &attr);
    pthread_mutexattr_destroy(&attr);
}

XboxController::~XboxController()
{
    for (int i = 0; i < axes.size(); i++)
    {
        delete axes[i];
    }

    close(js);

    if (munmap(this->sharedData, sizeof(SharedMemory)) == -1)
    {
        std::cerr << "Failed to unmap shared memory segment" << std::endl;
    }

    if (shm_unlink("/shared_memory") == -1)
    {
        std::cerr << "Failed to unlink shared memory segment" << std::endl;
    }
}

void XboxController::test(void)
{
    while (1)
    {
        {
            pthread_mutex_lock(&this->sharedData->mtx_speed);
            std::cout << "SPEED: " << this->sharedData->speed << std::endl;
            pthread_mutex_unlock(&this->sharedData->mtx_speed);
            pthread_mutex_lock(&this->sharedData->mtx_direction);
            std::cout << "DIRECTION: " << this->sharedData->direction
                      << std::endl;
            pthread_mutex_unlock(&this->sharedData->mtx_direction);
        }
        // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}