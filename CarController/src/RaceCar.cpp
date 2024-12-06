#include "RaceCar.hpp"

RaceCar::RaceCar()
{
    int shm_fd = shm_open("/shared_memory", O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to open shared memory segment");
    }

    this->sharedData = static_cast<SharedMemory*>(
        mmap(nullptr, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED,
             shm_fd, 0));
    if (this->sharedData == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory segment");
    }

    close(shm_fd);

    this->m_I2c      = new I2C;
    this->m_motorPCA = new PCA9685;
    this->m_ServoPCA = new PCA9685;

    std::cout << "Car created!" << std::endl;
}

RaceCar::~RaceCar()
{
    if (this->m_motorPCA)
    {
        delete this->m_motorPCA;
    }
    else
    {
        // nothing
    }

    if (this->m_ServoPCA)
    {
        delete this->m_ServoPCA;
    }
    else
    {
        // nothing
    }
    if (this->m_I2c)
    {
        delete this->m_I2c;
    }
    else
    {
        // nothing
    }

    if (munmap(this->sharedData, sizeof(SharedMemory)) == -1)
    {
        std::cerr << "Failed to unmap shared memory segment" << std::endl;
    }
}

void RaceCar::init(const std::string& i2cDevice, uint8_t motorAddress,
                   uint8_t servoAddress)
{
    this->m_I2c->init(i2cDevice);
    this->m_motorPCA->init(this->m_I2c, motorAddress);
    this->m_ServoPCA->init(this->m_I2c, servoAddress);

    this->m_motorPCA->setPWMFreq(1600);
    this->m_ServoPCA->setPWMFreq(50);

    this->motorLeft.init(this->m_motorPCA, LEFT);
    this->motorRight.init(this->m_motorPCA, RIGHT);
    this->servo.init(this->m_ServoPCA);
}

void RaceCar::setDirection(uint8_t angle)
{
    servo.setDirection(angle);
}

void RaceCar::setSpeed(int speed)
{
    motorLeft.setSpeed(speed);
    motorRight.setSpeed(speed);
}

void RaceCar::run(void)
{
    std::cout << "Car running!" << std::endl;
    int32_t prevSpeed;
    int32_t prevDirection;
    {
        pthread_mutex_lock(&this->sharedData->mtx_speed);
        prevSpeed = this->sharedData->speed;
        pthread_mutex_unlock(&this->sharedData->mtx_speed);
        pthread_mutex_lock(&this->sharedData->mtx_direction);
        prevDirection = this->sharedData->direction;
        pthread_mutex_unlock(&this->sharedData->mtx_direction);
    }
    while (signalTo)
    {
        {
            pthread_mutex_lock(&this->sharedData->mtx_speed);
            if (prevSpeed != this->sharedData->speed)
            {
                setSpeed(this->sharedData->speed);
                prevSpeed = this->sharedData->speed;
            }
            pthread_mutex_unlock(&this->sharedData->mtx_speed);
            pthread_mutex_lock(&this->sharedData->mtx_direction);
            if (prevDirection != this->sharedData->direction)
            {
                setDirection(this->sharedData->direction);
                prevDirection = this->sharedData->direction;
            }
            pthread_mutex_unlock(&this->sharedData->mtx_direction);
        }
    }
}