#include "../include/I2C.hpp"
#include "../include/PCA9685.hpp"

int main(void)
{
    try
    {
        I2C* i2c = new I2C("/dev/i2c-1");
        PCA9685 motorPCA(i2c, 0x60);
        // PCA9685 servoPCA(i2c, 0x40);

        motorPCA.setDutyCicle(0, 2000);
        motorPCA.setGPIO(1, true);
        motorPCA.setGPIO(2, false);

        delete i2c;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}