#include "I2C.hpp"
#include "PCA9685.hpp"

int main(void)
{
    try
    {
        I2C* i2c = new I2C;
        PCA9685 motorPCA;
        PCA9685 servoPCA;

        i2c->init("/dev/i2c-1");

        motorPCA.init(i2c, 0x60);
        motorPCA.setPWMFreq(1600);
        
        servoPCA.init(i2c, 0x40);
        servoPCA.setPWMFreq(50);

        motorPCA.setDutyCicle(0, 0);
        motorPCA.setGPIO(1, true);
        motorPCA.setGPIO(2, false);

        motorPCA.setDutyCicle(7, 0);
        motorPCA.setGPIO(6, true);
        motorPCA.setGPIO(5, false);

        while (true)
        {
            int angle = 90;
            std::cout << "Number: ";
            std::cin >> angle;
            uint16_t pulseWidth = static_cast<uint16_t>(102 + (angle / 180.0) * (510 - 102));
            servoPCA.setDutyCicle(0, pulseWidth);
        }

        delete i2c;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}