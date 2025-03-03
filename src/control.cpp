#include "XboxController.hpp"

int main(int ac, char** argv)
{
    try
    {
        XboxController controller;
        controller.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}