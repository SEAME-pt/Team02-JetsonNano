#include "XboxController.hpp"

int main()
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