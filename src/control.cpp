#include "XboxController.hpp"

int main(int argc, char** argv)
{
    try
    {
        XboxController* controller;
        if (argc == 2)
        {
            controller = new XboxController(argv[1]);
        }
        else
        {
            controller = new XboxController();
        }
        controller->run();

        delete controller;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}