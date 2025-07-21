#include "utils.hpp"

bool parseParameters(int argc, char** argv, std::string& configFile,
                     std::string& mode)
{
    configFile = "";
    mode       = "local";

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg == "--config" || arg == "-C")
        {
            if (i + 1 < argc)
            {
                configFile = argv[i + 1];
                i++;
                std::cout << "Using config file: " << configFile << std::endl;
            }
            else
            {
                std::cerr << "Error: --config/-C requires a file path"
                          << std::endl;
                return false;
            }
        }
        else if (arg == "--mode" || arg == "-M")
        {
            if (i + 1 < argc)
            {
                mode = argv[i + 1];
                i++;

                if (mode != "local" && mode != "carla" && mode != "test")
                {
                    std::cerr
                        << "Error: mode must be 'local', 'carla' or 'test'"
                        << std::endl;
                    return false;
                }

                std::cout << "Using mode: " << mode << std::endl;
            }
            else
            {
                std::cerr
                    << "Error: --mode/-M requires a value (local or carla)"
                    << std::endl;
                return false;
            }
        }
        // else {
        //     std::cerr << "Unknown argument: " << arg << std::endl;
        //     std::cerr << "Usage: " << argv[0] << " [--config/-C
        //     <config_file>] [--mode/-M <local|carla>]" << std::endl; return
        //     false;
        // }
    }

    return true;
}