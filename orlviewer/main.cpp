#include "glfw_application.h"

#include <iostream>

using namespace orl;

int main(int argc, char* argv[])
{
    Application app;

    try
    {
        app.run();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
    
    return 0;
}