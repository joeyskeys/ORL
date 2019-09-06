#pragma once

#include <GLFW/glfw3.h>

namespace orl
{

class Application
{
public:

    Application(size_t w = 800, size_t h = 600);

    void run();

private:

    void init_window(size_t w, size_t h);
    void init_vulkan();
    void loop();
    void cleanup();

private:

    GLFWwindow *m_window;
    VkInstance instance;

    size_t      m_width;
    size_t      m_height;
};

}