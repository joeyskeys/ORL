#include "glfw_application.h"

namespace orl
{

Application::Application(size_t w, size_t h):
    m_width(w), m_height(h)
{
    init_window(m_width, m_height);
    init_vulkan();
}

void Application::run()
{
    loop();
    cleanup();
}

void Application::init_window(size_t w, size_t h)
{
    glfwInit();

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    m_window = glfwCreateWindow(w, h, "orlviewer", nullptr, nullptr);
}

void Application::init_vulkan()
{

}

void Application::loop()
{
    while (!glfwWindowShouldClose(m_window))
    {
        glfwPollEvents();
    }
}

void Application::cleanup()
{
    glfwDestroyWindow(m_window);
    glfwTerminate();
}

}