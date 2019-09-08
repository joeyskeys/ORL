#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <vector>

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

    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& create_info);
    void setupDebugMessenger();
    std::vector<const char*> getRequiredExtensions();
    bool checkValidationLayerSupport();
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT message_severity, VkDebugUtilsMessageTypeFlagsEXT message_type, const VkDebugUtilsMessengerCallbackDataEXT* pcallback_data, void* puser_data);

private:

    GLFWwindow *m_window;
    VkInstance instance;
    VkDebugUtilsMessengerEXT m_debug_messenger;

    size_t      m_width;
    size_t      m_height;
};

}