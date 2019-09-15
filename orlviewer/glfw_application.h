#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <vector>

namespace orl
{

struct QueueFamilyIndices
{
    std::optional<uint32_t> graphicsFamily;

    bool isComplete() {
        return graphicsFamily.has_value();
    }
};

class Application
{
public:

    Application(size_t w = 800, size_t h = 600);

    void run();

private:

    void initWindow(size_t w, size_t h);
    void initVulkan();
    void loop();
    void cleanup();

    void createInstance();

    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& create_info);
    void setupDebugMessenger();
    std::vector<const char*> getRequiredExtensions();
    bool checkValidationLayerSupport();
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT message_severity, VkDebugUtilsMessageTypeFlagsEXT message_type, const VkDebugUtilsMessengerCallbackDataEXT* pcallback_data, void* puser_data);

    std::vector<const char*> getRequiredExtensions();
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device);
    bool isDeviceSuitable(VkPhysicalDevice device);
    void pickPhysicalDevice();
    void createLogicalDevice();

private:

    GLFWwindow *m_window;
    VkInstance instance;
    VkDebugUtilsMessengerEXT m_debug_messenger;

    VkPhysicalDevice m_physical_device = VK_NULL_HANDLE;
    VkDevice m_device;

    VkQueue m_graphics_queue;

    size_t      m_width;
    size_t      m_height;
};

}