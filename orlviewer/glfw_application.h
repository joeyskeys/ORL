#pragma once

#ifdef __linux__
#define GLFW_INCLUDE_VULKAN
#else
#include <vulkan/vulkan.h>
#endif
#include <GLFW/glfw3.h>

#include <vector>
#include <optional>

#define MAX_FRAMES_IN_FLIGHT 2

namespace orl
{

struct QueueFamilyIndices
{
    std::optional<uint32_t> graphics_family;
    std::optional<uint32_t> present_family;

    bool isComplete() {
        return graphics_family.has_value() && present_family.has_value();
    }
};

struct SwapChainSupportDetails {
    VkSurfaceCapabilitiesKHR capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
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
    void cleanupSwapChain();
    void cleanup();
    void recreateSwapChain();

    void createInstance();

    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& create_info);
    void setupDebugMessenger();
    bool checkValidationLayerSupport();
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT message_severity, VkDebugUtilsMessageTypeFlagsEXT message_type, const VkDebugUtilsMessengerCallbackDataEXT* pcallback_data, void* puser_data);

    void createSurface();

    std::vector<const char*> getRequiredExtensions();
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device);
    bool isDeviceSuitable(VkPhysicalDevice device);
    void pickPhysicalDevice();
    void createLogicalDevice();

    VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats);
    VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes);
    VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);
    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);

    void createSwapChain();
    void createImageViews();
    void createRenderPass();

    VkShaderModule createShaderModule(const std::vector<char>& code);
    void createGraphicsPipeline();

    void createFramebuffers();
    void createCommandPool();
    void createCommandBuffers();
    void createSyncObjects();

    void drawFrame();

public:

    bool m_frame_buffer_resized = false;

private:

    GLFWwindow *m_window;
    VkInstance instance;
    VkDebugUtilsMessengerEXT m_debug_messenger;
    VkSurfaceKHR m_surface;

    VkPhysicalDevice m_physical_device = VK_NULL_HANDLE;
    VkDevice m_device;

    VkQueue m_graphics_queue;
    VkQueue m_present_queue;

    VkSwapchainKHR m_swap_chain;
    std::vector<VkImage> m_swap_chain_images;
    VkFormat m_swap_chain_image_format;
    VkExtent2D m_swap_chain_extent;
    std::vector<VkImageView> m_swap_chain_image_views;
    std::vector<VkFramebuffer> m_swap_chain_framebuffers;

    VkRenderPass m_render_pass;
    VkPipelineLayout m_pipeline_layout;
    VkPipeline m_graphics_pipeline;

    VkCommandPool m_command_pool;
    std::vector<VkCommandBuffer> m_command_buffers;

    std::vector<VkSemaphore> m_image_available_semaphores;
    std::vector<VkSemaphore> m_render_finished_semaphores;
    std::vector<VkFence> m_in_flight_fences;
    size_t m_current_frame = 0;

    uint32_t 	m_width;
    uint32_t    m_height;
};

}
