#pragma once

namespace ORL
{

enum class ComputeDevice {
    Cpu,
    Gpu,
};

// Process-wide compute target. Set before the viewer loop; not switched live.
struct RuntimeConfig {
    ComputeDevice device = ComputeDevice::Gpu;
};

inline RuntimeConfig runtime_config;

inline const char* compute_device_label() {
    return runtime_config.device == ComputeDevice::Gpu ? "GPU" : "CPU";
}

} // namespace ORL
