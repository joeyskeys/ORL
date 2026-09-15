#pragma once

namespace ORL
{

enum class ComputeDevice {
    Cpu,
    Gpu,
};

// Process-wide runtime switches. The compute target is selected at startup;
// ORL evaluation can be toggled live while authoring the rig.
struct RuntimeConfig {
    ComputeDevice device = ComputeDevice::Gpu;
    bool evaluate_orl = true;
};

inline RuntimeConfig runtime_config;

inline const char* compute_device_label() {
    return runtime_config.device == ComputeDevice::Gpu ? "GPU" : "CPU";
}

inline const char* orl_evaluation_label() {
    return runtime_config.evaluate_orl ? "enabled" : "disabled";
}

} // namespace ORL
