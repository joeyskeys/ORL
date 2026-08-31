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

} // namespace ORL
