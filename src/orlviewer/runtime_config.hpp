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

// Temporarily pauses kernel execution while a viewport feature updates
// authoring data. Call resume() before invoking the graph so the previous
// evaluation state is restored for the actual evaluation pass.
class OrlEvaluationPause final {
public:
    OrlEvaluationPause()
        : was_enabled_(runtime_config.evaluate_orl)
    {
        if (was_enabled_) {
            runtime_config.evaluate_orl = false;
        }
    }

    OrlEvaluationPause(const OrlEvaluationPause&) = delete;
    OrlEvaluationPause& operator=(const OrlEvaluationPause&) = delete;

    ~OrlEvaluationPause() {
        resume();
    }

    bool was_enabled() const { return was_enabled_; }

    void resume() {
        if (resumed_) {
            return;
        }
        resumed_ = true;
        runtime_config.evaluate_orl = was_enabled_;
    }

private:
    bool was_enabled_ = false;
    bool resumed_ = false;
};

inline const char* compute_device_label() {
    return runtime_config.device == ComputeDevice::Gpu ? "GPU" : "CPU";
}

inline const char* orl_evaluation_label() {
    return runtime_config.evaluate_orl ? "enabled" : "disabled";
}

} // namespace ORL
