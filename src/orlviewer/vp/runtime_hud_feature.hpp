#pragma once

#include <iostream>
#include <string>

#include "runtime_config.hpp"
#include "vp/rhi_feature.hpp"

namespace ORL
{

class RuntimeHudFeature final : public RhiFeature {
public:
    void on_attach(ViewerAdapter& adapter, Rhi::Extent2D) {
        report(adapter.prepare_status_text());
    }

    void on_submit(ViewerAdapter& adapter, const Rhi::FrameState&) {
        report(adapter.draw_status_text(
            std::string("Device: ") + compute_device_label()
            + " | ORL evaluation: " + orl_evaluation_label()));
    }

private:
    void report(const Rhi::Status& status) {
        if (!status && !reported) {
            std::cerr << "RuntimeHudFeature: " << status.message << '\n';
            reported = true;
        }
    }

    bool reported = false;
};

} // namespace ORL
