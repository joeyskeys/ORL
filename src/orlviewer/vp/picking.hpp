#pragma once

#include <cstdint>
#include <functional>
#include <iostream>
#include <string_view>
#include <utility>
#include <vector>

#include "vp/rhi_feature.hpp"

namespace ORL
{

using GpuPickHit = Rhi::PickHit;

// Request/readback state shared by the GPU picking features. A request maps
// the cursor on the next update, the feature's layer renders while the pick
// is pending, and a later update reads the A-buffer and reports the hits.
class PickRequest {
public:
    using HitCallback = std::function<void(
        std::uint64_t, const std::vector<GpuPickHit>&, bool)>;

    PickRequest(Rhi::PickLayer layer, std::string_view owner)
        : layer(layer)
        , owner(owner)
    {
    }

    std::uint64_t request(double x, double y) {
        cursor_x = x;
        cursor_y = y;
        requested_request_id = ++next_request_id;
        want_pick = true;
        return requested_request_id;
    }

    void set_hit_callback(HitCallback callback) {
        hit_callback = std::move(callback);
    }

    // Pending readbacks target A-buffers that a resize has replaced.
    void reset() {
        pick_pending = false;
    }

    bool pending() const { return pick_pending; }

    void update(ViewerAdapter& adapter, const Rhi::FrameState& frame) {
        if (pick_pending) {
            Rhi::PickReadback readback;
            const auto status = adapter.read_pick(
                layer, pending_serial, pending_x, pending_y, readback);
            if (!status) {
                std::cerr << owner << ": " << status.message << '\n';
                pick_pending = false;
                report(pending_request_id, {}, false);
            }
            else if (readback.rendered) {
                pick_pending = false;
                report(pending_request_id, readback.hits, readback.overflow);
            }
        }

        if (!want_pick) {
            return;
        }
        if (adapter.map_cursor(cursor_x, cursor_y, pending_x, pending_y)) {
            pending_serial = frame.serial;
            pending_request_id = requested_request_id;
            pick_pending = true;
        }
        else {
            report(requested_request_id, {}, false);
        }
        want_pick = false;
    }

private:
    void report(std::uint64_t request_id, const std::vector<GpuPickHit>& hits, bool overflow) {
        if (hit_callback) {
            hit_callback(request_id, hits, overflow);
        }
    }

    Rhi::PickLayer layer;
    std::string_view owner;
    HitCallback hit_callback;
    double cursor_x = 0.0;
    double cursor_y = 0.0;
    std::uint32_t pending_x = 0;
    std::uint32_t pending_y = 0;
    std::uint64_t pending_serial = 0;
    std::uint64_t next_request_id = 0;
    std::uint64_t requested_request_id = 0;
    std::uint64_t pending_request_id = 0;
    bool want_pick = false;
    bool pick_pending = false;
};

} // namespace ORL
