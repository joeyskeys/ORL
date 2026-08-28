#pragma once

#include <cstdint>
#include <string>

#include "orl_exec.hpp"

namespace ORL
{

// Skin deformer owned by ComponentManager. bind_positions / inverse_binds
// are captured at setup; type selects the ORL stdlib entry (default lbs).
struct DeformerData {
    std::string type = "lbs";
    std::string mesh_name;
    exec::OrlBuffer bind_positions;
    exec::OrlBuffer inverse_binds;
    bool bound = false;

    DeformerData()
        : bind_positions("point", sizeof(double) * 4)
        , inverse_binds("matrix", sizeof(double) * 16)
    {
    }
};

} // namespace ORL
