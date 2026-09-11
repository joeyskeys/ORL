#pragma once

#include <string>

#include <glm/mat4x4.hpp>

#include "abi.hpp"
#include "../orl_exec.hpp"

namespace orlrig
{

struct DeformerData {
    std::string type = "lbs";
    std::string mesh_name;
    ORL::exec::OrlBuffer bind_positions;
    ORL::exec::OrlBuffer inverse_binds;
    glm::mat4 bind_model{1.0f};
    bool bound = false;

    DeformerData()
        : bind_positions(kPointOrlType, kPointStride)
        , inverse_binds(kMatrixOrlType, kMatrixStride)
    {
    }
};

} // namespace orlrig
