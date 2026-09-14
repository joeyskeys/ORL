#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>

#include "../orl_exec.hpp"
#include "runner_status.hpp"

namespace orlrig
{

// The two-stage runner models a rig evaluation as a dataflow boundary:
//
//   solver stage   ->   varying buffers   ->   deformer stage
//
// Solver inputs and deformer inputs are independent binding sets. Varyings
// are the explicitly named buffers written by the solver and consumed by the
// deformer. On CUDA, a device-only evaluation connects those buffers through
// their device pointers without a host readback.
enum class RigStage {
    Solver,
    Deformer,
};

struct StageProgram {
    std::string source;
    ORL::exec::CompileOptions options;
};

struct TwoStageProgram {
    StageProgram solver;
    StageProgram deformer;
};

// Named inputs that are local to one stage. A buffer may be host-backed or an
// externally-owned CUDA allocation; scalar values are stage-local uniforms.
class StageBindings {
public:
    bool bind_buffer(std::string parameter, ORL::exec::OrlBuffer& buffer);
    bool bind_device_buffer(std::string parameter,
        ORL::exec::DeviceBufferView buffer,
        std::size_t element_count = 0);
    bool bind_int(std::string parameter, std::int64_t value);
    bool bind_float(std::string parameter, double value);
    void clear();

private:
    struct DeviceBinding {
        ORL::exec::DeviceBufferView view;
        std::size_t element_count = 0;
    };

    std::map<std::string, ORL::exec::OrlBuffer*> buffers_;
    std::map<std::string, DeviceBinding> device_buffers_;
    std::map<std::string, std::int64_t> integers_;
    std::map<std::string, double> floats_;

    friend class TwoStageRunner;
};

class TwoStageRunner {
public:
    TwoStageRunner(TwoStageProgram program,
        ORL::exec::Backend backend = ORL::exec::Backend::Cpu);

    // Bind one logical varying. The parameter names may differ between the
    // stages, which keeps stage-local ORL signatures independent.
    bool bind_varying(std::string name,
        ORL::exec::OrlBuffer& buffer,
        std::string solver_parameter = {},
        std::string deformer_parameter = {});

    StageBindings& solver_inputs() { return solver_inputs_; }
    StageBindings& deformer_inputs() { return deformer_inputs_; }
    const StageBindings& solver_inputs() const { return solver_inputs_; }
    const StageBindings& deformer_inputs() const { return deformer_inputs_; }

    // Execute both stages in order. solver_element_count and
    // deformer_element_count are independent launch sizes, matching the
    // solver's and deformer’s potentially different parallel domains.
    RunnerStatus execute(std::uint32_t solver_element_count = 1,
        std::uint32_t deformer_element_count = 1,
        bool device_only = false);

    // The split calls are useful when a caller needs to run more than one
    // solver pass before invoking the deformer.
    RunnerStatus execute_solver(std::uint32_t element_count = 1,
        bool device_only = false);
    RunnerStatus execute_deformer(std::uint32_t element_count = 1,
        bool device_only = false);

    ORL::exec::Backend backend() const { return compute_backend_; }
    bool solver_complete() const { return solver_complete_; }

    // Returns the device allocation bound to a stage parameter after that
    // stage has been initialized/evaluated. For a deformer varying this may be
    // the solver-owned allocation imported at the stage boundary.
    std::optional<ORL::exec::DeviceBufferView> device_buffer(
        RigStage stage, std::string_view parameter);
    std::optional<ORL::exec::DeviceBufferView> varying_device_buffer(
        std::string_view name);

private:
    struct VaryingBinding {
        std::string name;
        std::string solver_parameter;
        std::string deformer_parameter;
        ORL::exec::OrlBuffer* buffer = nullptr;
    };

    RunnerStatus ensure_programs();
    RunnerStatus bind_solver_stage();
    RunnerStatus bind_deformer_stage();
    RunnerStatus bind_stage(ORL::exec::OrlExecution& execution,
        const ORL::exec::OrlProgram& program,
        const StageBindings& bindings,
        const char* stage);
    RunnerStatus validate_host_buffer(
        const ORL::exec::OrlProgram& program,
        std::string_view parameter,
        const ORL::exec::OrlBuffer& buffer,
        const char* stage) const;
    RunnerStatus validate_device_buffer(
        const ORL::exec::OrlProgram& program,
        std::string_view parameter,
        ORL::exec::DeviceBufferView buffer,
        std::size_t element_count,
        const char* stage) const;
    RunnerStatus bind_error(const ORL::exec::OrlExecution& execution,
        const char* stage) const;

    TwoStageProgram program_;
    ORL::exec::Backend compute_backend_;
    std::optional<ORL::exec::OrlProgram> solver_program_;
    std::optional<ORL::exec::OrlExecution> solver_execution_;
    std::optional<ORL::exec::OrlProgram> deformer_program_;
    std::optional<ORL::exec::OrlExecution> deformer_execution_;
    StageBindings solver_inputs_;
    StageBindings deformer_inputs_;
    std::map<std::string, VaryingBinding> varyings_;
    std::map<std::string, ORL::exec::DeviceBufferView> solver_device_varyings_;
    bool solver_complete_ = false;
    bool use_device_varyings_ = false;
};

} // namespace orlrig
