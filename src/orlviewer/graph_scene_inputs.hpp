#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "asset_mgr/scene.h"
#include "component_manager.hpp"
#include "orl_graph_exec.hpp"

namespace ORL
{

struct SceneInputDescriptor {
    orlgraph::StableId id;
    std::string label;
    orlgraph::InterfacePort port;
    exec::ParameterKind kind = exec::ParameterKind::Unsupported;
    std::size_t element_stride = 0;
    std::string count_binding;
    bool host_backed = true;
    bool gpu_backed = false;
};

enum class SceneElementKind {
    Joint,
    Controller,
    Locator,
    Mesh,
};

// Resolves stable scene binding names to the current scene and rig data.
// The catalog owns packed host buffers where the viewer data does not already
// use an ORL-compatible ABI. It can be refreshed after scene edits.
class SceneInputCatalog final {
public:
    SceneInputCatalog(vkkk::Scene& scene, ComponentManager& components);

    void refresh();

    const std::vector<SceneInputDescriptor>& descriptors() const {
        return descriptors_;
    }
    const SceneInputDescriptor* find(const orlgraph::StableId& id) const;
    std::size_t revision() const { return revision_; }
    std::vector<std::string> element_names(SceneElementKind kind) const;
    std::optional<std::int64_t> resolve_element_handle(
        SceneElementKind kind, std::string_view name) const;
    std::optional<std::int64_t> resolve_element_index(
        SceneElementKind kind, std::string_view name) const;

    bool make_interface_port(const orlgraph::StableId& id,
        orlgraph::InterfacePort* port, std::string* error = nullptr) const;
    // Enables the CUDA evaluation representation. The next refresh packs
    // scene arrays into one host arena; disabling it restores normal scene
    // buffer bindings for rig editing and CPU evaluation.
    bool set_cuda_evaluation(bool enabled);
    bool ensure_cuda_inputs();
    bool resolve(const orlgraph::InterfacePort& port,
        exec::GraphInputBinding& binding, std::string* error = nullptr);
    bool resolve_binding(std::string_view binding,
        exec::GraphInputBinding& result, std::string* error = nullptr);
    // The view is borrowed from the solver execution. Its owner must keep the
    // target allocation alive until the deformer has finished.
    void set_computed_joints_device(
        std::optional<exec::DeviceBufferView> view,
        std::size_t element_count);
    void clear_computed_joints_device();
    std::optional<exec::DeviceBufferView> computed_joints_device() const {
        return computed_joints_device_;
    }
    std::size_t computed_joints_device_count() const {
        return computed_joints_device_count_;
    }
    exec::OrlBuffer& computed_joints_buffer() {
        return joints_;
    }
    const exec::OrlBuffer& computed_joints_buffer() const {
        return joints_;
    }
    bool bind_graph_inputs(exec::OrlGraphExecution& execution,
        const orlgraph::GraphModule& module);
    bool bind_solver_context(exec::OrlGraphExecution& execution,
        std::string* error = nullptr);
    orlrig::SolverContext solver_context() const;
    std::optional<exec::DeviceBufferView> solver_joints_device_view(
        exec::OrlGraphExecution& execution) const;
    // Copies the host-side packed joint buffer back to component storage after
    // a graph evaluation that performed host readback. Device-only evaluation
    // must pass false and is rejected to avoid committing stale TRS values.
    bool commit_joints(bool host_readback_complete,
        std::string* error = nullptr);

private:
    enum class SourceKind {
        MeshPositions,
        MeshVertexCount,
        Joints,
        ComputedJoints,
        JointCount,
        WeightBuffer,
        WeightCount,
        InverseBinds,
        Locators,
        LocatorCount,
        LocatorXform,
        Controllers,
        ControllersCount,
        ControllerXform,
        ControllerCount,
    };

    struct Source {
        SourceKind kind = SourceKind::Joints;
        ComponentId component;
        std::string object_name;
    };

    void add_descriptors();
    void add_buffer_descriptor(orlgraph::StableId id,
        std::string label, orlgraph::LogicalType element,
        orlgraph::Domain domain, orlgraph::Shape shape,
        std::string semantic, std::string coordinate_space,
        std::size_t element_stride, std::string count_binding,
        Source source);
    void add_scalar_descriptor(orlgraph::StableId id,
        std::string label, std::string semantic, Source source);

    bool pack_mesh_positions(const std::string& object_name,
        const std::string& binding);
    bool pack_joints();
    bool pack_locators();
    bool pack_controllers();
    bool pack_locator(ComponentId id, const std::string& binding);
    bool pack_controller(ComponentId id, const std::string& binding);
    bool prepare_packed_inputs();
    exec::PackedBufferView packed_view(
        std::size_t offset, std::size_t bytes);
    bool set_error(std::string* error, std::string message) const;

    struct PackedSolverInputs {
        std::vector<std::byte> storage;
        std::vector<ComponentId> joint_ids;
        std::vector<ComponentId> locator_ids;
        std::vector<ComponentId> controller_ids;
        std::size_t joints_offset = 0;
        std::size_t locators_offset = 0;
        std::size_t controllers_offset = 0;
        std::uint64_t version = 1;
        bool ready = false;
    };

    vkkk::Scene& scene_;
    ComponentManager& components_;
    std::vector<SceneInputDescriptor> descriptors_;
    std::unordered_map<std::string, Source> sources_;
    std::unordered_map<std::string, exec::OrlBuffer> mesh_positions_;
    std::unordered_map<std::string, exec::OrlBuffer> locator_xforms_;
    std::unordered_map<std::string, exec::OrlBuffer> controller_xforms_;
    exec::OrlBuffer joints_;
    std::optional<exec::DeviceBufferView> computed_joints_device_;
    std::size_t computed_joints_device_count_ = 0;
    exec::OrlBuffer locators_;
    exec::OrlBuffer controllers_;
    std::vector<ComponentId> joint_ids_;
    std::vector<ComponentId> packed_joint_ids_;
    std::vector<ComponentId> locator_ids_;
    std::vector<ComponentId> controller_ids_;
    PackedSolverInputs packed_solver_inputs;
    bool cuda_evaluation = false;
    std::size_t revision_ = 0;
};

} // namespace ORL
