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
    std::vector<std::string> element_names(SceneElementKind kind) const;
    std::optional<std::int64_t> resolve_element_handle(
        SceneElementKind kind, std::string_view name) const;

    bool make_interface_port(const orlgraph::StableId& id,
        orlgraph::InterfacePort* port, std::string* error = nullptr) const;
    bool resolve(const orlgraph::InterfacePort& port,
        exec::GraphInputBinding& binding, std::string* error = nullptr);
    bool bind_graph_inputs(exec::OrlGraphExecution& execution,
        const orlgraph::GraphModule& module);

private:
    enum class SourceKind {
        MeshPositions,
        MeshVertexCount,
        Joints,
        JointCount,
        WeightBuffer,
        WeightCount,
        InverseBinds,
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
    bool pack_controller(ComponentId id, const std::string& binding);
    bool set_error(std::string* error, std::string message) const;

    vkkk::Scene& scene_;
    ComponentManager& components_;
    std::vector<SceneInputDescriptor> descriptors_;
    std::unordered_map<std::string, Source> sources_;
    std::unordered_map<std::string, exec::OrlBuffer> mesh_positions_;
    std::unordered_map<std::string, exec::OrlBuffer> controller_xforms_;
    exec::OrlBuffer joints_;
};

} // namespace ORL
