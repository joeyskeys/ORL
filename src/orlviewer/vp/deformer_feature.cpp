#include "vp/deformer_feature.hpp"

#include <utility>

namespace ORL
{

DeformerFeature::DeformerFeature(SceneGraphContext& graph_context,
    ComponentId deformer_id, ComponentId weight_id,
    const Selection& selection)
    : runtime(graph_context, selection, deformer_id, weight_id,
        orlgraph::GraphStage::Deformer)
{
}

bool DeformerFeature::set_type(std::string_view name) {
    if (!runtime.set_type(name)) {
        return false;
    }
    type_name = std::string{name};
    return true;
}

void DeformerFeature::set_mesh(std::string name) {
    runtime.set_mesh(std::move(name));
}

void DeformerFeature::request() {
    runtime.request_bind();
}

void DeformerFeature::unbind() {
    runtime.unbind();
}

void DeformerFeature::on_update(vkkk::Context& context,
    const vkkk::Context::Frame&) {
    runtime.on_update(context);
}

} // namespace ORL
