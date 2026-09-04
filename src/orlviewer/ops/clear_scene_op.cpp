#include "clear_scene_op.hpp"

#include <iostream>

#include "asset_mgr/drawable_mgr.h"
#include "asset_mgr/scene.h"
#include "ops/create_joint_op.hpp"
#include "ops/move_op.hpp"
#include "ops/rotate_op.hpp"
#include "selection.hpp"
#include "vk_ins/context.hpp"
#include "vp/auto_weight_feature.hpp"
#include "vp/deformer_feature.hpp"
#include "vp/mesh_csr_feature.hpp"

namespace ORL
{

ClearSceneOp::ClearSceneOp(vkkk::Scene& scene, vkkk::Context& context, ComponentManager& components,
    Selection& selection, ComponentId weight_id, ComponentId deformer_id)
    : scene(scene)
    , context(context)
    , components(components)
    , selection(selection)
    , weight_id(weight_id)
    , deformer_id(deformer_id)
{
}

void ClearSceneOp::on_eval(const InputEvent&) {
    if (create_joint != nullptr) {
        create_joint->cancel();
    }
    if (move != nullptr) {
        move->cancel();
    }
    if (rotate != nullptr) {
        rotate->cancel();
    }
    if (auto_weight != nullptr) {
        auto_weight->cancel_request();
    }
    if (deformer != nullptr) {
        deformer->unbind();
    }

    selection.clear();
    components.destroy_kind(ComponentKind::Joint);
    components.destroy_kind(ComponentKind::Curve);
    components.destroy_kind(ComponentKind::Constraint);

    if (auto* weight = components.weight(weight_id)) {
        weight->weights.clear();
        weight->weight_cnt = orlviewer::kDefaultWeightCount;
    }
    if (deformer == nullptr) {
        if (auto* data = components.deformer(deformer_id)) {
            *data = DeformerData{};
        }
    }

    scene.clear_objects();
    if (scene.drawable_mgr != nullptr) {
        for (const auto& name : scene.drawable_mgr->mesh_names()) {
            context.remove_mesh(name);
        }
        scene.drawable_mgr->clear_meshes();
    }
    if (csr != nullptr) {
        csr->clear();
    }
    std::cout << "Cleared scene\n";
}

} // namespace ORL
