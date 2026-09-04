#pragma once

#include "component_manager.hpp"
#include "vp_operation.hpp"

namespace vkkk
{
class Scene;
class Context;
}

namespace ORL
{

class Selection;
class MeshCsrFeature;
class DeformerFeature;
class AutoWeightFeature;
class CreateJointOp;
class MoveOp;
class RotateOp;
class ScaleOp;

// Immediate reset: drop scene objects, meshes, joints, and bind state.
class ClearSceneOp : public VpOperation<ClearSceneOp> {
public:
    ClearSceneOp(vkkk::Scene& scene, vkkk::Context& context, ComponentManager& components,
        Selection& selection, ComponentId weight_id, ComponentId deformer_id);

    void set_csr(MeshCsrFeature& feature) { csr = &feature; }
    void set_deformer(DeformerFeature& feature) { deformer = &feature; }
    void set_auto_weight(AutoWeightFeature& feature) { auto_weight = &feature; }
    void set_create_joint(CreateJointOp& op) { create_joint = &op; }
    void set_move(MoveOp& op) { move = &op; }
    void set_rotate(RotateOp& op) { rotate = &op; }
    void set_scale(ScaleOp& op) { scale = &op; }

    void on_eval(const InputEvent& event);

private:
    vkkk::Scene& scene;
    vkkk::Context& context;
    ComponentManager& components;
    Selection& selection;
    ComponentId weight_id;
    ComponentId deformer_id;
    MeshCsrFeature* csr = nullptr;
    DeformerFeature* deformer = nullptr;
    AutoWeightFeature* auto_weight = nullptr;
    CreateJointOp* create_joint = nullptr;
    MoveOp* move = nullptr;
    RotateOp* rotate = nullptr;
    ScaleOp* scale = nullptr;
};

} // namespace ORL
