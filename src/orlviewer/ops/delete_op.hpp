#pragma once

#include "asset_mgr/scene.h"
#include "component_manager.hpp"
#include "selection.hpp"
#include "vp_operation.hpp"

namespace ORL
{

// Deletes the current viewer selection. Scene objects are removed from the
// scene, joints remove their complete descendant subtree, and component
// deletion goes through ComponentManager so controller attachments are
// detached before their targets or controllers disappear.
class DeleteOp : public VpOperation<DeleteOp> {
public:
    DeleteOp(ComponentManager& components, vkkk::Scene& scene,
        Selection& selection)
        : components(components)
        , scene(scene)
        , selection(selection)
    {
    }

    void on_eval(const InputEvent& event);

private:
    ComponentManager& components;
    vkkk::Scene& scene;
    Selection& selection;
};

} // namespace ORL
