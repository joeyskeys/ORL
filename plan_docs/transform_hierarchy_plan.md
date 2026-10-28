# Unified Transform Hierarchy for Joints and Locators

## Status and dependency

This feature starts only after
`plan_docs/execution_stack_order_plan.md` is implemented, tested, and
accepted.

The execution-stack feature is the first phase because the transform
hierarchy needs a defined destination-writer composition policy. The
hierarchy will provide a base transform; the constraint stack will then be
able to override or modify that base without creating two independent
ordering systems.

The required dependency is:

```text
execution-stack ordering
    -> resolved constraint stacks
        -> transform hierarchy
            -> hierarchy + constraint + solver GPU fusion
```

## Goal

Provide a unified, explicit transform relationship for joints and locators so
that ordinary parent-space propagation is authored once and automatically
appears in the evaluation plan.

The first user-visible scenario is:

```text
leg_l_hip
    └── loc_leg_l_pole
```

Moving or evaluating `leg_l_hip` must move the pole locator by its stored
offset. An IK solver that reads the pole locator must automatically execute
after that locator transform has been produced. The user must not manually
mark the locator and solver order for this ordinary dependency.

The design must preserve:

- the existing joint ABI and packed joint buffer;
- stable component IDs as authoring identity;
- GPU-published locator buffers;
- fused CUDA no-host-readback execution;
- the execution-stack ordering rules from the preceding feature;
- full-evaluation fallback for opaque or unsupported relationships.

## Non-goals

The first implementation does not attempt to:

- make every component a general-purpose DAG node;
- replace the existing graph IR;
- change `Joint.parent` ABI layout;
- make controllers ordinary transform parents;
- infer parents permanently from spatial proximity;
- allow multiple unblended transform parents;
- silently apply two independent writers to one final transform;
- make a structural parent edit dynamic inside a running JIT artifact;
- expose device pointers or packed indices in project files.

Controllers may be supported as a later parent-space type, but existing
controller attachments remain a separate input-driving relationship in the
first hierarchy implementation.

## Current ORL situation

### Joints

The runtime already has a joint hierarchy:

- `orlrig::Joint.parent` stores a packed parent index;
- `ComponentStore` packs joints deterministically;
- `compile_hierarchy_plan` validates the joint tree and builds stable-ID
  preorder, depth, subtree, ancestor, and level data;
- `SceneGraphContext::ensure_hierarchy_plan` caches and publishes that plan;
- the GPU hierarchy buffer already contains the packed hierarchy artifact;
- `joint_world_matrix` and runners use the joint parent relationship.

This existing plan is joint-specific. It must be reused rather than
duplicated.

### Locators

`orlrig::Locator` currently contains only a world-space matrix. It has no
parent ID, local-space matrix, inheritance mask, or relationship metadata.
`ComponentManager::set_locator_world_xform` therefore writes a standalone
world transform.

Locator relationships are currently represented indirectly through graph
constraints, controller attachments, or explicit solver handle use. The graph
compiler can know that a solver reads a locator, but it cannot derive how that
locator follows a joint or another locator.

### GPU and drawing

`SceneInputCatalog` packs locator transforms and publishes computed locator
device views. `LocatorFeature` already prefers the published GPU locator view
when available. The hierarchy feature must preserve this direction:

```text
authored inputs / compiled hierarchy
    -> solver or hierarchy GPU writes
        -> published locator device view
            -> locator drawing and GPU consumers
```

It must not add a host readback merely to draw a parented locator.

### Existing controller relationships

`ComponentManager` has `ControllerAttachment`, which stores a controller
target and a target-local setup offset. It maps controller input to a joint or
locator. This is an input-driving relationship, not a general transform-parent
edge.

The implementation must not accidentally treat controller attachment and
transform parenting as the same edge. Their composition must be explicit:

```text
controller input
    -> attached target authoring/input layer
        -> transform hierarchy base
            -> constraint stack
                -> final transform
```

## Transform relationship semantics

### Parent relation

Introduce a viewer/rig authoring relation conceptually equivalent to:

```text
TransformParent {
    child: stable component ID
    parent: stable component ID or World
    local_offset: matrix
    inherit_translation: bool
    inherit_rotation: bool
    inherit_scale: bool
    enabled: bool
}
```

The first supported parent kinds are:

- Joint;
- Locator;
- World/no parent.

The child is initially limited to a locator. Joint-to-joint parenting
continues to use the existing joint hierarchy representation until the
unified plan can safely represent both.

### Transform equation

For a fully inherited parent:

```text
child_base_world = parent_final_world * local_offset
```

For partial inheritance, decompose the parent transform according to the
inheritance mask, then compose the selected parent components with the local
offset. This must be a named, tested operation; do not rely on ambiguous
matrix element replacement.

An unparented locator uses its authored world transform as its base transform.
When a parent is assigned with “maintain world transform,” compute:

```text
local_offset = inverse(parent_current_world) * child_current_world
```

The stored offset is then used for all later evaluation.

### Base versus final transform

The parent relation produces a base transform. It does not directly compete
with the constraint stack.

The complete per-destination pipeline is:

```text
authored local/world value
    -> transform hierarchy base result
        -> resolved constraint stack
            -> final joint/locator result
                -> downstream readers
```

A simple existing parent constraint should be migrated to this parent
relation or lowered as the base-layer operation. It must not be applied both
as a hierarchy relation and as a second independent writer.

If a constraint intentionally overrides the parented result, it is a stack
writer after the base layer. Its ordering is handled by the execution-stack
feature, not by a new hierarchy priority.

## Data ownership and ABI boundaries

### Authoring storage

Store transform-parent metadata outside the 128-byte joint and matrix-sized
locator payloads. A suitable first location is a relation store owned by
`ComponentManager` or by a rigging-side relationship store referenced by it.

The relation store owns:

- stable child/parent IDs;
- local offset;
- inheritance flags;
- enabled state;
- relation revision;
- optional authoring provenance/name.

The packed `Locator` payload remains a runtime transform value. It may hold a
CPU materialized result for compatibility and fallback, but the relationship
metadata must not be encoded into its ORL matrix bytes.

### Compiled storage

Create a `TransformHierarchyPlan` that combines:

- a reference or embedded copy of the existing joint `HierarchyPlan`;
- transform nodes for supported locator children;
- parent edges between joint and locator stable IDs;
- a topological transform order;
- packed joint/locator indices;
- local-offset and inheritance data;
- reverse dependency maps;
- topology/relation revision.

An alternative is to extend `HierarchyPlan` with typed transform nodes. The
first implementation should prefer a separate `TransformHierarchyPlan` to
avoid destabilizing joint-only consumers. The two plans must share the same
stable-ID and revision rules.

Conceptual compiled data:

```text
TransformNode {
    stable ID
    kind: joint or locator
    parent node index or world
    packed joint index or locator index
    offset index
    inheritance mask
}

TransformHierarchyPlan {
    topology revision
    relation revision
    nodes in topological order
    parent edges
    offsets
    inheritance masks
    level ranges
    child/reverse dependency ranges
}
```

Runtime GPU structures must use fixed-width packed indices and offsets. Stable
IDs remain on the CPU/compiler side and in project files.

## Authoring workflow

### Bind/reparent operation

Provide a viewport and graph-adjacent operation:

1. select the child locator;
2. shift-select a joint or locator parent;
3. choose `Bind Transform Parent`;
4. default to `Maintain World Transform`;
5. compute and store the local offset;
6. validate that the relation does not create a cycle;
7. invalidate the transform/evaluation plan;
8. show the parent link and offset in the UI.

The operation should also support:

- `Bind Without Offset`;
- `Reparent`;
- `Clear Transform Parent`;
- `Match Parent`;
- toggle translation, rotation, and scale inheritance;
- edit the local offset;
- temporarily mute the relation.

### Hierarchy panel

Add a compact transform hierarchy view:

```text
World
└── root_joint
    └── leg_l_hip
        └── loc_leg_l_pole
```

The panel should display:

- parent and child;
- object kind;
- local offset;
- inheritance flags;
- constraint-stack indicator;
- cycle/conflict diagnostics;
- whether the relation is authored, muted, or invalid.

Dragging a locator onto a joint/locator can be a convenience action, but it
must show the maintain-offset choice before committing.

### Existing scene objects

For an existing locator with no relation, preserve its current world matrix.
For a newly created relation, capture the current world transform as the
offset-preserving local value. Never infer a parent silently from the nearest
joint.

## Compilation process

### Step 1: Require the execution-stack feature

Before starting this feature, verify:

- `EvaluationPlan` has structured conflict/ordering results;
- constraint order metadata is serialized;
- unresolved stacks stop JIT;
- the resolver can apply a stack order;
- all current constraint tests pass.

The transform hierarchy compiler must call the resolved-order API rather than
reimplementing writer ordering.

### Step 2: Freeze authoring topology

When evaluation is enabled or a structural relation changes:

1. stop the current evaluation artifact;
2. freeze graph and hierarchy editing;
3. capture a consistent component/relation snapshot;
4. record graph, scene-input, topology, relation, and evaluation revisions.

Pose edits can remain dynamic after publication. Parent/reparent/offset
relationship edits require a new compiled plan.

### Step 3: Compile the existing joint plan

Call `compile_hierarchy_plan` for the existing joint tree. Keep all current
invariants:

- valid packed parent indices;
- no self-parent;
- no missing parent;
- no joint cycle;
- deterministic stable-ID preorder;
- valid ancestor/subtree/level arrays.

Do not replace `Joint.parent` or change its ABI in this feature.

### Step 4: Resolve locator relations

For every enabled transform-parent relation:

1. resolve child stable ID;
2. resolve parent stable ID;
3. verify child kind is supported;
4. verify parent kind is joint, locator, or World;
5. resolve packed joint/locator indices;
6. validate local offset values are finite;
7. validate inheritance flags;
8. reject a child with more than one active transform parent.

Missing or deleted references are compile diagnostics. They must not silently
fall back to world space.

### Step 5: Build the transform graph

Create typed transform nodes for:

- all joints in the existing hierarchy;
- all parented locators;
- unparented locators if they need to participate in a uniform topological
  publication stage.

Add edges:

- joint parent -> joint child;
- joint/locator parent -> locator child;
- world -> root/base nodes conceptually, without a runtime node if not needed.

Use a cycle-detection topological sort. Reject:

- joint-to-locator-to-joint feedback;
- locator A parented to locator B while B is parented to A;
- longer locator cycles;
- a relation whose parent final transform depends on the child.

The error should name the complete cycle.

### Step 6: Compute compiled levels and ranges

Produce:

- topological transform order;
- level buckets for independent base propagation;
- parent index per transform node;
- child/reverse ranges for dirty propagation;
- mapping from stable IDs to compiled node positions;
- mapping from compiled nodes to packed joint/locator offsets.

Keep the stable-ID lookup tables temporary unless a runtime consumer needs a
validated compact table. Do not retain a general authoring map in GPU ABI.

### Step 7: Pack offsets and inheritance data

Pack immutable relation data into a static hierarchy/transform buffer:

```text
TransformParentHeader
transform node records
parent indices
local offsets
inheritance masks
level offsets
subtree/reverse ranges
```

The buffer is uploaded only when relation/topology revisions change. Dynamic
joint, locator, and controller values remain in the existing packed state
arena.

### Step 8: Connect the constraint stack

For each transform destination:

1. publish the hierarchy base writer;
2. append resolved constraint-stack writers;
3. add base-before-constraint edges;
4. add constraint-writer-to-reader edges;
5. add constraint order edges from the execution-stack plan;
6. detect cycles introduced by a constraint reading a final transform that
   depends on itself.

For a pole locator:

```text
leg_l_hip base/final transform
    -> loc_leg_l_pole hierarchy result
        -> optional locator constraints
            -> ik_two_bone reads pole
```

If a simple parent constraint already represents the same edge, migrate it to
the base relation or mark it as the stack's base writer. Never emit both
writers.

### Step 9: Build dirty propagation

Add reverse dependencies:

```text
parent transform -> child transform nodes
joint/locator input -> affected base nodes
base node -> constraint writers
constraint output -> downstream readers
```

When a parent joint moves, mark all descendant locator base transforms and
their downstream constraint/solver regions dirty. When a locator's local
offset changes, mark that locator and its descendants dirty. When a
constraint output changes, use the execution plan's existing affected-region
rules.

A clean parented locator must not be recomputed every frame if neither its
parent nor local offset changed.

### Step 10: Publish the plan atomically

Build replacement artifacts off to the side. Publish only when:

- component topology revision is unchanged;
- relation revision is unchanged;
- graph/evaluation revision is unchanged;
- constraint-stack dependency revision is unchanged;
- backend artifact creation succeeded.

Clear old computed joint/locator device views before destroying artifacts that
own them. Publish the new hierarchy and solver views between frames.

## CPU runtime integration

### Authoring mode

When evaluation is disabled, the user may edit a locator's local transform or
reparent it. The editor should update the relation offset according to the
chosen space:

- editing world transform with maintain-parent-space: recompute local offset;
- editing local transform: preserve relation and recompute world preview;
- clearing parent: convert current final world transform into standalone
  authored world space.

### Compatibility materialization

The CPU component store may materialize the computed world result into the
locator payload for legacy features and CPU fallback. The materialized value
must be tagged or revisioned so a stale host mirror is not mistaken for a
user-authored edit after GPU evaluation.

Reuse the existing device-state policy:

- authored changes upload to the device arena;
- GPU-produced final locator views remain device-authoritative;
- host readback occurs only on explicit fallback/debug/writeback paths;
- renderer consumes the published device locator view when available.

### Controller attachments

Apply controller input to its target according to the existing attachment
semantics before the transform hierarchy base stage, unless a future explicit
space-switch feature changes that policy.

Validate controller/hierarchy combinations:

- a controller may drive a parent joint;
- a controller may drive a parented locator;
- a controller must not create a feedback cycle through its target's parent
  chain;
- controller attachment and transform parent must not both claim the same
  final write without a documented layer.

## GPU and fused execution integration

### Phase 1: Correct CPU/segmented behavior

First integrate the transform hierarchy into CPU evaluation and segmented
GPU execution with the same packed device state. This provides a correctness
reference before fusing propagation into one kernel.

The hierarchy stage must write locator transforms into the same device-backed
locator arena that:

- ORL `find_locator` handles read;
- constraint writers update;
- `LocatorFeature` publishes for drawing;
- downstream deformer/solver consumers reference.

Do not copy a parented locator back to the host merely to feed the next
solver or draw pass.

### Phase 2: Fused base propagation

When the fused CUDA solver is eligible:

1. include static transform-hierarchy data as an immutable input;
2. lower base hierarchy propagation into the fused execution order;
3. place base propagation before any stack writer that consumes it;
4. place IK/solver readers after the final locator/joint output;
5. keep all intermediate values in the unified device arena;
6. publish final joint/locator device views after the fused launch;
7. verify zero download calls and bytes.

If a relation requires a host adapter, unsupported matrix operation, stateful
feedback, or unresolved stack, use the existing conservative fallback and
report why fusion is unavailable.

### GPU drawing

`LocatorFeature` must continue to select:

1. published computed locator device view when valid;
2. host-authored locator data only when no computed device view exists.

A parented locator that has been evaluated on the GPU must not be drawn from
the stale `ComponentManager` matrix.

## Serialization and migration

### Project data

Serialize transform-parent relationships as authoring data, not runtime
evaluation state. Each relation should contain:

- child stable component identity;
- parent stable component identity and kind;
- local offset;
- inheritance flags;
- enabled/muted state;
- optional relation version.

Do not serialize packed joint/locator indices, GPU pointers, device buffer
addresses, or current execution phase.

The project format must be versioned. Existing projects with no locator
relations load with all locators unparented and preserve their current world
matrices.

### Loading

When loading:

1. create components and their authored world transforms;
2. load relation records after all stable IDs are known;
3. validate references and cycles;
4. keep evaluation disabled by default;
5. expose relation diagnostics before enabling evaluation;
6. compile the relation plan only when evaluation is explicitly enabled.

Invalid relation records must not be silently dropped. The UI should identify
the missing parent/child and offer clear/remove relation actions.

### Reparent migration

If a future project format replaces a simple parent constraint with a
transform-parent relation:

- recognize the old simple parent constraint;
- preserve its target and offset;
- create one equivalent base relation;
- remove or mark the old writer as migrated;
- reject migration when the old constraint has blending, multiple sources, or
  behavior that cannot be represented by one parent.

The migration must not create duplicate transform writers.

## Invalidation rules

Require a new transform/evaluation plan and JIT artifact for:

- locator parent assignment/removal;
- locator reparenting;
- relation offset or inheritance changes;
- joint creation/deletion/reparenting;
- component ID/name changes that affect stable relation references;
- constraint-stack changes;
- solver read/write footprint changes;
- packed buffer topology changes.

Do not rebuild the topology artifact for:

- controller pose edits;
- locator local/world pose edits that preserve the same relation;
- ordinary joint TRS edits;
- frame/time changes.

Those changes produce dirty ranges and dependency closure updates only.

## Viewer state and feedback

### Structural relation edit

When a user creates/reparents/clears a relation while evaluation is enabled:

1. disable or pause the current runtime;
2. freeze evaluation-dependent graph edits;
3. apply the relation transaction;
4. invalidate hierarchy, constraint-stack, and execution-plan revisions;
5. return to disabled/editing state;
6. compile again only after the user explicitly enables evaluation.

### Compile failure

On hierarchy or combined-stack failure:

- show the relation cycle/missing-reference diagnostic;
- do not publish partial hierarchy data;
- clear stale computed device views;
- disable evaluation;
- unlock authoring;
- preserve the relation data so the user can repair it.

### Visual feedback

Show parent links in the hierarchy panel and viewport. Display:

- base-parent link;
- constraint-stack writers;
- final transform source;
- warning badges for muted, unresolved, or invalid relations.

This is important because a locator may be visually stationary while its
authored local transform is unchanged but its computed world transform is
changing through a parent.

## Implementation phases

### Gate 0: Execution-stack prerequisite

Confirm the acceptance criteria in
`plan_docs/execution_stack_order_plan.md`.

No transform-hierarchy code should be merged before:

- structured unresolved-order results exist;
- order metadata persists;
- a selected stack order is executed consistently;
- unresolved plans fail closed;
- fused/fallback order parity tests pass.

### Phase 1: Authoring relation model

- Add relation storage to the viewer/rig authoring layer.
- Add stable child/parent IDs, offset, inheritance flags, and revisions.
- Add create/reparent/clear/mute APIs.
- Add cycle/reference validation.
- Preserve current locator and joint ABIs.

Exit criteria:

- relation can be created and removed without evaluation;
- maintain-world-offset behavior is numerically correct;
- relation data is independent of packed indices.

### Phase 2: Project serialization

- Add versioned relation section.
- Load old projects with unparented locators.
- Save relation metadata deterministically.
- Add malformed-reference and migration diagnostics.
- Keep evaluation state excluded from persistence.

Exit criteria:

- relation round-trips;
- old projects remain loadable;
- missing references fail visibly.

### Phase 3: Transform hierarchy compiler

- Reuse existing joint `HierarchyPlan`.
- Add `TransformHierarchyPlan` for joint/locator edges.
- Build topological order, levels, packed indices, offsets, and reverse maps.
- Add relation/topology signatures and atomic publication.
- Integrate parent-cycle diagnostics.

Exit criteria:

- joint-parented locator follows the joint;
- locator-parented locator follows its locator;
- cycles and invalid references are rejected;
- unaffected transforms remain unchanged.

### Phase 4: Constraint-stack integration

- Add base hierarchy writer to the execution-plan dependency graph.
- Make simple parent constraints migrate or lower as base writers.
- Apply resolved stack overrides after base propagation.
- Detect duplicate writers and feedback cycles.
- Add dirty closure from parent to locator to solver.

Exit criteria:

- parent hierarchy plus override constraint produces exactly one final result;
- IK reads the final parented locator;
- no double-transform occurs;
- constraint resolver can order post-parent writers.

### Phase 5: CPU and segmented GPU runtime

- Propagate parented transforms before graph consumers.
- Pack static relation data separately from dynamic pose/input state.
- Update `SceneInputCatalog` bindings and revisions.
- Publish computed locator views.
- Make locator drawing consume GPU views.
- Preserve host fallback and debug writeback.

Exit criteria:

- CPU and segmented GPU results match;
- drawing uses final computed GPU locator data;
- no unnecessary host readback occurs between solver stages.

### Phase 6: Fused CUDA hierarchy

- Add transform hierarchy static data to fused execution inputs.
- Lower base propagation into the fused graph/program.
- Preserve stack order and downstream solver dependencies.
- Keep joint/locator state in one device arena.
- validate zero-download telemetry.
- invalidate all borrowed device views on rebuild/unbind/backend change.

Exit criteria:

- one fused CUDA execution produces final joint and locator views;
- IK and deformer consumers use those views directly;
- no host readback is required for parented locator drawing;
- fallback remains correct for unsupported cases.

### Phase 7: UI and diagnostics

- Add hierarchy panel/tree.
- Add bind/reparent/clear actions.
- Add viewport parent-link visualization.
- Add relation conflict/cycle diagnostics.
- Add resolver links from a failed plan to the relevant relation/constraint.

Exit criteria:

- a user can author the pole-parent relation without editing JSON;
- the hierarchy and constraint stack are understandable in the viewer;
- failures are actionable.

## Test matrix

### Relation and compiler tests

- unparented locator remains unchanged;
- locator parented to a root joint;
- locator parented to a deep joint;
- locator parented to another locator;
- multi-level joint -> locator -> locator chain;
- maintain-world offset preserves current world transform;
- no-offset bind matches parent transform;
- translation-only inheritance;
- rotation-only inheritance;
- scale-only inheritance;
- invalid parent ID;
- child parented to itself;
- joint/locator cycle;
- locator chain cycle;
- deterministic topological ordering;
- stable IDs survive packed-order changes;
- relation revision invalidates the plan.

### Constraint-stack tests

- hierarchy base followed by one parent/override constraint;
- two same-destination constraints use stored stack order;
- hierarchy base plus unresolved competing writers prompts/fails closed;
- constraint reading final locator executes after hierarchy and stack;
- constraint cycle is rejected;
- simple parent constraint migration creates one writer only;
- complex/multi-source parent constraint is not silently migrated.

### Runtime parity tests

- CPU and segmented CUDA parent propagation match;
- fused CUDA matches CPU reference;
- dirty parent update reaches all locator descendants;
- unrelated branches remain clean;
- changing local offset dirties only affected subtree and consumers;
- GPU locator draw uses the published device view;
- no solver-to-solver locator readback in fused mode;
- host fallback explicitly reports when readback occurs.

### Serialization and UI tests

- relation records round-trip;
- legacy projects load unparented;
- evaluation remains disabled after load;
- invalid relation diagnostics name stable IDs;
- bind/reparent/clear preserve or intentionally change world transform;
- resolver choices persist;
- stale resolver choices are rejected after relation edits.

## Performance and correctness policy

The hierarchy plan is compiled infrequently; the primary performance target is
animation-time propagation and avoiding unrelated solver work.

The runtime should:

- keep static relation data on the device across frames;
- upload only changed dynamic ranges;
- use level buckets for independent base propagation;
- union parent dirty ranges before dispatch;
- use a full-evaluation threshold for small rigs or broad constraints;
- compare fused and fallback results in validation builds;
- log hierarchy and stack barriers separately from ordinary kernel time.

Correctness takes priority over partial dispatch. Any unknown or opaque
relationship expands to a conservative region or disables the optimized path.

## Acceptance criteria

The transform-hierarchy feature is complete when:

1. locators can be parented to joints or locators with maintainable offsets;
2. relation edits are serialized as authoring data;
3. cycles and missing references fail with actionable diagnostics;
4. the compiled transform plan has deterministic stable-ID order;
5. the existing joint hierarchy ABI remains compatible;
6. hierarchy output is a base layer for the resolved constraint stack;
7. no destination has two uncomposed final transform writers;
8. IK and other consumers automatically receive final locator transforms;
9. CPU, segmented CUDA, fused CUDA, and fallback paths agree;
10. locator drawing consumes the published GPU locator buffer;
11. ordinary pose edits do not rebuild topology/JIT;
12. structural relation edits do rebuild the required artifacts;
13. no host readback is introduced merely to propagate or draw locators;
14. the viewer provides usable bind/reparent/clear and diagnostic workflows.

The implementation may then extend the same model to controller spaces,
multiple weighted parents, space switching, or more advanced constraint
blending. Those are separate features and must not weaken the single-writer
and cycle-detection guarantees established here.
