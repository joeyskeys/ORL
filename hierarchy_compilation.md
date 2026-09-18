# ORL Hierarchy Compilation

## 1. Purpose

This document defines how ORL can turn a rigging-time skeleton hierarchy into
a fixed animation-time runtime artifact.

The central assumption is:

> After a JIT/evaluation-plan build, the rig topology is fixed. Adding,
> deleting, reparenting, or structurally changing rig elements requires
> disabling the current animation runtime and starting another JIT build.

Controller TRS values, locator transforms, and ordinary pose values remain
dynamic. They create per-frame dirty changes but do not rebuild the hierarchy
artifact.

The goal is to make hierarchy propagation deterministic and inexpensive while
keeping the skeleton as data rather than turning every joint into a graph
node.

## 2. Terminology

There are three useful terms:

- `build_hierarchy_index()` or `precompute_hierarchy_map()` creates only the
  parent/child and traversal data.
- `compile_hierarchy_plan()` validates the hierarchy, creates traversal and
  ancestor data, resolves solver regions, packs immutable data, and publishes
  a fixed runtime artifact.
- `compile_rig_evaluation_plan()` is the broadest operation. It includes the
  hierarchy plan, graph scheduling, solver dependencies, cached execution
  units, and backend/JIT artifacts.

For ORL, the complete operation should be called hierarchy-plan compilation.
The tree-only operation is more accurately called hierarchy-map construction.

## 3. Design Decisions

### 3.1 Stable IDs are the retained identity

The final hierarchy plan retains stable component IDs. It does not retain a
general dense-index-to-ID or ID-to-dense-index table.

The current pose buffer can continue to use its packed `Joint.parent` field.
That is an ABI/runtime detail. It is not the identity system for the compiled
hierarchy.

During compilation, temporary lookup tables may be created to resolve parent
relationships and solver inputs. Those temporary tables are discarded after
the final animation runtime is published.

If generated GPU code needs numeric pose offsets, those offsets are baked into
the generated solver region or kernel artifact. They are not retained as a
general mapping structure.

### 3.2 Preorder is the hierarchy traversal order

The retained hierarchy order is:

```text
preorder_joints[] = stable IDs in hierarchy preorder
```

Subtree ranges refer to positions in this preorder array:

```text
[subtree_begin[i], subtree_end[i])
```

This lets the runtime mark a complete descendant subtree without retaining a
second dense joint-order mapping.

The existing packed pose-buffer order does not need to be changed in the first
implementation. Reordering that ABI would affect `Joint.parent`, weights,
deformer data, and project files.

### 3.3 Ancestor storage is configurable

Different rigs have different hierarchy depth and ancestor-query patterns.
The compile API should therefore expose an option:

```cpp
enum class AncestorStorageMode {
    ParentChain,
    Flattened,
};

struct HierarchyCompileOptions {
    AncestorStorageMode ancestor_storage =
        AncestorStorageMode::Flattened;
};
```

The default is `Flattened`.

`Flattened` stores each joint's complete ancestor list and provides predictable
ancestor queries. `ParentChain` stores only the parent relationship and walks
the chain when needed, reducing memory at the cost of repeated traversal.

The flattened list should be stored root-to-parent:

```text
ancestor_offsets[i] .. ancestor_offsets[i + 1]
    -> root, ..., direct parent of preorder_joints[i]
```

## 4. Current ORL Situation

The viewer stores joints through `ComponentManager` and `ComponentStore`.
The current `ComponentStore` maintains a deterministic joint packing order,
and `Joint.parent` refers to an index in that packed pose array.

ORL's `joint_world_matrix()` helper can follow those parent indices. Therefore
the solver already knows the immediate runtime parent relationship.

The missing information is the compiled closure needed by partial evaluation:

- all ancestors;
- all descendants;
- subtree ranges;
- level boundaries;
- solver read/write regions;
- controller and locator reverse dependencies;
- cross-constraint propagation.

The graph scheduler currently sees broad buffers and graph nodes. It does not
automatically know which joint elements a solver reads or writes.

## 5. Final HierarchyPlan

A conceptual CPU-side representation is:

```cpp
struct HierarchyPlan {
    std::uint64_t topology_revision = 0;
    std::size_t joint_count = 0;
    AncestorStorageMode ancestor_storage =
        AncestorStorageMode::Flattened;

    // Stable IDs in final hierarchy traversal order.
    std::vector<ComponentId> preorder_joints;

    // Optional level-by-level evaluation boundaries.
    std::vector<std::uint32_t> depth;
    std::vector<std::uint32_t> level_offsets;

    // Descendant lookup in preorder space.
    std::vector<std::uint32_t> subtree_begin;
    std::vector<std::uint32_t> subtree_end;

    // ParentChain mode.
    std::vector<ComponentId> parent_joints;

    // Flattened mode.
    std::vector<std::uint32_t> ancestor_offsets;
    std::vector<ComponentId> ancestor_joints;
};
```

The final plan does not need to retain child adjacency if subtree ranges and
the selected ancestor representation are sufficient. Child adjacency can be a
temporary build structure used to create preorder, depth, levels, and
subtree ranges.

For small rigs, a descendant bitset may be a useful alternative. For larger
rigs, preorder subtree intervals or compressed stable-ID lists avoid the
memory cost of a full `joint_count * joint_count` bitset.

### 5.1 Required invariants

The plan builder must guarantee:

- every stable joint ID appears exactly once in `preorder_joints`;
- each parent is either absent/root or references an existing joint;
- no self-parenting exists;
- no hierarchy cycle exists;
- every subtree range is valid and nested correctly;
- each flattened ancestor list is ordered root-to-parent;
- all plan arrays use the same final topology revision.

## 6. Solver Dependency Plan

The hierarchy plan answers tree-propagation questions. It does not determine
which graph nodes execute. A separate solver-region plan is required.

```cpp
struct SolverRegion {
    StableId node_id;

    StableJointSet read_joints;
    StableJointSet write_joints;

    std::vector<ComponentId> read_controllers;
    std::vector<ComponentId> read_locators;

    PropagationRule propagation;
    bool stateful = false;
    bool global = false;
};
```

The regions should also produce reverse lookup tables:

```text
controller -> solver regions
locator -> solver regions
joint -> regions that read it
joint -> regions that write it
solver region -> affected joint set
```

Built-in solvers can provide this metadata directly. Exported user functions
without read/write footprints should be treated as global and use full
evaluation.

### 6.1 Two-bone IK example

A leg IK region might declare:

```text
read controllers:
    leg_ik_controller

read locators:
    leg_target
    leg_pole

read joints:
    root ancestors
    root, mid, end chain

write joints:
    root, mid

propagation:
    descendants of mid

stateful:
    false

global:
    false
```

The hierarchy plan expands the changed chain into its world-space descendants.
The solver dependency plan determines that the two-bone node and its target and
pole conversion nodes must execute.

## 7. Hierarchy-Plan Compilation Process

The build happens when animation evaluation is enabled for a new rig/JIT
generation or after a structural rig edit.

### Step 1: Freeze the topology snapshot

Disable the current partial-evaluation runtime. Capture the final joint and
constraint topology at a frame boundary.

No solver or renderer may observe the plan while it is being rebuilt.

### Step 2: Capture stable parent relationships

The authoring representation should provide:

```text
joint stable ID
parent stable ID or root
local setup transform
```

The current runtime has a packed `Joint.parent` index. If stable parent IDs are
not yet stored by the authoring layer, the compiler can resolve the current
packed parent index to a stable ID during this snapshot step. That lookup is
temporary and is discarded after compilation.

Adding an explicit authoring parent ID later would make topology editing and
serialization safer, but it is not required for the first fixed-topology
runtime.

### Step 3: Validate topology

Reject:

- missing parents;
- self-parenting;
- cycles;
- invalid root references;
- disconnected or unreachable joints;
- duplicate stable IDs;
- inconsistent parent relationships.

The runtime should not publish a partial hierarchy plan when validation fails.

### Step 4: Build temporary traversal state

Use temporary build-only structures:

```text
parent_id_by_joint
children_by_parent
unvisited/visiting/visited state
```

These structures exist only while producing the final artifact.

### Step 5: Generate preorder traversal

Traverse from every root:

1. append the current stable ID to `preorder_joints`;
2. assign its depth;
3. remember the start position;
4. visit children;
5. close its subtree range after all descendants are visited.

The result is:

```text
preorder_joints
depth
subtree_begin
subtree_end
```

### Step 6: Generate level ranges

If level-by-level forward kinematics or parallel hierarchy propagation is
desired, group the preorder entries by depth and record:

```text
level_offsets[level] .. level_offsets[level + 1]
```

This is optional runtime data. It should only be retained when a consumer uses
it.

### Step 7: Generate ancestor data

Use the selected `AncestorStorageMode`.

#### Flattened mode

For every preorder joint:

1. follow its stable parent relationship;
2. collect the parent chain;
3. reverse it into root-to-parent order;
4. append it to `ancestor_joints`;
5. record the range in `ancestor_offsets`.

This is the default.

#### ParentChain mode

Store only:

```text
parent_joints[preorder_position]
```

Runtime code follows the chain when an ancestor query is needed.

### Step 8: Compile solver footprints

Resolve every built-in or annotated solver region against stable joint IDs,
controllers, and locators.

The compiler should record:

- direct reads;
- direct writes;
- ancestor requirements;
- descendant propagation rules;
- stateful barriers;
- global/full-evaluation status.

Any unresolved or opaque function becomes a conservative full-evaluation
region.

### Step 9: Build reverse dependencies

Create the reverse maps needed by animation-time dirty propagation:

```text
controller ID -> solver region IDs
locator ID -> solver region IDs
joint ID -> reading region IDs
joint ID -> writing region IDs
```

These maps are part of the final runtime plan because they are queried on
every controller change.

### Step 10: Create backend artifacts

The CPU runtime retains stable-ID hierarchy data and solver regions.

If a GPU solver needs hierarchy information, create an immutable hierarchy
buffer containing only the selected data:

```text
HierarchyHeader
preorder_joints[]
parent_joints[]          // ParentChain mode only
ancestor_offsets[]       // Flattened mode only
ancestor_joints[]        // Flattened mode only
depth[]
level_offsets[]
subtree_begin[]
subtree_end[]
```

The existing 16-byte `SolverContext` remains unchanged. Variable-length
hierarchy data must not be inserted into that header or represented as device
pointers inside it.

### Step 11: Bake backend-specific offsets

The stable-ID plan is the retained identity representation. If an ORL kernel
needs fixed numeric pose offsets, resolve them during JIT/code generation and
embed them in the generated execution artifact or solver region.

Do not retain a general dense-index mapping solely for this purpose.

### Step 12: Publish atomically

Assign:

```text
topology_revision
dependency_plan_revision
pose_generation
```

Build the replacement plan independently and publish it between frames. The
new JIT and runtime must reference the same plan revisions.

## 8. Animation-Time Evaluation

Once the plan is published, normal animation does not rebuild it.

```text
controller/locator changes
    -> ChangeSet
    -> reverse dependency lookup
    -> dirty solver and constraint regions
    -> union read/write stable joint sets
    -> ancestor expansion for parent-space inputs
    -> subtree expansion for world-space outputs
    -> affected skinning vertex regions
    -> topological execution of dirty regions
    -> shared pose-buffer publication
    -> rendering
```

For a leg IK controller, the upper body can remain untouched when:

- the controller space does not depend on the upper body;
- no upper-body constraint reads the leg result;
- no global solver consumes both legs;
- the leg solver declares a local write region.

If a pelvis or balance solver is involved, the dirty closure expands
accordingly.

## 9. Invalidation and New JIT Builds

The following require a new hierarchy-plan/JIT generation:

- joint creation or deletion;
- reparenting;
- topology or constraint relationship changes;
- solver graph edits;
- solver footprint metadata changes;
- dynamic spaces that alter dependency relationships;
- changes to the final pose-buffer layout.

The following do not require a new plan:

- controller TRS changes;
- locator transform changes;
- ordinary joint pose changes;
- animation frame changes.

Rigging mode is therefore a plan-building mode. Animation mode is a plan-use
mode.

If a structural edit is requested while animation evaluation is enabled:

1. disable the current animation runtime;
2. invalidate the current hierarchy and dependency plan;
3. apply the rig edit;
4. build and JIT a new plan when evaluation is re-enabled.

## 10. CPU and GPU Placement

The hierarchy plan can be used entirely on the CPU for dirty propagation. ORL
function signatures do not need to change for that design.

If ORL code itself must query the hierarchy plan, use a reserved hierarchy
input or accessors such as:

```text
solver_parent_id(index)
solver_depth(index)
solver_subtree_begin(index)
solver_subtree_end(index)
solver_ancestor_count(index)
solver_ancestor(index, offset)
```

Backend-specific numeric pose offsets should be generated by the JIT rather
than exposed as a general dense-index mapping.

The static hierarchy buffer should be uploaded only when topology changes. The
dynamic pose/input arena remains separately versioned:

```text
SolverContext
joints
locators
controllers
```

This preserves the current solver-to-deformer device handoff.

## 11. Stateful and Global Solvers

Partial hierarchy propagation is insufficient for:

- full-body IK;
- balance solvers;
- contact solvers;
- space-switch feedback;
- history-based solvers;
- nodes with unknown read/write footprints.

These regions need explicit evaluation groups with:

- iteration order;
- state buffers;
- invalidation rules;
- convergence/iteration limits;
- output regions;
- barriers before dependent regions.

When a stateful group is invalidated, the whole group should be evaluated.

## 12. Validation Strategy

The first implementation should support a validation mode that compares full
and partial evaluation:

1. Apply the same controller change to both paths.
2. Compare every joint result.
3. Compare solver outputs and state buffers.
4. Compare deformed vertices.
5. Report the first mismatched stable-ID region and responsible node.
6. Disable partial evaluation for a node whose footprint is insufficient.

Required tests:

- local leg IK affects only the leg and descendants;
- pelvis changes expand to both legs and torso;
- cross-branch constraints expand the dirty region;
- stateful solvers force their full group;
- cycles are rejected during compilation;
- parent-chain and flattened ancestor modes produce identical results;
- reparenting requires a new JIT;
- controller edits do not rebuild the hierarchy plan;
- stable-ID relationships survive plan regeneration.

## 13. Performance Considerations

The hierarchy plan avoids repeated topology traversal, but its construction
cost is not itself the main performance goal. The main benefit is avoiding
unrelated solver and deformation work during animation.

For small rigs, launching many partial GPU kernels can cost more than evaluating
the complete graph. The runtime should:

- merge adjacent dirty regions;
- use one fused kernel per evaluation island where possible;
- use a full-evaluation threshold;
- compare partial and full paths with instrumentation.

Partial evaluation should also eventually support dirty byte ranges or pages for
the dynamic pose arena. The static hierarchy buffer does not need per-frame
uploads.

## 14. Comparison with DCC Designs

Maya separates its scene DAG hierarchy from its dependency graph. Dirty
propagation identifies affected dependency nodes, and the Evaluation Manager
schedules the resulting evaluation graph.

Houdini KineFX/APEX is closer to this ORL design: the skeleton remains data,
while rig logic is represented by a separate graph that can be evaluated
later and partially.

ORL should combine these ideas without turning every joint into a graph node:

```text
stable-ID skeleton data
    + compiled hierarchy propagation
    + explicit solver footprints
    + graph dirty propagation
    + conservative full-evaluation fallback
```

## 15. Recommended Implementation Phases

### Phase 1: Stable-ID hierarchy build

- Capture stable parent relationships at the rig/JIT boundary.
- Validate cycles and missing parents.
- Build stable-ID preorder and subtree ranges.
- Add configurable ancestor storage.
- Default to flattened ancestor lists.
- Keep temporary lookup structures inside compilation only.

### Phase 2: Runtime plan publication

- Add topology and dependency-plan revisions.
- Store the static hierarchy plan separately from dynamic pose values.
- Publish plans atomically between frames.
- Invalidate the plan on structural edits.

### Phase 3: Solver footprint metadata

- Add stable-ID read/write joint regions.
- Add controller and locator reverse dependencies.
- Add ancestor/descendant propagation rules.
- Mark opaque and global solvers conservatively.

### Phase 4: Partial evaluation

- Add `ChangeSet` generation.
- Propagate dirty stable-ID regions.
- Cache node output generations.
- Execute only dirty evaluation islands.
- Preserve full-evaluation fallback.

### Phase 5: GPU optimization

- Upload the hierarchy buffer only on topology/JIT changes.
- Add dirty dynamic-pose ranges.
- Add range-aware solver dispatch.
- Preserve the device pose handoff to the deformer and renderer.

## 16. Final Assessment

The user's fixed-topology assumption is appropriate for animation evaluation.
The hierarchy can be built once per JIT generation and reused for all
controller edits in that generation.

The final plan should retain stable IDs, preorder traversal, subtree ranges,
and one configurable ancestor representation. It should not retain a general
dense joint-order mapping.

Flattened ancestor lists are the recommended default because they provide
predictable propagation and solver dependency queries. Parent-chain storage
remains available for rigs where memory is more important than lookup speed.

The hierarchy plan solves skeleton propagation. Solver-region metadata remains
necessary to determine which discrete graph subgraphs execute. Together, these
two compiled artifacts can make local biped evaluation practical while
retaining safe full-evaluation fallback for global or opaque rig logic.
