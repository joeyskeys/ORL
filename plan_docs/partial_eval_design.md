# ORL Partial Evaluation Design

## 1. Summary

ORL should keep the skeleton hierarchy as an implicit, data-oriented resource
and represent rig logic as a graph of solver and constraint subgraphs. A
controller edit should invalidate only the graph nodes, joints, descendants,
and deformed vertices that depend on that edit.

This is feasible and matches the direction of modern rigging systems, but the
current runtime cannot infer these dependencies automatically. ORL functions
currently receive broad `Joint[]`, `Locator[]`, and matrix buffers, while
hierarchy traversal is hidden inside functions such as
`joint_world_matrix()`. Partial evaluation therefore requires explicit
dependency metadata and runtime dirty-state tracking.

The recommended design is a hybrid:

- Keep the skeleton implicit instead of creating one graph node per joint.
- Make rig subgraphs explicit and modular.
- Add declared read/write footprints for graph nodes.
- Propagate dirty state through both the rig graph and the skeleton hierarchy.
- Use conservative full evaluation for opaque, global, or stateful nodes.

## 2. Goals

- Reevaluate only the necessary rig logic after a controller or locator edit.
- Keep unrelated skeleton branches unchanged.
- Preserve the existing compact joint ABI and GPU rendering path.
- Support local solvers such as leg IK and global solvers such as balance or
  full-body IK.
- Keep correctness when constraints introduce cross-branch dependencies.
- Allow the runtime to fall back to full evaluation when partial information is
  unavailable.

## 3. Non-goals

- Making every joint a graph node.
- Inferring arbitrary user-function dependencies from pointer-like buffer
  accesses.
- Replacing the existing solver/deformer stage split.
- Requiring every solver to support partial evaluation immediately.
- Assuming that a hierarchy subtree is always an independent evaluation unit.

## 4. Current Architecture and Limitations

### 4.1 Skeleton representation

The viewer stores joints in `ComponentManager` and `ComponentStore`. The
runtime still uses a packed pose array, and `Joint.parent` is currently a
packed-array index. That packed pose layout is an implementation detail of the
final animation runtime, not a hierarchy-plan identity system.

The hierarchy is therefore implicit in the joint buffer. ORL helper functions
walk parent indices to calculate world transforms. This is compact and
GPU-friendly, but the graph scheduler cannot see the individual hierarchy
dependencies. During animation evaluation, the topology is frozen after JIT
and no general dense-index mapping needs to remain in the hierarchy plan.

Stable component IDs remain the external identity during hierarchy-plan
construction. Any temporary index lookup needed while compiling is discarded
after the final animation runtime is published.

### 4.2 Graph execution

`GraphModule` already represents nodes, ports, connections, resource effects,
stage masks, and runtime nodes. `GraphSceneRuntime` builds a deterministic
execution plan and groups consecutive ORL nodes into compiled segments.

This provides:

- graph validation and topological ordering;
- solver/deformer stage separation;
- cached execution plans and compiled programs;
- device-only solver execution;
- CUDA device-buffer handoff to rendering and the deformer;
- whole-buffer versioned uploads.

It does not yet provide:

- per-node dirty propagation;
- per-element read/write ranges;
- affected-joint sets;
- cached output generations;
- partial CUDA dispatch;
- controller or locator change events.

At present, controller edits update scene input values and the active graph is
evaluated again as a whole on the next runtime update.

## 5. Two Dependency Graphs

The design should explicitly separate two related structures.

### 5.1 Skeleton hierarchy index

This is a data structure derived from the component hierarchy:

```text
stable joint ID
    -> parent stable joint ID
    -> preorder position
    -> subtree range
    -> depth
```

Suggested fields:

- `preorder_joints[]` containing stable IDs;
- `parent_joints[]` containing stable IDs when parent-chain mode is selected;
- `depth[]`;
- `level_offsets[]`;
- `subtree_begin[]`;
- `subtree_end[]`;
- optional flattened ancestor offsets and stable IDs;
- `topology_revision`.

If joints are stored in preorder, a subtree can often be represented by one
contiguous range. Otherwise, the runtime needs an explicit stable-ID list or
bitset.

`Joint.parent` remains a fixed pose-buffer field for ORL and GPU execution, but
it is not retained as the hierarchy plan's external identity. Stable IDs are
used to describe the compiled topology and solver relationships.

### 5.2 Rig dependency graph

The graph contains controller inputs, locator inputs, solver nodes, constraint
nodes, space conversions, and output/publish nodes.

Each node or subgraph should eventually declare:

- resources it reads;
- resources it writes;
- fields it reads or writes;
- affected joint set or region;
- whether it expands to ancestors or descendants;
- whether it is stateful or iterative;
- whether it is global or safe for partial evaluation;
- its required evaluation order.

A node that cannot provide these declarations is conservatively treated as
global and forces a larger evaluation.

### 5.3 Compiled hierarchy propagation

The fixed-topology assumption makes hierarchy propagation compilable. During
rigging, or when animation evaluation is enabled after a rig edit, ORL can
compile a hierarchy snapshot and reuse it for subsequent controller changes.

There is one important distinction:

- Solvers currently know the immediate `Joint.parent` indices because those
  values are present in `Joint[]`.
- Solvers and the scheduler do not know the precomputed ancestor,
  descendant, solver-region, or cross-constraint closure needed for partial
  evaluation.

The compiled snapshot fills this second gap. It does not need to turn joints
into graph nodes.

The snapshot and JIT runtime should be rebuilt when any of these change:

- joint creation, deletion, or reparenting;
- solver/constraint graph structure;
- solver footprint metadata;
- a dynamic space or constraint relationship that changes influence.

Controller TRS values, locator transforms, and ordinary pose values do not
require rebuilding the snapshot. They only create dynamic dirty changes.

## 6. Compiled Hierarchy Plan

### 6.1 Build-time steps

When the topology is considered fixed for a new animation/JIT generation:

1. Snapshot stable joint IDs and stable parent IDs.
2. Validate all parent references and reject cycles.
3. Generate preorder traversal, depth, level, and subtree ranges.
4. Generate the selected ancestor representation.
5. Compile solver and constraint footprints against stable IDs and final
   preorder positions.
6. Build controller/locator-to-solver reverse dependency lists.
7. Assign topology and dependency-plan revisions.
8. Upload immutable hierarchy data when GPU access is required.

The final animation runtime does not retain a general dense joint-order map.
The preorder array is the hierarchy traversal order, and subtree ranges refer
to positions in that array. Backend-specific numeric pose offsets may be
baked into generated execution artifacts, but they are not an authoritative
stable-identity map.

### 6.2 Proposed hierarchy data

The runtime-side plan can contain:

```text
HierarchyPlan
  topology_revision
  ancestor_storage_mode
  joint_count
  preorder_joints[joint_count]        // stable IDs
  depth[joint_count]
  level_offsets[level_count + 1]
  subtree_begin[joint_count]
  subtree_end[joint_count]
  parent_joints[joint_count]          // ParentChain mode only
  ancestor_offsets[joint_count + 1]   // Flattened mode only
  ancestor_joints[ancestor_count]     // stable IDs
```

`ancestor_storage_mode` is a compile option:

- `ParentChain`: retain the parent stable ID and walk the chain at runtime;
- `Flattened`: retain each joint's complete ancestor list.

`Flattened` is the default because it gives predictable propagation and avoids
repeating parent traversal during solver dependency checks. `ParentChain` is
available when memory size is more important than lookup speed.

For small rigs, a precomputed descendant bitset may be simpler and faster.
For larger rigs, subtree intervals or compressed stable-ID lists avoid the
`joint_count * joint_count` memory cost of a full bitset.

The plan should also contain compiled rig footprints:

```text
SolverRegion
  read_joints
  write_joints
  read_controllers
  read_locators
  propagation_rule
  stateful
  global
```

The hierarchy plan answers tree propagation questions. The solver-region map
answers graph and cross-constraint questions. Both are required.

### 6.3 Buffer and ABI placement

The existing 16-byte `SolverContext` header should remain unchanged. Variable
hierarchy arrays should not be embedded directly into that header.

Use one of these runtime representations:

- a separate immutable `solver_hierarchy` device buffer bound implicitly to
  solver segments; or
- a fixed hierarchy range in the packed runtime arena with a stable offset and
  range uploads only when topology changes.

The runtime can use the hierarchy buffer for propagation without changing ORL
function signatures. If ORL solver code itself must query precomputed
hierarchy data, expose it through a reserved hierarchy input or built-in
accessors rather than adding device pointers to `SolverContext`.

The dynamic pose/input arena remains separate in terms of versioning. Its
joint, locator, and controller values may change every frame; the hierarchy
plan should not be reuploaded for ordinary animation.

### 6.4 Runtime propagation

For each animation update:

```text
controller/locator change set
    -> reverse dependency lookup
    -> dirty solver and constraint nodes
    -> union their read/write joint regions
    -> expand world-transform descendants using HierarchyPlan
    -> include dependent skinning vertex regions
    -> schedule dirty graph regions in topological order
```

If a solver writes a local joint chain, the runtime marks that chain and the
descendant closure. If a solver reads a parent-space transform, the ancestor
closure is added before evaluating it.

If a node is global, stateful with invalidated history, or has an unknown
footprint, the runtime marks the corresponding evaluation island as full
dirty.

### 6.5 Fixed-plan invalidation

Every partial evaluation request should carry:

```text
topology_revision
dependency_plan_revision
pose_generation
```

The runtime must abandon partial evaluation and invoke a new JIT/plan build
when the topology or dependency revision differs. A pose generation change
only marks the affected dynamic regions dirty.

This preserves the rigging/animation distinction:

- rigging mode may add, delete, reparent, and reorder elements;
- animation mode reuses the compiled topology and changes values only.

### 6.6 Correctness validation

The first implementation should run partial and full evaluation side by side
in a validation mode:

1. Apply the same controller change to both paths.
2. Compare all joints, solver outputs, and deformed vertices.
3. Report the first mismatched region and the node that produced it.
4. Disable partial evaluation for a node whose declared footprint is
   insufficient.

Required regression cases include:

- local leg IK changing only the leg and its descendants;
- pelvis changes expanding to both legs and the torso;
- a cross-branch constraint expanding the dirty region;
- a stateful solver forcing its full evaluation group;
- reparenting invalidating and rebuilding the hierarchy plan;
- a new JIT generation preserving stable-ID dependency relationships.

### 6.7 Feasibility conclusion

The compiled hierarchy-map idea is doable and should be part of the design.
It can provide fast, deterministic ancestor/descendant propagation during the
animation phase without making the skeleton an explicit graph of nodes.

It is not sufficient by itself to decide which solver or constraint subgraphs
must execute. That requires the separate compiled solver-footprint and reverse
dependency maps described above. With both maps, local biped regions such as a
leg IK chain can avoid evaluating unrelated upper-body regions. Without solver
footprints, the hierarchy map can only safely optimize pure hierarchy
propagation, not arbitrary rig logic.

### 6.8 Terminology

The terminology should distinguish the hierarchy data from the complete rig
runtime artifact:

- `build_hierarchy_index()` or `precompute_hierarchy_map()` creates the
  parent/child lookup data.
- `compile_hierarchy_plan()` validates the hierarchy, derives traversal
  metadata, resolves solver regions, packs the result, and publishes a fixed
  runtime artifact.
- `compile_rig_evaluation_plan()` is the broader operation that also includes
  graph scheduling, solver dependencies, cached execution units, and backend
  artifacts.

For the complete ORL operation, "compiled hierarchy plan" is appropriate.
For the tree-only function, "build" or "precompute" is more precise than
"compile". The compilation step represents the transition to a final
animation runtime: after publication, rig edits are not applied in place.
Structural edits disable the current runtime and require another JIT and plan
build.

### 6.9 Detailed hierarchy-plan structure

The final CPU-side plan should retain stable IDs and traversal information,
not a general dense-index conversion table:

```cpp
struct HierarchyPlan {
    std::uint64_t topology_revision = 0;
    std::size_t joint_count = 0;
    AncestorStorageMode ancestor_storage = AncestorStorageMode::Flattened;

    // Final hierarchy traversal. IDs are the runtime's external identity.
    std::vector<ComponentId> preorder_joints;
    std::vector<std::uint32_t> depth;

    // Optional level-by-level evaluation boundaries.
    std::vector<std::uint32_t> level_offsets;

    // Descendant lookup in preorder space.
    std::vector<std::uint32_t> subtree_begin;
    std::vector<std::uint32_t> subtree_end;

    // Only one ancestor representation is retained.
    std::vector<ComponentId> parent_joints;
    std::vector<std::uint32_t> ancestor_offsets;
    std::vector<ComponentId> ancestor_joints;
};
```

The compile implementation may create temporary lookup tables while resolving
parent relationships, but those tables are discarded after the final runtime
artifact is produced. Stable IDs are used in the retained hierarchy plan and
in solver-region descriptions.

The existing packed pose buffer can continue to use its fixed `Joint.parent`
field. That field is part of the final pose ABI, not a retained hierarchy
identity map. If generated GPU code needs numeric pose offsets, those offsets
are baked into the generated execution artifact or node region during JIT.
They do not require keeping a general dense-index table.

The final plan does not need to retain child adjacency if subtree ranges and
the selected ancestor representation are sufficient. Child adjacency can be a
temporary compilation structure used to produce preorder, levels, and subtree
ranges.

If joints are traversed in preorder, every subtree is represented by:

```text
[subtree_begin[joint], subtree_end[joint])
```

Level ranges are optional, but useful when forward-kinematics or hierarchy
propagation is evaluated level by level:

```text
level_offsets[level] .. level_offsets[level + 1]
    -> joints at one hierarchy depth
```

### 6.9.1 Ancestor storage option

The compilation API should expose an explicit option:

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

In `ParentChain` mode, `parent_joints[i]` stores the parent stable ID for the
preorder entry. Runtime code follows the chain when needed.

In `Flattened` mode, the ancestor arrays use:

```text
ancestor_offsets[i] .. ancestor_offsets[i + 1]
    -> ancestors of preorder_joints[i]
```

The default flattened order should be root-to-parent so consumers can use the
list directly for parent-space preparation and propagation checks.

The flattened mode uses more memory but makes repeated dependency queries
predictable. Parent-chain mode is useful for deep or memory-constrained rigs
where ancestor queries are infrequent.

### 6.10 Solver-region structure

The hierarchy plan answers tree-propagation questions, but it does not answer
which graph nodes must execute. A separate compiled solver-region map is
required:

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

For a two-bone leg solver, the compiled region might contain:

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

The runtime should also build reverse maps:

```text
controller -> solver nodes
locator -> solver nodes
joint -> reading nodes
joint -> writing nodes
solver node -> affected joints
```

Built-in solver definitions can provide these footprints directly. Exported
user functions without footprint metadata should be classified as global and
use the full-evaluation fallback.

### 6.11 Hierarchy-plan build process

The hierarchy-plan build should happen when entering animation evaluation or
after a rig edit has completed:

1. Freeze the topology snapshot for the current evaluation generation.
2. Collect all joint records and their stable parent relationships.
3. Build a temporary parent-ID lookup for compilation only.
4. Validate missing parents, self-parenting, cycles, and unreachable joints.
5. Traverse from roots to assign stable-ID preorder, depths, and level ranges.
6. Calculate subtree ranges or explicit stable-ID descendant lists.
7. Generate the selected parent-chain or flattened-ancestor representation.
8. Resolve solver and constraint footprints to stable joint IDs and final
    preorder positions.
9. Build controller/locator reverse dependency lists.
10. Assign topology and dependency-plan revisions.
11. Pack immutable data for the CPU and, when needed, the GPU.
12. Bake any backend-specific numeric pose offsets into generated execution
    artifacts.
13. Publish the new plan atomically at a frame boundary.

The active plan must not be mutated while a solver or renderer is using it.
Build a replacement plan first, then swap it in between evaluation frames.

### 6.12 Static hierarchy data versus dynamic pose data

The existing `SolverContext` is a fixed 16-byte header containing counts. It
should not be expanded with variable-length arrays or device pointers.

The hierarchy data should instead use either:

- a separate immutable `solver_hierarchy` device buffer; or
- a fixed hierarchy range in a larger runtime arena, with range uploads only
  when topology changes.

The static buffer can contain:

```text
HierarchyHeader
preorder_joints[]          // stable IDs
parent_joints[]            // ParentChain mode only
ancestor_offsets[]         // Flattened mode only
ancestor_joints[]          // stable IDs, Flattened mode only
level_offsets[]
subtree_begin[]
subtree_end[]
```

The dynamic pose arena remains separately versioned:

```text
SolverContext
joints
locators
controllers
```

Controller TRS, locator transforms, and ordinary pose values can change every
frame without rebuilding or reuploading the hierarchy plan.

The runtime can use the hierarchy plan entirely on the CPU for dirty
propagation without changing ORL signatures. If ORL solver code itself must
query the plan, add a reserved hierarchy input or built-in accessors such as
`solver_parent_id(index)`, `solver_depth(index)`,
`solver_subtree_begin(index)`, and `solver_subtree_end(index)`. Backend
specific numeric pose offsets should be baked into the generated execution
artifact rather than retained as a general dense-index mapping. Device
pointers should not be added to `SolverContext`.

### 6.13 Animation-time usage

After the plan is built, each animation update follows this flow:

```text
controller/locator changes
    -> ChangeSet
    -> controller-to-solver reverse lookup
    -> dirty solver and constraint nodes
    -> union read/write joint regions
    -> ancestor expansion for required parent-space inputs
    -> descendant expansion for changed world transforms
    -> affected skinning vertex regions
    -> topological evaluation of dirty graph regions
    -> shared pose-buffer publication
    -> rendering
```

If a solver writes a local joint chain, the chain and its world-space
descendants become dirty. If a solver reads a parent-space transform, the
required ancestor closure is added first.

A node with a global footprint, unknown dependencies, invalidated stateful
history, or a dense dirty set should mark its whole evaluation island dirty.
This provides a safe fallback instead of relying on an incomplete map.

### 6.14 Rebuild and invalidation rules

Each partial evaluation request should carry:

```text
topology_revision
dependency_plan_revision
pose_generation
```

Rebuild the hierarchy and dependency plan when any of these structural
conditions changes:

- joint creation, deletion, or reparenting;
- solver or constraint graph structure;
- solver footprint metadata;
- dynamic space or constraint relationships that alter influence.

Any structural change requires another JIT and hierarchy-plan publication.
Only the dynamic pose generation changes for ordinary controller, locator, and
pose edits.

This preserves the rigging/animation distinction:

- rigging mode can modify the topology and invalidates the plan;
- animation mode reuses the fixed plan and changes values only.

### 6.15 Validation strategy

The first implementation should support a validation mode that compares
partial and full evaluation:

1. Apply the same controller change to both paths.
2. Compare all joints, solver outputs, and deformed vertices.
3. Report the first mismatched region and responsible node.
4. Disable partial evaluation for a node whose declared footprint is
   insufficient.

Important regression cases are:

- local leg IK changing only the leg and descendants;
- pelvis changes expanding to both legs and torso;
- a cross-branch constraint expanding the dirty region;
- a stateful solver forcing its full evaluation group;
- reparenting rebuilding the hierarchy plan;
- a new JIT generation preserving stable-ID dependency relationships.

## 7. Dirty Propagation Model

Viewport edits should publish a `ChangeSet` instead of directly triggering
recursive evaluation.

Example:

```text
controller TRS changed
    -> changed controller resource
    -> reverse graph dependencies
    -> affected solver/constraint nodes
    -> written joint set
    -> hierarchy closure
    -> affected skinning vertices
```

The `ChangeSet` should contain:

- changed controller and locator IDs;
- changed stable joint IDs;
- dirty graph node IDs;
- dirty joint bitsets or ranges;
- dirty mesh vertex ranges;
- resource generations;
- a topology-change flag.

Evaluation should be coalesced until the next viewport update. Multiple
controller edits during one drag should result in one propagated change set per
frame rather than repeated recursive graph execution.

### 6.1 Hierarchy propagation

If a joint's local transform changes, descendants whose world transforms depend
on it become dirty.

If a local solver writes a chain, the runtime marks the written chain and all
world-space descendants dirty.

This is not sufficient by itself. A constraint may read a distant joint,
locator, or controller, so graph dependency propagation must happen before
hierarchy descendant expansion.

### 6.2 Conservative fallback

The runtime should evaluate the full graph when any of the following applies:

- a node has unknown read/write footprints;
- a node reads the complete joint array;
- a full-body or global solver is active;
- a stateful feedback loop is invalidated;
- a topology change occurred and requires a new JIT;
- the affected region exceeds a configured threshold.

Correctness is more important than partial-evaluation coverage.

## 8. Example: Leg IK

For a leg IK controller, a useful dependency chain is:

```text
leg controller TRS
    -> target/pole-space conversion
    -> two-bone IK
    -> leg and foot joint rotations
    -> toe descendants
    -> mesh vertices influenced by those joints
```

The upper body can remain unchanged if:

- the controller space does not depend on upper-body joints;
- no upper-body constraint reads the leg result;
- the solver declares a local write region;
- no global balance or full-body solver consumes the change.

If the controller is in pelvis space, pelvis evaluation may be required even
when the rest of the upper body is not. If a balance solver reads both feet,
the change may expand to the spine and opposite leg.

Therefore the affected set is a dependency closure, not simply "the selected
joint's subtree."

## 9. Runtime Data Structures

### 8.1 Per-resource generations

Each significant resource should have a generation counter:

- controller inputs;
- locator inputs;
- local joint pose;
- world joint pose;
- hierarchy topology;
- solver state;
- skinning output.

Node state can then record:

```text
last_input_generations
last_output_generation
dirty_region
state_generation
```

A node can be skipped when all of its required input generations and regions
are unchanged.

### 8.2 Joint regions

Use a common representation for affected joints:

- one subtree interval when possible;
- several intervals for disconnected regions;
- a compact index list for sparse ranges;
- a bitset for arbitrary sets.

The runtime can choose the representation based on density. Dense regions
should remain contiguous for GPU access.

### 8.3 Solver metadata

A solver definition should eventually describe its footprint. For example:

```text
reads:
  controllers: {leg_ik}
  locators: {leg_target, leg_pole}
  joints: {ancestors(root), chain(root, mid, end)}

writes:
  joints: {chain(root, mid, end)}
  propagation: descendants
  stateful: false
```

The metadata can initially be authored in the node registry rather than added
to the ORL language. Language-level annotations can be added later when custom
solver authoring requires them.

## 10. GPU and Buffer Strategy

The current solver arena is optimized for one upload of the complete
fixed-topology input set. Partial evaluation introduces a second optimization
problem: transferring only changed regions.

Recommended progression:

1. Keep the current whole-arena upload for small rigs.
2. Track dirty byte ranges for joints, locators, and controllers.
3. Merge nearby ranges before uploading.
4. Add page-level or range-level device storage for large rigs.
5. Dispatch one fused kernel per evaluation island rather than one kernel per
   tiny node.
6. Fall back to the full kernel when the dirty region is dense.

Launching many small CUDA kernels can cost more than evaluating an entire small
character. Partial evaluation should therefore be measured against a
full-evaluation baseline.

The existing solver-to-deformer device handoff should remain. A partial solver
must publish the same stable device pose buffer, with unchanged regions
preserved and only affected ranges updated.

## 11. Stateful Solvers and Cycles

Constraints and solvers may form intentional feedback loops. Examples include
space switching, pinning, contacts, and iterative full-body IK.

These need explicit evaluation groups with:

- a fixed iteration order;
- state buffers;
- invalidation rules;
- convergence or iteration limits;
- a declared output region;
- a barrier before dependent nodes execute.

Partial evaluation must not reuse an old stateful result merely because one
input appears unchanged. The state generation and all group inputs must be
checked.

## 12. Comparison with Other DCC Designs

### Maya

Maya separates a scene DAG hierarchy from a dependency graph. Dirty
propagation marks downstream attributes, and the Evaluation Manager builds an
evaluation graph for serial or parallel scheduling.

This provides strong generality, but correctness depends on all node
dependencies being declared. Hidden plugin dependencies can defeat or
invalidate partial scheduling.

ORL should borrow Maya's explicit dependency and dirty-propagation model
without requiring every skeleton joint to become a graph node.

### Houdini KineFX/APEX

Houdini's modern KineFX architecture is the closest match. The skeleton is
data, while rig logic is represented by APEX graphs and evaluated separately.
APEX supports delayed evaluation and partial evaluation based on changed inputs
and requested outputs.

ORL's implicit skeleton plus discrete solver subgraphs follows this direction.

### Other systems

Many real-time rig systems use a similar compromise: a compact pose hierarchy
combined with a dependency graph or compiled evaluation schedule. They often
partition a rig into evaluation islands and use full evaluation for small
characters when dispatch overhead is greater than the saved computation.

## 13. Recommended Implementation Phases

### Phase 1: Explicit hierarchy index

- Add stable-ID parent relationships and final preorder traversal.
- Add depth, optional level ranges, subtree ranges, ancestor-storage mode, and
  topology revision.
- Default to flattened ancestor lists, with parent-chain mode as an option.
- Keep temporary index resolution inside the JIT build only.
- Keep `Joint.parent` as a fixed pose-buffer ABI field.
- Add tests for deletion, reparenting, cycles, ancestor modes, and subtree
  closure.

### Phase 2: Change sets and invalidation

- Emit controller/locator change records from viewport operations.
- Add per-resource generations.
- Add dirty joint sets and graph-node dirty flags.
- Continue evaluating the current full graph, but record affected regions.

### Phase 3: Footprint metadata

- Add read/write joint regions to node definitions.
- Add ancestor/descendant propagation rules.
- Mark unknown and global nodes as conservative barriers.
- Add validation for overlapping writes and unsupported partial nodes.

### Phase 4: Evaluation islands

- Partition the graph into reusable local regions.
- Cache node outputs and input generations.
- Execute only dirty regions in topological order.
- Preserve full-graph fallback.

### Phase 5: Partial GPU execution

- Add dirty range/page uploads.
- Add range-aware solver dispatch.
- Reuse the existing device pose handoff.
- Add thresholds comparing partial and full evaluation.

## 14. Success Criteria

The design is successful when:

- moving a local leg controller does not execute unrelated upper-body nodes;
- descendants and dependent constraints are still updated correctly;
- cross-branch and global dependencies expand the affected set correctly;
- opaque or unsupported nodes safely trigger full evaluation;
- GPU partial evaluation does not create excessive launch/upload overhead;
- the final pose and deformation match full evaluation exactly;
- topology edits require a new JIT and hierarchy-plan publication.

## 15. Final Assessment

The design is good and technically achievable. Keeping the skeleton implicit is
not a weakness; it is likely the right data-oriented choice for ORL. The
missing piece is an explicit dependency and invalidation layer around that
implicit hierarchy.

The first practical target should be annotated local evaluation islands such
as leg IK, arm IK, and facial control groups. Global solvers and unannotated
custom ORL functions should continue using full evaluation until their
footprints can be declared and validated.
