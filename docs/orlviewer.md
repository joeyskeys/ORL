# `src/orlviewer`: editor, viewport, and scene integration

## Scope and current status

This document describes the current `src/orlviewer` implementation at commit
`4e0fb3d`. It covers:

- application startup and vkkk/Vulkan frame ownership;
- viewer/component ownership boundaries;
- scene graph context and revision tracking;
- graph-to-scene input packing;
- staged solver/deformer runtime;
- typed handles and CPU/CUDA device handoff;
- project serialization;
- controls, selection, viewport operations, and Qt panels;
- rendering/picking/mesh/auto-weight features;
- dependencies, tests, and current discrepancies.

The viewer is an application adapter around `orlexec`, `orlrig`, and
`orlgraph`. It is not the owner of the core graph model or compiler:

```text
vkkk Scene / Vulkan context
          |
          v
ComponentManager
  viewer metadata + authoring state
          |
          v
SceneGraphContext
  graphs + revisions + mappings + plans
          |
          +-----------------------+
          |                       |
          v                       v
SceneInputCatalog          GraphSceneRuntime
  scene bindings            adapters + ORL segments
          |                       |
          +-----------+-----------+
                      v
               orlexec / orlrig
                      |
                      v
          CPU result or CUDA/Vulkan output
```

The current viewer target is monolithic: `orl_viewer` directly compiles most
viewer sources. The viewer test target also compiles selected viewer sources
directly; there is no reusable `orlviewer` library target.

## Build and dependency boundary

`src/orlviewer/CMakeLists.txt` creates one executable:

```text
orl_viewer
```

It links:

- `vkkk`;
- `orlexec`;
- `orlrig`;
- `orlgraph_io`;
- `Eigen3::Eigen`;
- `RapidJSON`.

The viewer target is included by the root project only when
`ORL_BUILD_VIEWER=ON`, which defaults to on. `orlgraph_io` must exist, so
RapidJSON is effectively required for the viewer.

Compile definitions provide:

```text
ORL_VKKK_SOURCE_DIR
ORL_RESOURCE_DIR
ORL_USE_QT6
```

The `advanced_gui` root option selects Qt6 when available. Otherwise the
viewer uses GLFW through vkkk. vkkk may build both backends, but the selected
viewer compile path chooses one.

The viewer depends indirectly on the LLVM/CUDA/TBB requirements of
`orlexec`/`orlrig`. In the current root build, `orlexec` is skipped when
`orlcomp_codegen` is unavailable, so a frontend-only build cannot produce the
viewer.

## File inventory

### Application and state

| File | Responsibility |
| --- | --- |
| `main.cpp` | Startup, vkkk context, feature composition, event/render loop |
| `runtime_config.hpp` | Process-wide CPU/GPU and ORL-evaluation switches |
| `component_manager.hpp/.cpp` | Viewer adapter around `orlrig::ComponentStore` |
| `scene_graph_context.hpp/.cpp` | Graph/scene mappings, revisions, plans, writeback |
| `graph_scene_inputs.hpp/.cpp` | Scene binding descriptors, packing, handles, CUDA arenas |
| `graph_scene_runtime.hpp/.cpp` | Graph segment compilation, runtime adapters, staged evaluation |

### Project, controls, and selection

| File | Responsibility |
| --- | --- |
| `project_serialization.hpp/.cpp` | Project JSON and editor layout persistence |
| `control_map.hpp/.cpp` | Input-to-operation dispatch, scopes, modal operations |
| `selection.hpp` | Selection references and transform attribute abstraction |
| `vp_operation.hpp` | CRTP operation interface |
| `camera_navigator.hpp` | Orbit/pan/zoom and camera frame behavior |

### Viewport features

| Area | Files |
| --- | --- |
| Mesh | `vp/scene_mesh_feature.hpp`, mesh picking |
| Joints | `vp/joint_feature.hpp`, `joint_picking_feature.hpp` |
| Controllers/locators | `vp/controller_feature.hpp`, `locator_feature.hpp` |
| Rig compute | `vp/solver_feature.*`, `deformer_feature.*`, `auto_weight_feature.*` |
| Mesh preprocessing | `vp/mesh_csr_feature.*` |
| Viewport UI | grid, axes, HUD, transform guide, display modes |

### Operations

Operations cover model loading, scene clearing, joint/locator/controller/IK
creation, deletion, mirroring, selection, transforms, attachment toggling,
controller curve cycling, camera switching, and display modes.

### Optional Qt/editor files

When Qt6 is selected:

- `qt/node_graph_editor.*` provides a graph canvas and stage/layout editing;
- `qt/property_editor.*` edits selected components and names;
- node graph operation helpers support frame/copy/paste behavior.

### Resources

The viewer consumes:

- `resource/shaders/*.vert`, `*.frag`, `*.comp`;
- `resource/config/control_map.json`;
- vkkk font and shader assets through `ORL_VKKK_SOURCE_DIR`.

## Startup architecture

### Startup flags and caches

`main.cpp` parses only:

```text
--recompile
--recompile-shaders
```

Both force shader/pipeline cache recompilation.

The working directory determines:

```text
.orlviewer_spirv_cache/
orlviewer.pipeline.cache
```

The vkkk context is configured with the selected window backend, Vulkan API
version 1.3, shader cache, and pipeline cache.

### Coordinate frame

The viewer uses an ORL frame abstraction and derives:

- local viewport axes;
- semantic handedness;
- camera position/front/up;
- vkkk coordinate-system labels.

The default world frame is `ORL::frame_dx`. The viewport comments describe
`+X` as right, `+Y` as up, and `+Z` as outward/toward the viewer for that
frame. The code computes semantic handedness from the frame and passes it to
mesh rendering/winding decisions.

### Initial scene state

Startup:

1. creates the window backend and vkkk context;
2. creates camera and empty vkkk scene;
3. creates `ComponentManager`;
4. creates persistent `weights` and `deformer` components;
5. constructs `SceneGraphContext`;
6. registers rig graph definitions;
7. installs an empty graph named `orlrig.scene`;
8. optionally creates Qt node/property panels;
9. creates selection state;
10. constructs the viewport feature tuple;
11. constructs operations and input controls;
12. loads `resource/config/control_map.json`;
13. enters the frame loop.

The persistent weights/deformer creation explains why runtime component IDs
in project files are not required to start at one.

## Viewport feature composition

The main viewport is a vkkk feature container including:

```text
GridFeature
OrthoGridFeature
SceneMeshFeature
JointFeature
ControllerFeature
MeshPickingFeature
MeshCsrFeature
AutoWeightFeature
SolverFeature
JointPickingFeature
DeformerFeature
LocatorFeature
FrameAxisFeature
RuntimeHudFeature
TransformGuideFeature
```

### Frame loop

The active loop is:

```text
window_backend.poll_events()
  -> ControlMap::poll()
  -> optional Qt panel refresh
  -> viewport.begin_frame()
  -> camera/preview/grid updates
  -> viewport.update()
       feature update order
  -> viewport.record_frame()
       draw/picking passes
  -> viewport.end_frame()
```

At shutdown it waits for the vkkk context to become idle, saves the pipeline
cache, detaches the control map, and exits.

`viewport.update()` is where solver/deformer/auto-weight runtime state is
advanced. `record_frame()` is where mesh, joints, controllers, locators,
grid/HUD, and picking features record GPU work.

## Ownership model

### `orlrig::ComponentStore`

The renderer-independent store owns:

- stable process-local `ComponentId`s;
- component names/kinds;
- Joint, Controller, Locator, WeightData, ConstraintData, and DeformerData
  payloads;
- deterministic joint and locator packing order;
- topology revision.

The viewer does not duplicate these payloads as a separate authoritative rig
store.

### `ORL::ComponentManager`

`ComponentManager` wraps `orlrig::ComponentStore` and adds:

- display links to vkkk scene/mesh/line/point resources;
- controller shape selection;
- curve links;
- controller attachment relationships;
- reverse target-to-controller lookup.

It does not own GPU allocations or vkkk rendering objects. A `DisplayLink`
names an external resource.

### `GraphModule`/`NodeRegistry`

The graph module stores node instances, ports, connections, resources, and
metadata. Definitions remain in the registry supplied to
`SceneGraphContext`. Project JSON stores definition IDs, not definitions.

### Scene and GPU ownership

The vkkk scene owns scene objects/meshes. `SceneInputCatalog` creates
ORL-compatible host buffers or aliases existing data. CUDA device allocations
are owned by `OrlExecution`/vkkk integration and may be borrowed across stage
boundaries for a controlled lifetime.

## `ComponentManager`

### Creation and lookup

Creation methods delegate payload creation to the rig store and create
viewer metadata:

```text
create_joint
create_controller
create_locator
create_curve
create_weight
create_constraint
create_deformer
```

Names must remain unique through the underlying store. Lookup is by stable
`ComponentId` or name. Typed accessors return Joint, Controller, Locator,
WeightData, ConstraintData, or DeformerData pointers.

### Packed indexes and stable IDs

Stable `ComponentId` is the authoring/scene identity. Live packed joint and
locator arrays are dense runtime storage. `Joint.parent` is a packed index,
not a `ComponentId`.

This means structural edits can change packed locations. Graph scene binding
therefore resolves stable names/IDs to current packed slots on refresh.

### Recursive joint deletion

`destroy_joint_recursive`:

1. discovers the selected subtree;
2. removes descendants in reverse packed/order-safe order;
3. repairs surviving parent indices;
4. removes affected attachments/relationships through normal destroy paths.

The store's topology revision changes, causing hierarchy and evaluation-plan
invalidation.

### Controller attachments

A controller attachment records:

- target kind: joint or locator;
- target component ID;
- target-local offset matrix;
- whether controller input drives the target.

At attach time:

```text
offset = inverse(target_world) * controller_world
```

Effective controller transform:

```text
controller_world = setup_xform * input_xform
```

When input drives a target:

```text
target_world = controller_world * inverse(offset)
```

The manager enforces one controller per target and repairs reverse maps when a
controller or target is removed.

Setup edits change placement/attachment state. Animation edits update
`input_xform`. The selection layer chooses which mode is active.

## Selection and transform abstraction

### `SelectionRef`

Supported selected kinds:

```text
None
Joint
SceneObject
Vector
Controller
Locator
```

A selection reference contains either a component ID, a scene-object name, or
a vector pointer.

### `XformAttr`

`XformAttr` abstracts a transform destination for viewport operations:

- matrix pointer;
- vector pointer;
- joint translation/rotation/scale arrays;
- local-to-world matrix;
- `read_world` callback;
- `write_world` callback.

This lets move/rotate/scale operations treat scene objects, joints,
controllers, locators, and arbitrary vectors through one interface.

Joint selection uses callbacks that compute a packed world matrix and write a
world transform back into local TRS. Controller selection chooses input/setup
write behavior based on controller input mode.

Selection also updates packed Joint `selected` flags and propagates selected
state to descendants for display.

## `SceneGraphContext`

### Responsibilities

`SceneGraphContext` owns the runtime relationship between:

- one vkkk scene;
- one ComponentManager;
- one legacy graph or staged solver/deformer graphs;
- one node registry;
- SceneInputCatalog;
- hierarchy/evaluation plans;
- graph-to-scene input mappings;
- queued operations;
- change/revision state;
- computed-joints device views.

It is the viewer's orchestration object, not the graph IR itself.

### Legacy and staged graphs

APIs:

```text
set_graph(module, registry)
set_stage_graphs(solver, deformer, registry)
ensure_stage_graphs()
stage_graph(GraphStage)
graph()
```

`set_graph` installs a single graph and clears staged solver state. The
runtime can convert a legacy context into staged storage. `graph()` remains
the deformer/legacy graph when staged graphs exist, while `stage_graph` selects
solver/deformer explicitly.

### Revisions

The context tracks:

```text
graph_revision
graph_edit_revision
evaluation_revision
scene input revision
topology revision
pose revision
change generation
```

`touch_graph` invalidates plans and increments graph/evaluation revisions.
`request_evaluation` requests a full evaluation without changing graph
topology.

### `ChangeSet`

`ChangeSet` contains:

- generation;
- topology/pose revisions;
- full-evaluation flag;
- plan-invalidated flag;
- changed joints;
- changed controllers;
- changed locators.

`update_change_set` compares topology signatures and pose snapshots. Pose
changes remain incremental when topology/input layout is unchanged. Topology
or graph changes invalidate plans and request broader evaluation.

### Plan caching

`hierarchy_signature` hashes packed joint IDs, parent relationships, and
topology inputs. `evaluation_signature` combines graph/stage, component,
hierarchy, and registry-related information.

The context caches:

- optional `HierarchyPlan`;
- packed `HierarchyContext`/data;
- shared `EvaluationPlan`;
- computed-joints device view/count.

Plan compilation errors are retained and not repeatedly printed for the same
revision.

### Input mappings

The context supports explicit mapping:

```text
graph input ID -> scene input ID
graph binding name -> namespaced scene binding
```

This keeps generic graph names such as `joints` separate from scene bindings
such as `scene.rig.joints`.

### Host writeback

`commit_scene_writes` delegates to the scene input catalog and maps packed
runtime values back to stable component IDs. It refuses to commit joint TRS
after a device-only evaluation unless host readback is known to be complete.

### Queued operations and renames

Viewport operations request strings such as `"bind"` through the context.
The graph runtime consumes them during its ordered update phase.

Renaming a joint/locator retargets `find_joint`/`find_locator` node parameter
values in both graph stages so authored graph references continue to resolve
by name.

## `SceneInputCatalog`

### Descriptor model

Each `SceneInputDescriptor` contains:

- stable descriptor ID;
- human label;
- logical interface port;
- runtime parameter kind;
- element stride;
- count binding;
- host/GPU availability;
- canonical handle type.

Sources include:

```text
mesh positions/count
joints/computed joints/joint count
weight buffer/count
inverse binds
locators/count/per-locator transform
controllers/count/per-controller transform
joint handle
locator handle
```

### Namespaced bindings

The catalog produces names such as:

```text
scene.rig.joints
scene.rig.joint_count
scene.rig.locators
scene.rig.locator_count
scene.rig.controllers
scene.rig.controller_count
scene.mesh.<name>.positions
scene.mesh.<name>.vertex_count
scene.rig.weights.<name>.buffer
scene.rig.deformer.<name>.inverse_binds
```

Handle bindings use:

```text
scene.joint.handle.<name>
scene.locator.handle.<name>
```

### Resolve and compatibility

`resolve`/`resolve_binding` maps a graph interface port to:

- host buffer;
- scalar;
- packed view;
- external device pointer;
- exact `HandleValue`.

It validates logical type, domain, shape, semantic, coordinate space, and
runtime kind. This is stricter than some core graph validation, because the
catalog knows actual scene descriptors.

### Refresh

`refresh` rebuilds descriptors and source maps after component/scene edits.
It preserves stable component IDs while repacking current arrays.

Controller transforms are packed deterministically; current code sorts
controller names before packing. Joint/locator arrays follow the store's
packed order.

### CUDA packed arena

When CUDA evaluation is enabled, `prepare_packed_inputs` creates one aligned
host allocation:

```text
SolverContext
aligned joint storage
aligned locator storage
aligned controller matrix storage
```

`SolverContext` stores counts and byte offsets. `PackedBufferView`s let
multiple reflected parameters alias ranges in this allocation.

The active `HandleViewContext` points at the same host arenas. The executor
uploads them to device handle arenas when a CUDA execution requires typed
views.

### Computed joint device view

The catalog can store a borrowed `DeviceBufferView` and element count for
computed joints. A downstream graph input can resolve to that external device
allocation instead of the host `joints_` buffer.

The producer execution must outlive the consumer. Clearing the view occurs
when the solver stage is disabled or a new scene revision invalidates it.

### Host commit

`commit_joints(host_readback_complete)`:

- rejects device-only/stale-host commit;
- copies packed Joint values back into component records by stable ID;
- preserves current stable IDs despite packed index changes.

## `GraphSceneRuntime`

### Role

`GraphSceneRuntime` executes one active graph stage through:

- generated ORL segments for ordinary ORL-function nodes;
- registered runtime adapters for scene/runtime operations;
- `EvaluationPlan` dirty selection;
- CPU or CUDA `OrlGraphExecution`.

It is the main viewer-to-executor bridge.

### Runtime adapters

Current adapter names include:

```text
orlrig.input.find_joint
orlrig.input.find_locator
orlrig.input.find_mesh
orlrig.stage.computed_joints
orlrig.deformer.lbs.capture_bind
orlrig.deformer.lbs.evaluate
```

Adapters are member-function pointers registered in a map. They run outside
generated ORL code and can provide runtime output expressions to graph
lowering.

### Setup and binding

`setup`/`ensure_execution_plan`:

1. selects active graph/stage;
2. validates graph and obtains schedule;
3. ensures hierarchy/evaluation plans;
4. calculates graph fingerprint and scene/graph revision;
5. partitions scheduled nodes;
6. prepares scene graph inputs/runtime expressions;
7. builds `OrlGraphProgram` for each ORL segment;
8. creates backend-specific `OrlGraphExecution`s;
9. binds graph inputs, solver context, hierarchy context/data, and handle
   view context.

The compiled segment cache is invalidated by graph revision, scene input
revision, graph fingerprint, scene revision, and backend selection.

### Segment partitioning

Consecutive lowerable ORL-function nodes are grouped into one generated ORL
segment. Runtime nodes remain separate steps.

Cross-boundary data must be represented through graph inputs/outputs or a
runtime output expression. A data dependency that cannot be reconstructed
inside the destination segment is rejected rather than silently copied.

### Dirty segment skipping

`segment_is_clean` consults the current dirty dispatch and region node IDs.
Stateless segments with no selected affected region can be skipped. Global,
stateful, or unresolved regions force execution.

This is not the same as a complete per-node output cache: the runtime skips
work based on current dirty state, but does not persist all generated outputs
as independent generations.

### Solver stage

If a solver graph has active content, `SolverFeature` delegates to
`GraphSceneRuntime`. The runtime:

1. refreshes inputs;
2. applies current controller inputs in the operational code path;
3. builds dirty dispatch;
4. executes ORL segments/adapters;
5. uses `evaluate_device` for CUDA solver segments;
6. publishes the joint device view as computed-joints output;
7. commits host joints only for host-readback paths.

If the solver graph is empty, `SolverFeature` uses the compatibility path:

1. ensure hierarchy plan;
2. disable CUDA scene packing;
3. apply controller inputs;
4. iterate bound `ik_two_bone` component constraints;
5. use `SolverRunner`.

The fallback rejects controller attachments that would feed a chain joint and
create a cycle.

### Deformer stage

`DeformerFeature` wraps a `GraphSceneRuntime` for `GraphStage::Deformer`.
Only `"lbs"` is currently accepted by `GraphSceneRuntime::set_type`.

On bind:

1. selected mesh and joints are required;
2. weights must be populated;
3. `make_lbs_graph` is installed if no deformer graph exists;
4. bind positions/inverse binds are captured;
5. the deformer is marked bound.

On evaluation:

- CPU calls `LbsRunner` and writes host positions into the vkkk mesh;
- CUDA obtains an output device view and asks the mesh/vkkk path to consume
  the device memory.

The built-in LBS runtime adapter can use a host `components.packed_joints()`
snapshot, so availability of a solver `computed_joints` device view does not
guarantee every LBS path is zero-copy.

### Auto-weight stage

`AutoWeightFeature`:

- requires a selected mesh and at least one selected joint;
- extracts vkkk vertex/index data into `orlrig::MeshData`;
- optionally obtains CSR data from `MeshCsrFeature`;
- runs `AutoWeightRunner`;
- writes vertex-major `WeightData`;
- prints diagnostics/statistics and writes `orl_debug_weights.txt`.

Current UI algorithms:

```text
closest_distance
closest_hierarchy
heat
geodesic
```

Auto-weight execution uses `OrlEvaluationPause` so graph evaluation does not
race authoring/data preparation.

## Rendering and GPU features

### Scene mesh

`SceneMeshFeature` renders vkkk meshes in Phong or x-ray modes and can render
wireframe/edge data. X-ray shaders use alpha/rim behavior. Mesh frame
handedness controls winding decisions.

### Joint rendering

Joint points and parent-child lines use the same 128-byte Joint layout as
`orlrig`. If a computed joint CUDA device view is published, `JointFeature`
can import that allocation for Vulkan rendering; otherwise it packs host
components.

### Controllers and locators

Controllers render curve/polygon shapes using effective world transforms.
Locators render instanced transform/line geometry and use selection state for
coloring.

### Picking

Joint picking renders instanced points into a GPU A-buffer and stores packed
joint indices. Selection resolves those packed indices back to stable
component IDs.

Mesh picking renders integer scene-object IDs to an offscreen target. The
operation layer resolves those IDs to scene object names.

### Mesh CSR

`MeshCsrFeature` runs compute-shader passes over mesh data:

```text
clear
count
scan
scatter
unique
compact
```

The resulting offsets/neighbors are consumed by surface-based auto-weight
algorithms.

The shaders are viewer preprocessing/render resources, not ORL compiler
intrinsics.

## Model loading and scene assets

`LoadModelOp` opens a platform-specific file dialog:

- Qt file dialog in Qt mode;
- Windows common dialog on Windows GLFW mode;
- no file dialog implementation on non-Windows non-Qt builds.

It calls vkkk's `DrawableMgr::load_file`, requests vertex/normal components,
transforms loaded meshes from the file frame to the viewer world frame,
corrects normals and triangle winding if handedness changes, synchronizes GPU
resources, and creates scene objects with unique names.

This replaces the old `src/orltest`/hand-written OBJ loader. The old loader is
not part of current viewer behavior.

## Project serialization

### Top-level document

The viewer writes UTF-8 JSON:

```text
header
evaluation
graph
components
editor (optional)
```

Constants:

```text
project magic: ORL_PROJECT
project version: 1
embedded graph magic: ORL_GRAPH_STAGES
graph format version: 1
language: orl-0
logical ABI: orlgraph-1
```

### Definition/asset ownership

Project files do not store:

- graph node definitions;
- the application registry;
- vkkk meshes or scene objects;
- Vulkan resources.

On load, startup code must register the same definition IDs and externally
load/associate mesh assets.

### Integer namespaces

The project uses distinct IDs:

| Value | Meaning |
| --- | --- |
| file component ID | serialized joint/locator/controller/constraint identity |
| packed joint index | runtime parent/weight index |
| runtime component ID | new process-local ID assigned on load |
| graph StableId | graph node/interface/resource identity |
| handle slot | dispatch-local packed slot |

Joint parents are saved as component IDs and converted back to packed indices
on load. Weight `joint` values remain packed indices. File IDs are remapped to
new runtime IDs.

### Components

Serialized components include:

- joints with local TRS/parent/selection;
- locators with matrix;
- controllers with setup/input transforms and shape;
- constraints and references;
- controller attachments;
- optional weights;
- optional LBS deformer buffers/inverse binds.

Matrices are written as 16-number column-major GLM arrays. ORL runtime
matrices are row-major, so packing/handle callbacks perform explicit
transposition.

### Staged graph

`project.graph` embeds:

```text
stages.solver
stages.deformer
```

The staged serializer includes canonical hash/version metadata. Project load
validates both graphs against the current registry, while incomplete
authoring connections can be treated as warnings.

### Load sequence

The implementation validates:

1. project header/evaluation fields;
2. staged graph document and hash/version;
3. graph definitions against the startup registry;
4. component IDs, names, parent references, constraints, and attachments.

It then preserves persistent weights/deformer components, recreates authoring
components, restores relationships and buffers, and installs the staged
graphs.

### Editor layout

The optional `editor.node_graph` stores separate solver/deformer:

- pan;
- zoom;
- node positions;
- frames;
- collapsed state and members.

This is viewer UI state, not graph execution data.

## Controls and viewport operations

### `ControlMap`

`ControlMap` maps:

- keys;
- mouse buttons;
- mouse drags;
- scroll;

to named operations. Each binding can be window-scoped or panel-scoped and
can invoke one operation or an ordered list.

It supports:

- predicate overloads;
- immediate operations;
- modal operations;
- edit operations that suspend ORL evaluation;
- active panel providers;
- GLFW callbacks or Qt state polling.

`resource/config/control_map.json` supplies the default mappings.

### Evaluation pause policy

Structural operations are registered through `bind_edit_op`. The operation
scope handler pauses ORL evaluation and disables CUDA scene packing while
scene/graph structure is being changed. When the scope ends it requests a
fresh evaluation.

Continuous move/rotate/scale operations remain live so pose/controller input
changes can drive solver evaluation during a drag.

### `Selection`

Selection supports scene objects, joints, controllers, locators, and vectors.
It provides:

- selected mesh name/model;
- selection flags;
- `XformAttr` destinations;
- controller input/setup mode;
- world/local transform callbacks.

### Operations

Representative operations:

- model load;
- clear scene;
- create/extend joint;
- create locator/controller/IK;
- delete and recursive joint deletion;
- mirror;
- move/rotate/scale;
- select/pick;
- controller attachment toggle;
- controller curve cycle;
- display mode/camera switch;
- graph frames/copy/paste in Qt mode.

Operations modify ComponentManager, SceneGraphContext, graph modules, or
vkkk scene objects according to their scope.

## Qt graph/property editor

When Qt6 is selected:

### Node graph editor

`NodeGraphEditor`:

- switches solver/deformer stages;
- displays graph inputs/outputs/nodes/ports;
- resolves scene-input controls;
- creates/removes nodes and connections;
- validates socket compatibility;
- supports frames, collapse, copy/paste;
- imports/exports layout;
- opens/saves project files;
- notifies `SceneGraphContext` of edits.

Graph layout is maintained in viewer-only structures and serialized under the
optional editor section.

### Property editor

`PropertyEditor` edits:

- selected component names;
- joint/controller/locator transforms;
- controller shape;
- constraint fields;
- graph node parameters;
- stage/runtime properties.

Renaming scene elements triggers `SceneGraphContext::retarget_element_name`
so name-based `find_joint`/`find_locator` graph nodes continue to reference
the same logical element.

## CPU/CUDA and scene handoff

### CPU path

```text
scene edits
  -> refresh catalog
  -> host graph input bindings
  -> OrlGraphExecution::evaluate/evaluate_result
  -> host joint/mesh writeback
  -> vkkk scene update
```

### CUDA solver path

```text
scene edits
  -> packed solver arena + HandleViewContext
  -> OrlGraphExecution::evaluate_device
  -> device Joint handle arena
  -> computed_joints DeviceBufferView
  -> downstream graph/runtime consumer
```

No host readback is required for the device-only path. The viewer refuses to
commit stale host Joint values after such an evaluation.

### CUDA deformer path

```text
bind positions/weights/inverse binds
  -> Orl/CUDA LBS execution
  -> output DeviceBufferView
  -> vkkk/Vulkan mesh device update
```

The exact zero-copy route depends on whether the deformer is executed as a
graph segment consuming `computed_joints` or through the built-in LBS runner,
which may repack host component joints.

## Current discrepancies and limitations

### Monolithic target

There is no reusable viewer library. This makes unit reuse and isolated viewer
testing harder; scene tests compile selected implementation files directly.

### Staged deformer support

`GraphSceneRuntime::known_type` currently accepts only `"lbs"`.

### Empty versus populated solver graph

An empty solver graph uses the legacy component-constraint fallback. A
populated solver graph is authoritative and uses graph runtime execution
instead of the fallback constraint loop.

### Controller-input documentation drift

`project_file_details.md` states that populated solver graphs skip
`apply_controller_inputs`. Current `GraphSceneRuntime` code applies controller
inputs before dispatch. The source behavior is the current operational
authority; the documentation should be reconciled later.

### Device handoff is not universal

The solver publishes a computed-joints device view, but the built-in LBS
adapter can call `LbsRunner` with the host component snapshot. A graph wired
through `orlrig.stage.computed_joints` can use the device view; every deformer
path does not automatically do so.

### Registry coverage

Public stdlib node registration deliberately skips:

```text
hd_id
spline_ik
full_body_ik
```

`node_refs.md` is stale where it describes those as current registered nodes.

### Runtime ABI limits

- ORL runtime entries return exact `int`;
- custom struct buffer strides are not generic;
- graph outputs cannot allocate arbitrary output values;
- universal direct runtime handle parameters are rejected;
- CUDA result width is 32-bit.

### Plan/dispatch boundary

Evaluation plans and dirty dispatch exist. CUDA sparse dispatch falls back to
full evaluation when selected regions do not support sparse operation. Viewer
code packs a `SolverDispatchContext`, but it is not yet a general implicit
ORL parameter path.

### Project portability

Projects depend on:

- matching startup registry IDs;
- external meshes/vkkk scene assets;
- matching element names for name-based scene lookup.

They are not self-contained graph/runtime packages.

### Test boundary

Scene tests cover headless graph/component/ABI behavior. They do not fully
prove:

- a window-backed Vulkan frame;
- CUDA/Vulkan interop;
- visible GPU picking;
- shader/pipeline behavior in a real viewer session;
- device mesh deformation displayed on screen.

### File-dialog portability

The model loader uses Qt on Qt builds and Windows common dialogs on Windows
GLFW builds. On non-Windows, non-Qt builds it reports that the file dialog is
not implemented.

## Tests

### Scene graph tests

`tests/scene_graph_tests/scene_graph_tests.cpp` covers:

- active LBS graph and staged context;
- typed solver graph execution;
- control-map overloads/scopes/modal behavior;
- controller attachments;
- joint creation/deletion/mirroring;
- host joint writeback;
- computed-joints device binding;
- shared CUDA arena;
- hierarchy packing;
- pose/topology change sets;
- stable handles after packed-index changes.

### Related runtime tests

- `tests/rig_graph_tests`: registry/resources/typed node contracts;
- `tests/orlrig_tests`: ABI, hierarchy, runners, evaluation plans;
- `tests/graph_lowering_tests`: graph source/runtime/CPU-CUDA behavior;
- `tests/exec_tests`: executor/cache/binding/handle behavior;
- `tests/vkkk_tests`: shader cache and vkkk integration.

The current test CMake primarily creates executables and does not visibly
register them with `add_test()` at the root.

## Recommended reading order

1. `main.cpp` for startup, feature composition, and frame loop.
2. `component_manager.hpp/.cpp` for viewer ownership and attachments.
3. `selection.hpp` and `control_map.hpp/.cpp` for user interaction/state
   mutation.
4. `scene_graph_context.hpp/.cpp` for graph/scene revisions and plans.
5. `graph_scene_inputs.hpp/.cpp` for scene bindings and CUDA packing.
6. `graph_scene_runtime.hpp/.cpp` for execution segmentation and adapters.
7. `vp/solver_feature.cpp` and `vp/deformer_feature.cpp` for stage entry.
8. `project_serialization.cpp` and `project_file_details.md` for persistence.
9. `vp/*_feature.*` and `resource/shaders` for rendering/picking/CSR.
10. `tests/scene_graph_tests` while tracing a complete edit-to-frame path.
11. `docs/orlexec.md`, `docs/orlgraph.md`, and `docs/orlcomp.md` for the
    downstream compiler/graph/runtime contracts.
