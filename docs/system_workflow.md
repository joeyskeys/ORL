# ORL system workflow: authored rig/graph to runnable machine code

## Purpose

This document traces the operational path from:

```text
user creates a rig and graph in the viewer
  -> graph is validated and split into executable work
  -> graph nodes become generated ORL source
  -> ORL becomes LLVM IR
  -> LLVM becomes a CPU JIT object or CUDA PTX/cubin
  -> the runtime binds current scene data
  -> solver/deformer work runs every frame
  -> joints and mesh output return to the viewer
```

It is a code-reading map rather than a design proposal. The source paths and
function names below describe the current implementation at commit `4e0fb3d`.

The most important distinction is:

```text
Graph IR is a logical dataflow description.
Generated ORL is the executable source representation.
LLVM IR is the backend intermediate representation.
JIT object/PTX/cubin is the machine-runnable artifact.
```

The graph is not directly turned into LLVM instructions. The current path
lowers the graph into ORL source first, then reuses the ordinary ORL parser,
semantic analysis, runtime reflection, LLVM codegen, optimizer, and backend.

## One-page pipeline

```text
Viewer startup
  main()
    -> register_rig_node_definitions()
    -> SceneGraphContext::set_graph()
    -> viewport features and ControlMap

User setup
  LoadModelOp::on_eval()
    -> vkkk DrawableMgr::load_file()
    -> scene object
  ComponentManager::create_joint/controller/locator()
  AutoWeightFeature::run()
    -> AutoWeightRunner::run()
    -> ORL auto-weight program
  graph editor
    -> GraphModule nodes/connections
    -> SceneGraphContext::touch_graph()

Frame update
  main() loop
    -> window_backend.poll_events()
    -> ControlMap::poll()
    -> viewport.update()
       -> SolverFeature::on_update()
       -> DeformerFeature::on_update()

Populated graph stage
  GraphSceneRuntime::on_update()
    -> ensure_hierarchy_plan()
    -> ensure_evaluation_plan()
    -> build_dirty_dispatch_plan()
    -> GraphSceneRuntime::dispatch_graph()
       -> orlgraph::validate()
       -> topological_schedule()
       -> ensure_execution_plan()
          -> build_orl_segment()
             -> OrlGraphProgram::Compile()
                -> OrlGraphLowerer::lower()
                -> OrlProgram::Compile()
                   -> Parser / SemanticAnalyzer / reflection
             -> OrlGraphExecution::Create()
                -> OrlExecution::Create()
                   -> cache or LLVM codegen
                   -> O2 optimization
                   -> CPU LLJIT or CUDA module
       -> bind scene buffers/contexts/handles
       -> evaluate_result() or evaluate_device()

Machine execution
  CPU:
    __orl_host_entry_<entry>()
      -> native ORL function in LLJIT
  CUDA:
    orl_cuda_entry<<<blocks, threads>>>()
      -> generated device function
      -> orl_cuda_result

Output
  solver:
    host Joint writeback or computed-joints device view
  deformer:
    host mesh update or CUDA/Vulkan mesh update
```

## The objects and identity layers

Before following the calls, keep these identities separate:

| Value | Created/owned by | Meaning in the workflow |
| --- | --- | --- |
| `orlgraph::StableId` | graph IR | serialized node/input/output/resource identity |
| `ComponentId` | `orlrig::ComponentStore` | stable process-local rig component identity |
| packed joint index | `ComponentStore`/runtime buffers | dense slot used by `Joint.parent` and weights |
| `HandleValue{type_id, slot}` | compiler/runtime binding | dispatch-local typed handle token |
| ORL parameter name | generated source/runtime reflection | machine-call ABI slot |
| `DeviceBufferView` | executor/GPU runtime | borrowed device address and byte range |

A project file stores component IDs and graph stable IDs. It does not store
current packed slots or runtime handle slots. The viewer resolves those at
bind time after the current scene has been packed.

## Phase 0: application startup and registry construction

### Entry point

File: `src/orlviewer/main.cpp`

Key function: `main(int argc, char** argv)`

Startup sequence:

1. `parse_startup_options` reads shader/pipeline recompile flags.
2. A `vkkk::QtBackend` or `vkkk::GlfwBackend` is created.
3. A `vkkk::Context` is initialized with shader/pipeline caches.
4. A camera and `vkkk::Scene` are created.
5. `ComponentManager` is created.
6. Persistent `weights` and `deformer` components are created.
7. `SceneGraphContext` is constructed with the scene and component manager.
8. `orlrig::register_rig_node_definitions` fills a `NodeRegistry`.
9. An empty graph is installed through `SceneGraphContext::set_graph`.
10. Viewer/viewport features and operations are created.
11. `ControlMap` loads `resource/config/control_map.json`.
12. The frame loop starts.

Important calls:

| Call | File | Role |
| --- | --- | --- |
| `parse_startup_options` | `src/orlviewer/main.cpp` | Viewer startup flags |
| `orlrig::register_rig_node_definitions` | `src/orlexec/orlrig/graph_resources.cpp` | Registers scene/runtime/stdlib node definitions |
| `SceneGraphContext::set_graph` | `src/orlviewer/scene_graph_context.cpp` | Installs graph and registry |
| `Viewport::add_feature` calls | `src/orlviewer/main.cpp` | Installs solver/deformer/render/picking features |
| `ControlMap::load_config` | `src/orlviewer/control_map.cpp` | Loads user operations |

### What node registration actually does

`register_rig_node_definitions` in
`src/orlexec/orlrig/graph_resources.cpp`:

1. calls `register_rig_handle_views`;
2. registers the computed-joints runtime definition;
3. registers LBS capture/deform runtime definitions;
4. registers typed `find_joint`, `find_locator`, and `find_mesh`;
5. scans the solver, constraint, and auto-weight stdlib directories;
6. calls `compile_node_definitions_to_oro_file` for selected exported ORL
   functions;
7. registers the resulting definitions from `.oro`.

The stdlib registration path uses:

```text
register_compiled_stdlib_nodes
  -> NodeImportOptions
  -> compile_node_definitions_to_oro_file
  -> register_orl_node_definitions_from_oro
```

The registry contains definitions. It does not execute anything and does not
pack scene data.

## Phase 1: user creates or loads the rig

### Loading a mesh

File: `src/orlviewer/ops/load_model_op.cpp`

Key function: `LoadModelOp::on_eval`

The operation:

1. opens a platform-specific file dialog;
2. calls `scene.drawable_mgr->load_file`;
3. requests vkkk vertex/normal components;
4. transforms the mesh from file frame to viewer frame;
5. fixes normals and triangle winding when handedness changes;
6. waits for the vkkk context;
7. synchronizes the mesh to GPU;
8. creates a unique `vkkk::Scene` object name.

The old `src/orltest` OBJ parser is not involved. The current production
loader is the vkkk path.

### Creating components

User operations call `ComponentManager`:

| User action | Entry/source |
| --- | --- |
| create joint | `CreateJointOp::on_eval`, `src/orlviewer/ops/create_joint_op.cpp` |
| extend chain | `ExtendJointChainOp`, same operation family |
| create locator | `CreateLocatorOp`, `src/orlviewer/ops/create_locator_op.hpp` |
| create controller | `CreateControllerOp`, `src/orlviewer/ops/create_controller_op.hpp` |
| create IK constraint | `CreateIkOp`, `src/orlviewer/ops/create_ik_op.hpp` |
| attach controller | `ToggleControllerAttachmentOp`, `src/orlviewer/ops/toggle_controller_attachment_op.hpp` |
| delete/mirror | `DeleteOp` / `MirrorOp`, `src/orlviewer/ops` |

`ComponentManager::create_*` delegates payload allocation to
`orlrig::ComponentStore` and records viewer metadata such as display links,
controller shapes, curves, or attachments.

For joints:

```text
ComponentId
  -> ComponentStore record
  -> packed joint order
  -> Joint.parent packed index
  -> later hierarchy plan and ORL Joint buffer
```

For controllers:

```text
setup xform + input xform
  -> effective controller world transform
  -> optional attachment target update
  -> packed controller matrix for ORL scene inputs
```

For graph `find_joint`/`find_locator`, the authored value is a name parameter
and a typed handle output. The packed slot is resolved later.

### Controller attachments

`ComponentManager::attach_controller` computes the target-local offset:

```text
attachment.xform =
    inverse(target_world) * controller_world
```

During animation input application:

```text
target_world =
    controller_world * inverse(attachment.xform)
```

Important functions:

| Function | File |
| --- | --- |
| `ComponentManager::attach_controller` | `src/orlviewer/component_manager.cpp` |
| `ComponentManager::controller_world_xform` | `src/orlviewer/component_manager.cpp` |
| `ComponentManager::apply_controller_inputs` | `src/orlviewer/component_manager.cpp` |
| `ComponentManager::validate_controller_attachments` | `src/orlviewer/component_manager.cpp` |

Structural operations are registered as edit operations. The operation scope
handler in `main.cpp` temporarily disables ORL evaluation and CUDA scene
packing while graph/scene structure is being changed.

## Phase 2: user creates and edits the graph

### Graph editor entry points

When Qt6 is enabled, `NodeGraphEditor` is the user-facing graph editor:

File: `src/orlviewer/qt/node_graph_editor.cpp`

Important calls:

| Function | Role |
| --- | --- |
| `NodeGraphEditor::set_stage` | Switch solver/deformer graph |
| `NodeGraphEditor::create_node` | Adds a `NodeInstance` from the registry |
| `NodeGraphEditor::create_graph_input` | Adds a graph interface input |
| `NodeGraphEditor::add_connection` | Adds a typed graph connection |
| `NodeGraphEditor::notify_graph_changed` | Calls `SceneGraphContext::touch_graph` |
| `NodeGraphEditor::save_project_file` | Calls `save_project_json` |
| `NodeGraphEditor::load_project_file` | Calls `load_project_json` |
| `NodeGraphEditor::refresh_scene_inputs` | Rebuilds find-node controls |

`node_graph/node_ops.cpp` contains reusable creation/deletion/frame helpers.

After a graph edit:

```text
NodeGraphEditor::notify_graph_changed()
  -> SceneGraphContext::touch_graph()
     -> graph_revision++
     -> graph_edit_revision++
     -> evaluation_revision++
     -> mark_plan_invalidated()
        -> discard cached evaluation plan
        -> pending full evaluation
```

### What is authored

The graph stores:

- node instance IDs;
- registry definition IDs;
- compile-time parameter values;
- graph inputs/outputs;
- connections;
- resource declarations;
- conversions/adapters;
- solver/deformer stage membership.

The graph does not store:

- LLVM IR;
- CPU function pointers;
- CUDA pointers;
- packed joint indices;
- runtime handle slots;
- vkkk scene objects;
- node definitions in project JSON.

### Save path

File: `src/orlviewer/project_serialization.cpp`

Key function: `save_project_json`

The save path:

1. serializes solver/deformer graph modules with
   `orlgraph::serialize_graph_stages_json`;
2. serializes components using file-local component IDs;
3. converts runtime joint parent packed indices back to component IDs;
4. serializes weights/deformer buffers;
5. serializes optional editor layout;
6. writes project header/evaluation settings.

The output embeds `ORL_GRAPH_STAGES`, but definitions and meshes remain
external.

### Load path

Key function: `load_project_json`

The load path:

1. parses project JSON;
2. calls `deserialize_graph_stages_json`;
3. validates both graph stages against the startup `NodeRegistry`;
4. validates component IDs/names/references;
5. remaps file component IDs to new runtime IDs;
6. rebuilds packed parents/attachments/constraints;
7. restores weights/deformer data;
8. installs graphs with `SceneGraphContext::set_stage_graphs`;
9. restores editor layout.

Name-based `find_joint`/`find_locator` parameters are expected to match
current component names. `SceneGraphContext::retarget_element_name` updates
graph parameter values after an in-editor rename.

## Phase 3: frame loop and stage dispatch

### Main loop

File: `src/orlviewer/main.cpp`

The central loop is:

```cpp
while (!window_backend.should_close()) {
    window_backend.poll_events();
    controls.poll();
    viewport.begin_frame();
    viewport.update();
    viewport.record_frame();
    viewport.end_frame();
}
```

The important calls are:

```text
ControlMap::poll
  -> operation handlers / modal operations

Viewport::update
  -> SolverFeature::on_update
  -> DeformerFeature::on_update
  -> other viewport feature updates
```

`RuntimeConfig` selects:

```text
ComputeDevice::Cpu -> exec::Backend::Cpu
ComputeDevice::Gpu -> exec::Backend::Cuda
```

`runtime_config.evaluate_orl` can disable execution temporarily or
permanently while authoring.

## Phase 4: solver stage branch

`SolverFeature::on_update` in
`src/orlviewer/vp/solver_feature.cpp` has two materially different paths.

### Path A: empty solver graph compatibility mode

Condition:

```text
GraphSceneRuntime::has_active_graph_content() == false
```

Call sequence:

```text
SolverFeature::on_update
  -> SceneGraphContext::ensure_hierarchy_plan
     -> compile_hierarchy_plan
     -> pack_hierarchy_plan
  -> SolverRunner::set_hierarchy_plan
  -> clear computed-joints device view
  -> SceneInputCatalog::set_cuda_evaluation(false)
  -> ComponentManager::apply_controller_inputs
  -> iterate ComponentKind::Constraint
  -> SolverFeature::evaluate_two_bone
     -> SolverRunner::evaluate_two_bone
```

`SolverFeature::evaluate_two_bone`:

1. checks root/mid/end components;
2. rejects controller attachments that would feed chain joints;
3. resolves target/pole as locators or controller transforms;
4. calls `SolverRunner`.

`SolverRunner::ensure_program` compiles:

```text
use solver/ik_two_bone;
```

through:

```text
OrlProgram::Compile
  -> OrlExecution::Create
```

It then binds packed joints, locator transforms, solver context, hierarchy
data, and typed handle view context before evaluation.

This path is not graph-authored execution. It is retained compatibility for
scenes without an authored solver graph.

### Path B: populated solver graph

Condition:

```text
GraphSceneRuntime::has_active_graph_content() == true
```

Call sequence:

```text
SolverFeature::on_update
  -> GraphSceneRuntime::on_update
  -> ensure_hierarchy_plan
  -> ComponentManager::apply_controller_inputs
  -> GraphSceneRuntime::dispatch_graph(capture=false)
```

`GraphSceneRuntime::on_update` also:

- watches `graph_edit_revision` and `evaluation_revision`;
- invalidates compiled segments after graph/scene changes;
- returns early when the graph is empty;
- retains a previous computed-joints device view when a partial frame has no
  dirty solver region.

## Phase 5: hierarchy and dirty evaluation planning

### Hierarchy plan

File: `src/orlviewer/scene_graph_context.cpp`

Call sequence:

```text
SceneGraphContext::ensure_hierarchy_plan
  -> hierarchy_signature
  -> orlrig::compile_hierarchy_plan
  -> orlrig::pack_hierarchy_plan
  -> fill hierarchy_data OrlBuffer
  -> store HierarchyContext
```

`compile_hierarchy_plan` in
`src/orlexec/orlrig/hierarchy.cpp` validates:

- parent bounds;
- self-parenting;
- cycles;
- packed ID/data consistency.

It computes preorder, depth, subtree ranges, level buckets, and ancestor
storage.

`pack_hierarchy_plan` emits:

```text
HierarchyContext
int64 hierarchy_data[]
```

The offsets in `HierarchyContext` index the integer data array.

### Evaluation plan

File: `src/orlviewer/scene_graph_context.cpp`

Call sequence:

```text
SceneGraphContext::ensure_evaluation_plan(stage)
  -> ensure_hierarchy_plan
  -> evaluation_signature(stage)
  -> orlrig::compile_evaluation_plan
       -> orlgraph::validate
       -> orlgraph::topological_schedule
       -> resolve typed handle effects
       -> expand hierarchy propagation
       -> build region dependencies/batches
  -> cache shared EvaluationPlan
```

`compile_evaluation_plan` in
`src/orlexec/orlrig/evaluation.cpp` creates `SolverRegion`s for solver,
constraint, partial-footprint, and non-identity ORL nodes.

It resolves:

- graph connections;
- constant `find_joint`/`find_locator` names;
- typed handle read/write effects;
- resource effects;
- hierarchy ancestors/descendants;
- writer/reader dependencies.

Missing/opaque/stateful/full metadata is conservative and can force global
evaluation.

### Dirty dispatch

Inside `GraphSceneRuntime::dispatch_graph`:

```text
SceneGraphContext::take_change_set
  -> DirtyInputs
  -> build_cuda_dispatch_plan (GPU)
     or build_dynamic_dispatch_plan (CPU)
  -> pack_dynamic_dispatch_plan
```

`DirtyInputs` contains:

- full-evaluation/plan-invalidated flags;
- dirty joints;
- dirty controllers;
- dirty locators.

`build_dynamic_dispatch_plan` selects affected regions and dependency closure.
It falls back to full evaluation above the density threshold. CUDA sparse
dispatch also falls back to full evaluation when selected regions lack sparse
support.

The packed `SolverDispatchContext` is computed, but it is not currently a
general implicit ORL kernel parameter. The viewer primarily uses dispatch
selection to skip clean segments.

## Phase 6: graph validation and execution-plan construction

### Validation

`GraphSceneRuntime::dispatch_graph` calls:

```text
SceneGraphContext::validate(stage)
  -> orlgraph::validate
```

`orlgraph::validate` checks:

- node definitions;
- stage availability;
- port/parameter contracts;
- required connections;
- type/cardinality/handle compatibility;
- semantics/shapes/domains;
- conversions/output adapters;
- feedback legality;
- cycle/schedule result.

It returns `ValidationResult` with diagnostics and a schedule.

### Segment cache key

`GraphSceneRuntime::ensure_execution_plan` reuses the execution plan when:

```text
graph_revision
scene_input_revision
graph_fingerprint
backend
```

still match. Otherwise it clears `execution_plan_` and `orl_segments_`.

### Segment partitioning

The graph schedule is traversed in `ensure_execution_plan`:

```text
ImplementationKind::OrlFunction
  -> collect into an ORL segment

ImplementationKind::Runtime
  -> runtime adapter execution step
  -> scene input/computed-joint nodes may be included in ORL segment
     when they have runtime output expressions

identity operation
  -> can remain in an ORL segment
```

Runtime operations without registered adapters fail. Unsupported
implementations fail if their outputs are used.

Solver region batches can cause ORL segments to be stashed/reordered so
region execution follows evaluation batches.

## Phase 7: converting one graph segment into ORL source

### `GraphSceneRuntime::build_orl_segment`

File: `src/orlviewer/graph_scene_runtime.cpp`

Call sequence:

```text
GraphSceneRuntime::build_orl_segment(node_ids, segment_index)
  -> copy source graph resources
  -> copy segment nodes
  -> prepare_runtime_output_expressions
  -> copy internal/graph-output connections
  -> create GraphLoweringOptions
  -> set entry_function = orl_graph_segment_<N>
  -> set scene_revision
  -> set runtime_output_expression callback
  -> OrlGraphProgram::Compile
  -> OrlGraphExecution::Create
  -> store OrlSegment{graph, program, execution, region_nodes}
```

`prepare_runtime_output_expressions` asks the scene/runtime adapter boundary
how an external runtime node's output should appear as an ORL expression.

Examples:

```text
find_joint.handle
find_locator.handle
computed_joints
scene mesh/weights/locator/controller inputs
```

are supplied from current `SceneInputCatalog`/runtime expressions rather than
invented as unresolved external function calls.

## Phase 8: graph lowering into generated ORL

### `OrlGraphProgram::Compile`

File: `src/orlexec/orl_graph_exec.cpp`

Call sequence:

```text
OrlGraphProgram::Compile(module, registry, options)
  -> OrlGraphLowerer::lower
  -> retain generated source/entry/outputs/scene revision
  -> build local-to-canonical handle identity map
  -> OrlProgram::Compile(generated source)
```

If lowering or normal ORL compilation fails, the graph program is invalid and
its diagnostics are retained.

### `OrlGraphLowerer::lower`

File: `src/orlcomp/orl_graph_lowering.cpp`

Detailed sequence:

1. Check generated entry name.
2. Call `orlgraph::validate`.
3. Collect `use` modules from:
   - graph input types;
   - ORL node implementations;
   - conversion functions;
   - writeback functions.
4. Collect all handle types from graph interfaces and node definitions.
5. Generate safe private local handle names.
6. Record local name → canonical name identities.
7. Emit `use module;` lines.
8. Emit private `handle ...;` declarations.
9. Emit generated entry parameters for graph inputs.
10. Specialize generic/union handle inputs from exact incoming edges.
11. Build endpoint expression lookup.
12. Walk `validation.schedule.order`.
13. Lower identity/runtime nodes or reject unsupported runtime nodes.
14. Emit conversions.
15. Emit ORL function calls in schedule order.
16. Record first output expressions for node calls.
17. Emit output adapters and writebacks.
18. Emit graph outputs.
19. Return one scalar integer output directly when permitted.
20. Return `0` if there is no integer output.

Conceptual generated source:

```orl
use rig/handles;
use solver/ik_two_bone;

handle __orl_handle_orlrig_joint_handle;
handle __orl_handle_orlrig_locator_handle;

int orl_graph_segment_0(
    __orl_handle_orlrig_joint_handle root,
    __orl_handle_orlrig_locator_handle target) {
    int node_ik_status = solver_ik_two_bone(...);
    return node_ik_status;
}
```

The exact private names/parameters depend on the graph.

### Important lowering constraints

- ordinary ORL nodes become direct function calls;
- only the first definition output is assigned as the ordinary node result;
- runtime nodes require callback-provided expressions;
- generic handles are compile-time specialized, not runtime-dispatched;
- writable adapters retain temporary/selector/writeback state;
- buffer/float/handle graph outputs must alias graph-input parameters;
- only one computed scalar integer can be returned.

## Phase 9: generated ORL parsing and semantic analysis

`OrlProgram::Compile` receives the generated source and executes the normal
compiler frontend:

```text
OrlProgram::Compile
  -> register_rig_handle_views
  -> orlcomp::Parser::Parse
     -> Preprocessor::Process
        -> expand use modules
     -> Lexer::NextToken
     -> Parser AST
  -> SemanticAnalyzer::analyze
  -> HandleTypeRegistry collision checks
  -> DescribeRuntimeFunction
  -> ParameterDesc reflection
```

Files:

| Stage | File/function |
| --- | --- |
| parser/preprocessor | `src/orlcomp/orl_parser.cpp`, `orl_preprocessor.cpp` |
| semantic analysis | `src/orlcomp/orl_analysis.cpp::SemanticAnalyzer::analyze` |
| handle reflection | `src/orlcomp/orl_runtime_signature.h::DescribeRuntimeFunction` |
| public compile object | `src/orlexec/orl_exec.cpp::OrlProgram::Compile` |

The generated entry must return exact ORL `int`. Its explicit parameters are
reflected as buffers, `int`, `float`, or exact handles. Implicit
solver/hierarchy contexts are added when the expanded program references
those names.

## Phase 10: creating the executable backend

### `OrlGraphExecution::Create`

File: `src/orlexec/orl_graph_exec.cpp`

```text
OrlGraphExecution::Create(program, backend)
  -> OrlExecution::Create(*program.program_, backend, cache)
```

### `OrlExecution::Create`

File: `src/orlexec/orl_exec.cpp`

The exact implementation is backend-neutral up to code generation:

```text
OrlExecution::Create
  -> retain OrlProgram implementation
  -> build execution_cache_material
  -> make_binary_cache_key
  -> create implicit contexts
  -> try CPU object / CUDA cubin/PTX cache
  -> on cache miss:
       parse source again
       LlvmIrCodegen::Generate
       LlvmOptimizer::Optimize(O2)
       create OrlJitEngine or OrlGpuEngine
       load module
       save backend artifact
```

The cache key includes top-level source, target/LLVM/ABI data, include paths,
and handle-view registry fingerprint. It does not include the contents of
transitive `use` files, so stdlib changes require cache invalidation care.

## Phase 11A: CPU machine-runnable path

### LLVM code generation

`OrlExecution::Create` creates:

```cpp
orlcomp::LlvmIrCodegen codegen(
    program->options.source_name,
    orlcomp::OrlCodegenTarget::Host);
```

Then:

```text
LlvmIrCodegen::Generate
  -> create LLVM context/module
  -> create hidden context/handle types
  -> predeclare structs
  -> define structs
  -> predeclare functions
  -> generate function bodies
  -> generate __orl_host_entry_<entry> wrappers
```

File: `src/orlcomp/orl_codegen.cpp`

The host wrapper has the dynamic shape:

```text
int64_t __orl_host_entry_<entry>(
    void* const* buffers,
    const int64_t* integers,
    const double* floats,
    optional handle lanes,
    optional solver context,
    optional hierarchy context/data)
```

### LLVM optimization

`OrlJitEngine::LoadModuleWithOptimization` calls:

```text
LlvmOptimizer::Optimize(module, O2)
  -> LLVM default per-module O2 pipeline
  -> LLVM module verification
```

File: `src/orlcomp/orl_optimizer.cpp`

### LLJIT loading

`OrlJitEngine::LoadModule`:

1. rejects CUDA/ROCm target selections;
2. creates native LLJIT;
3. installs optional CPU object cache;
4. registers `__orl_parallel_for`;
5. registers runtime symbols supplied by `OrlExecution`;
6. adds the optimized module.

Files:

```text
src/orlcomp/orl_jit.cpp
src/orlcomp/orl_parallel_runtime.cpp
src/orlexec/orl_exec.cpp::register_handle_view_symbols
```

The machine-runnable CPU symbol is looked up by
`OrlJitEngine::InvokeInt64WithRuntimeArgs*`.

## Phase 11B: CUDA machine-runnable path

### CUDA-target LLVM generation

For CUDA, `OrlExecution::Create` uses:

```text
LlvmIrCodegen(..., OrlCodegenTarget::Cuda)
```

`parallel for` emits `__orl_global_id`. `OrlGpuEngine::CompileModule` then:

1. lowers `__orl_global_id` to PTX register expressions;
2. reflects natural entry arguments;
3. creates `orl_cuda_result` (`i32`);
4. creates `orl_cuda_entry`;
5. calls the natural entry from the kernel;
6. writes the integer result to the result global;
7. emits PTX for `nvptx64-nvidia-cuda`/`sm_52`.

File: `src/orlcomp/orl_gpu.cpp`

### CUDA driver load

`OrlGpuEngine::LoadToDriver`:

1. dynamically loads `libcuda.so.1`/`libcuda.so` or Windows driver DLL;
2. resolves CUDA driver functions;
3. initializes/retains a primary context;
4. optionally links PTX to cubin;
5. loads the module;
6. exposes allocation/import/upload/download/launch functions.

The kernel entry is:

```text
orl_cuda_entry
```

The result symbol is:

```text
orl_cuda_result
```

ROCm target selection exists, but driver execution is not implemented.

## Phase 12: binding current rig/scene data

Compilation creates a reusable executable, but no current joint/mesh values
are in it yet. Each frame binds current scene state.

### `SceneInputCatalog::refresh`

File: `src/orlviewer/graph_scene_inputs.cpp`

`SceneGraphContext::refresh_scene_inputs` calls:

```text
SceneInputCatalog::refresh
  -> rebuild descriptors/source maps
  -> refresh joints/locators/controllers/mesh buffers
  -> update scene input revision
```

The catalog preserves stable IDs and repacks runtime arrays.

### `SceneGraphContext::bind_graph_inputs`

```text
SceneGraphContext::bind_graph_inputs(execution, module)
  -> resolve_graph_input for each graph input
  -> SceneInputCatalog::resolve
  -> OrlGraphExecution::bind_graph_inputs
```

`OrlGraphExecution::bind_graph_inputs`:

1. clears old underlying bindings/results;
2. invokes the resolver for each graph input;
3. maps graph input ID to generated ORL parameter name;
4. checks reflected runtime kind;
5. validates host/device/packed buffer source;
6. validates exact handle type ID;
7. delegates to `OrlExecution::bind_*`.

### Context binding

For each ORL segment, `execute_orl_segment` binds:

```text
SceneGraphContext::ensure_hierarchy_plan
OrlGraphExecution::set_hierarchy_context
OrlGraphExecution::bind_hierarchy_data
OrlGraphExecution::bind_handle_view_context
```

Solver context/packed arena binding is supplied through
`SceneInputCatalog::bind_solver_context` when the generated entry references
`solver_context`.

### Handle resolution

The authored graph stores a typed handle and name parameter. At binding:

```text
find_joint name
  -> ComponentManager/ComponentStore lookup
  -> current packed joint index
  -> HandleValue{HandleTypeIdFor("orlrig::joint_handle"), slot}
```

The slot is dispatch-local. Deleting/repacking joints changes slots without
changing the stable graph/name identity.

## Phase 13: running one ORL graph segment

### CPU

`GraphSceneRuntime::execute_orl_segment`:

1. checks `segment_is_clean`;
2. binds graph inputs;
3. binds hierarchy context/data;
4. binds handle-view context;
5. calls `OrlGraphExecution::evaluate_result`;
6. checks `GraphEvaluationResult::ok`;
7. commits scene writes through `SceneGraphContext::commit_scene_writes`.

The result path reconstructs:

- integer status;
- aliases to host buffers/scalars/handles;
- CUDA device views when applicable.

### CUDA solver

For a CUDA solver segment:

```text
execute_orl_segment
  -> OrlGraphExecution::evaluate_device()
  -> OrlExecution::evaluate_device()
  -> upload changed buffers/packed arenas
  -> upload HandleDeviceContext
  -> bind orl_cuda_entry arguments
  -> launch/synchronize
  -> no host output/result readback
  -> handle_joint_device_view()
  -> SceneGraphContext::set_computed_joints_device()
```

The viewer intentionally does not read back solver Joint data at this point.
The next device consumer can use the device allocation.

### Runtime nodes

`GraphSceneRuntime::execute_runtime_node` looks up the implementation runtime
name and calls a registered adapter:

```text
execute_scene_input_adapter
execute_computed_joints_adapter
execute_lbs_capture_adapter
execute_lbs_evaluate_adapter
```

Runtime nodes are not automatically turned into external ORL calls. The
application adapter supplies scene data or performs a native runtime action.

## Phase 14: deformer output

### LBS capture

`execute_lbs_capture_adapter`:

1. finds selected/deformer mesh;
2. extracts bind positions;
3. reads packed joints;
4. calls `LbsRunner::capture_bind`;
5. compiles/runs `deformer_lbs_capture_bind` if needed;
6. stores inverse bind matrices;
7. marks `DeformerData::bound`.

The direct runner path is:

```text
LbsRunner::ensure_programs
  -> OrlProgram::Compile("use deformer/lbs;")
  -> OrlExecution::Create
  -> bind solver context/joints/inverse binds
```

### LBS evaluation

`execute_lbs_evaluate_adapter`:

1. requires bound deformer and populated weights;
2. obtains current packed joints;
3. calls `LbsRunner::evaluate`;
4. CPU:
   - reads `output_positions`;
   - converts/writes positions into vkkk mesh;
   - calls `context.update_mesh`;
5. CUDA:
   - obtains `LbsRunner::output_device`;
   - calls `context.write_mesh_positions_from_cuda`;
   - writes the device result into the vkkk/Vulkan mesh path.

The ORL implementation is `resource/stdlib/deformer/lbs.orl`:

```text
deformer_lbs_capture_bind
  -> inverse bind matrices

deformer_lbs
  -> parallel per-vertex weighted skinning
```

### Important handoff caveat

The solver graph can publish a typed Joint device view through
`computed_joints`, but the built-in `execute_lbs_evaluate_adapter` currently
calls `LbsRunner::evaluate` using `components.packed_joints()`. Therefore:

- graph-connected computed-joint consumers can use the external device view;
- the built-in LBS runner path may repack the host component snapshot;
- “solver output is always zero-copy into built-in LBS” is not a safe current
  claim.

## Phase 15: subsequent frames and incremental work

### Detecting changes

Each refresh/update cycle:

```text
SceneGraphContext::refresh_scene_inputs
  -> SceneInputCatalog::refresh
  -> SceneGraphContext::update_change_set
```

`update_change_set` compares:

- hierarchy signature (packed IDs/parents);
- scene input revision;
- pose snapshots of joints/controllers/locators.

It emits:

```text
topology/plan invalidated
or dirty joints/controllers/locators
```

### Reusing compiled work

`GraphSceneRuntime::ensure_execution_plan` reuses segments if:

```text
graph_revision
scene_input_revision
graph_fingerprint
backend
```

match previous values.

Changing graph structure, node parameters, scene layout, backend, or relevant
scene input revision discards compiled segments and rebuilds them.

### Skipping clean solver segments

`segment_is_clean` maps segment region nodes into the current
`EvaluationPlan`. If none of the region indices are in
`active_dispatch_.region_indices`, the segment is skipped.

This is dirty-region scheduling, not a complete persistent output cache.
Global/stateful/opaque regions can force execution or full evaluation.

## Alternative path: direct runner instead of graph stage

Not every operation goes through `GraphSceneRuntime`.

### Auto-weight

```text
AutoWeightFeature::request
  -> on_update
  -> AutoWeightFeature::run
  -> extract_mesh
  -> MeshCsrFeature::build (when needed)
  -> AutoWeightRunner::run
     -> OrlProgram::Compile(use auto_weight/<algorithm>)
     -> OrlExecution::Create
     -> bind buffers/counts/context
     -> evaluate/evaluate_device
  -> write WeightData
```

### Legacy two-bone solver

```text
SolverFeature::evaluate_two_bone
  -> SolverRunner::evaluate_two_bone
     -> compile use solver/ik_two_bone
     -> bind typed handles/views/contexts
     -> evaluate
```

### Explicit two-stage runner

`orlrig::TwoStageRunner` provides a non-viewer equivalent:

```text
solver OrlProgram/OrlExecution
  -> named varying DeviceBufferView
  -> deformer OrlProgram/OrlExecution
```

This is useful for understanding device-resident stage handoff without
viewer-specific graph segmentation.

## Machine artifact map

| Stage | Artifact | Function/file |
| --- | --- | --- |
| graph | `GraphModule` + `NodeRegistry` | `src/orlgraph/graph_ir.*` |
| validated graph | `ValidationResult::schedule` | `src/orlgraph/graph_validation.cpp` |
| runtime plan | `ExecutionStep`/`OrlSegment` | `src/orlviewer/graph_scene_runtime.cpp` |
| generated source | `LoweredGraph::source` | `src/orlcomp/orl_graph_lowering.cpp` |
| parsed source | `Program` AST | `src/orlcomp/orl_parser.cpp` |
| semantic model | `AnalysisResult` | `src/orlcomp/orl_analysis.cpp` |
| reflected entry | `OrlRuntimeFunctionSignature`/`ParameterDesc` | `orl_runtime_signature.h`, `orl_exec.cpp` |
| LLVM IR | `llvm::Module`/`OrlExecution::ir()` | `orl_codegen.cpp`, `orl_exec.cpp` |
| optimized LLVM IR | verified O2 module | `orl_optimizer.cpp` |
| CPU artifact | LLJIT module/object | `orl_jit.cpp` |
| CUDA artifact | PTX/cubin + loaded module | `orl_gpu.cpp` |
| bound runtime | `OrlExecution` | `orl_exec.cpp` |
| frame result | host buffer/device view/status | `orl_graph_exec.cpp`, viewer runtime |

## Complete call-chain tables

### Viewer graph stage to CPU machine execution

```text
main
  src/orlviewer/main.cpp
  -> viewport.update
  -> SolverFeature::on_update
  -> GraphSceneRuntime::on_update
  -> GraphSceneRuntime::dispatch_graph
  -> SceneGraphContext::ensure_evaluation_plan
  -> orlrig::compile_evaluation_plan
  -> SceneGraphContext::validate
  -> orlgraph::validate
  -> GraphSceneRuntime::ensure_execution_plan
  -> GraphSceneRuntime::build_orl_segment
  -> OrlGraphProgram::Compile
  -> OrlGraphLowerer::lower
  -> OrlGraphProgram::Compile -> OrlProgram::Compile
  -> Parser::Parse
  -> SemanticAnalyzer::analyze
  -> DescribeRuntimeFunction
  -> OrlGraphExecution::Create
  -> OrlExecution::Create
  -> LlvmIrCodegen::Generate(Host)
  -> LlvmOptimizer::Optimize(O2)
  -> OrlJitEngine::LoadModuleWithOptimization
  -> OrlGraphExecution::bind_graph_inputs
  -> SceneGraphContext::bind_graph_inputs
  -> OrlGraphExecution::evaluate_result
  -> OrlExecution::evaluate
  -> OrlJitEngine::InvokeInt64WithRuntimeArgs*
  -> SceneGraphContext::commit_scene_writes
```

### Viewer graph stage to CUDA machine execution

```text
main
  src/orlviewer/main.cpp
  -> viewport.update
  -> GraphSceneRuntime::on_update
  -> dispatch_graph
  -> SceneGraphContext::ensure_hierarchy_plan
  -> compile_hierarchy_plan
  -> pack_hierarchy_plan
  -> ensure_evaluation_plan
  -> build_cuda_dispatch_plan
  -> ensure_execution_plan
  -> build_orl_segment
  -> OrlGraphProgram::Compile
  -> OrlGraphLowerer::lower
  -> OrlProgram::Compile
  -> OrlGraphExecution::Create(Cuda)
  -> OrlExecution::Create(Cuda)
  -> LlvmIrCodegen::Generate(Cuda)
  -> LlvmOptimizer::Optimize(O2)
  -> OrlGpuEngine::CompileModuleWithOptimization
  -> OrlGpuEngine::LoadToDriver
  -> bind_graph_inputs
  -> bind hierarchy context/data
  -> bind handle view context
  -> OrlGraphExecution::evaluate_device
  -> OrlExecution::evaluate_device
  -> OrlGpuEngine::SetupCudaKernelArguments
  -> OrlGpuEngine::LaunchCudaKernelForElements
  -> OrlGpuEngine::Synchronize
  -> handle_joint_device_view
  -> SceneGraphContext::set_computed_joints_device
```

### Built-in deformer after stage execution

```text
GraphSceneRuntime::execute_runtime_node
  -> execute_lbs_capture_adapter (capture)
     -> LbsRunner::capture_bind
        -> OrlProgram::Compile("use deformer/lbs")
        -> OrlExecution::Create
        -> evaluate

GraphSceneRuntime::execute_runtime_node
  -> execute_lbs_evaluate_adapter
     -> LbsRunner::evaluate
        -> OrlExecution::evaluate/evaluate_device
     -> CPU: write_positions + Context::update_mesh
     -> CUDA: output_device + Context::write_mesh_positions_from_cuda
```

## What is not in the graph-to-machine path

The following are easy assumptions to make but are not current behavior:

- `GraphOptimizer::optimize` is not automatically called by
  `GraphSceneRuntime::dispatch_graph`; the viewer path validates/schedules and
  lowers the active graph. Optimization is an available graph-core API.
- Runtime nodes are not automatically converted into external ORL calls;
  viewer adapters must provide expressions or execute the operation.
- The graph does not return arbitrary newly allocated output buffers; the
  current output ABI aliases buffers/scalars/handles or returns one integer.
- `.oro`/project JSON does not contain executable machine artifacts.
- Project JSON does not contain node definitions, current packed indices,
  device pointers, or vkkk scene objects.
- A CUDA device-only solver result does not automatically mean the built-in
  LBS runner consumes the same device Joint allocation.
- The empty solver graph uses a legacy C++ constraint loop rather than an
  authored graph program.

## Recommended code-reading order

### Start at the user-visible application

1. `src/orlviewer/main.cpp::main`
2. `src/orlviewer/control_map.cpp`
3. `src/orlviewer/component_manager.cpp`
4. `src/orlviewer/selection.hpp`
5. `src/orlviewer/qt/node_graph_editor.cpp`

### Follow graph state

6. `src/orlviewer/scene_graph_context.cpp`
   - `set_graph`
   - `set_stage_graphs`
   - `touch_graph` (header)
   - `update_change_set`
   - `ensure_hierarchy_plan`
   - `ensure_evaluation_plan`
   - `bind_graph_inputs`
7. `src/orlviewer/project_serialization.cpp`
8. `src/orlgraph/graph_ir.hpp`
9. `src/orlgraph/graph_validation.cpp`
10. `src/orlgraph/graph_schedule.cpp`

### Follow generated execution

11. `src/orlviewer/graph_scene_runtime.cpp`
    - `on_update`
    - `dispatch_graph`
    - `ensure_execution_plan`
    - `build_orl_segment`
    - `execute_orl_segment`
    - `execute_runtime_node`
12. `src/orlcomp/orl_graph_lowering.cpp::OrlGraphLowerer::lower`
13. `src/orlexec/orl_graph_exec.cpp`
14. `src/orlexec/orl_exec.cpp`
15. `src/orlcomp/orl_codegen.cpp`
16. `src/orlcomp/orl_optimizer.cpp`
17. `src/orlcomp/orl_jit.cpp` or `src/orlcomp/orl_gpu.cpp`

### Follow rig data and output

18. `src/orlexec/orlrig/abi.hpp`
19. `src/orlexec/orlrig/handle_registry.cpp`
20. `src/orlexec/orlrig/hierarchy.cpp`
21. `src/orlexec/orlrig/evaluation.cpp`
22. `src/orlexec/orlrig/runners.cpp`
23. `src/orlviewer/vp/solver_feature.cpp`
24. `src/orlviewer/vp/deformer_feature.cpp`
25. `src/orlviewer/graph_scene_runtime.cpp::execute_lbs_evaluate_adapter`

This order follows the actual control/data flow rather than the directory
layout alone.
