# ORL project summary

## Scope and snapshot

This is a first-pass implementation map of the repository, prepared from
commit `4e0fb3d` (`adjust doc folder structure`). It describes what the source
currently does, not what the older design notes intended it to do.

The project is a C++20 rigging and animation system built around a small
handwritten language named ORL. ORL source can be compiled to LLVM IR and then
executed by a native JIT or lowered to CUDA/PTX. The same compiler is also used
to turn exported ORL procedures into typed graph-node definitions. A
renderer-independent rig runtime and an optional Vulkan viewer consume those
compiler and graph services.

The implementation is spread across several layers:

```text
ORL source and stdlib
        |
        v
src/orlcomp
  lexer, parser, AST, semantic analysis, graph import/lowering,
  LLVM codegen, optimizer, JIT, CUDA backend, caches
        |
        +------------------------------+
        |                              |
        v                              v
src/orlgraph                    src/orlexec
typed graph IR                  source/graph execution API
        |                              |
        +--------------+---------------+
                       v
                 src/orlexec/orlrig
           rig storage, ABI, hierarchy,
           evaluation planning, runners
                       |
                       v
                 src/orlviewer
       scene adapter, editor, viewport,
       project files, CPU/GPU handoff
```

Module-level implementation documents:

- [`orlcomp`](orlcomp.md) — compiler/frontend/backend
- [`orlexec`](orlexec.md) — execution and rig runtime
- [`orlgraph`](orlgraph.md) — target-independent graph IR
- [`orlviewer`](orlviewer.md) — viewer/editor and scene integration
- [`orltest`](orltest.md) — legacy/orphaned test harness

## What the project is now

The repository is no longer only a procedural language compiler. It currently
contains four connected systems:

1. **A procedural ORL compiler.** Source is textually expanded, parsed into a
   polymorphic AST, semantically checked, lowered directly to LLVM IR, and
   executed on CPU or CUDA.
2. **A target-independent graph IR.** Graph modules contain typed node
   instances, ports, resources, effects, domains, shapes, stable identities,
   stage information, and provenance. They can be validated, scheduled,
   optimized, reflected, serialized, and lowered back into ORL source.
3. **A reusable rig execution layer.** `orlexec` owns dynamic source execution
   and buffer bindings. `orlrig` owns packed joint/locator/weight data,
   hierarchy plans, incremental evaluation plans, standard rig node
   registration, and convenience runners.
4. **A viewer/editor integration.** The viewer maps scene names and components
   to graph inputs, builds solver/deformer stages, manages authoring state,
   executes graph segments, and transfers computed GPU buffers into the
   deformation/render path.

The source and tests show that typed handles are now a real feature rather than
only a proposal. Exact joint and locator handles, handle unions, `is` tests,
storage-backed views, typed effects, dirty evaluation plans, and CUDA device
handoff are implemented in the current snapshot.

## Repository map

### Root and build files

| Path | Current role |
| --- | --- |
| `CMakeLists.txt` | Root CMake configuration, dependency discovery, and target inclusion |
| `CMakeUserPresets.json` | Includes generated presets from `build/build/generators`; not a standalone preset set |
| `.gitmodules` | Declares `thirdparty/vkkk` |
| `README.md` | Currently contains only the `ORL` heading |
| `resource/` | ORL stdlib, viewer shaders, and input configuration |
| `tests/` | Catch2 test executables for compiler, graph, rig, execution, and viewer behavior |
| `plan_docs/` | Design proposals, implementation plans, and historical architecture notes |
| `project_file_details.md` | Detailed viewer project JSON format |
| `node_refs.md` | Current graph-node and stdlib reference catalog |

There is also a tracked root-level `aim.orl` prototype. It is an early syntax
and design sketch and is not part of the active compiler resource pipeline.

### `src/orlcomp`: language and compiler

`src/orlcomp` is the first implementation layer. It is split into CMake
libraries so the frontend and graph import can be used without LLVM:

| Target | Contents | LLVM required |
| --- | --- | --- |
| `orlcomp_lexer` | Tokens and lexical scanning | No |
| `orlcomp_parser` | Textual `use` expansion, parser, AST | No |
| `orlcomp_analysis` | Semantic summaries and ORL-to-node import | No |
| `orlcomp_graph_lowering` | Typed graph-to-ORL source lowering | No |
| `orlcomp_codegen` | LLVM IR, optimization, cache, JIT, CUDA, TBB runtime | Yes |

The compiler supports scalar and aggregate values, data-only structs, fixed
arrays, buffers, typed handles, handle-backed views, metadata-bearing exported
functions, and a restricted `parallel for`. A detailed inventory is in
[`orlcomp.md`](orlcomp.md).

### `src/orlgraph`: graph IR

`orlgraph` is deliberately independent of LLVM, CUDA, TBB, GLM, Eigen, the
viewer, and `orlexec`. Its public aggregate include is
`src/orlgraph/orlgraph.hpp`.

The core model is in `graph_ir.hpp`:

- `GraphModule`: module metadata, nodes, connections, interfaces, and resources.
- `NodeRegistry`: reusable `NodeDefinition`, conversion, and subgraph lookup.
- `NodeDefinition`: typed ports, implementation reference, stage mask, effects,
  capabilities, purity/statefulness, provenance, and optional partial
  evaluation footprint.
- `NodeInstance`: a graph-local use of a definition with parameters and
  provenance.
- `Connection` and `Endpoint`: graph dataflow, optional conversion, shape,
  feedback flag, and provenance.
- `InterfacePort` and `Resource`: external graph inputs/outputs and observable
  data resources.

Supporting modules are:

- `graph_ids.*`: serialized `StableId` and `Version`. Stable graph identity is
  intentionally independent of process-local `orlrig::ComponentId`.
- `graph_types.*`: logical scalar, aggregate, buffer, struct, array, and handle
  types; domains; symbolic shapes; constant values; assignability helpers.
- `graph_validation.*`: definition/port existence, required connections, stage
  restrictions, type/cardinality/domain/shape checks, conversion requirements,
  effect checks, and ordinary-cycle diagnostics.
- `graph_schedule.*`: deterministic topological ordering. Explicit feedback
  edges are treated differently from ordinary DAG edges.
- `graph_optimizer.*`: constant specialization, identity simplification,
  common elimination, and effect-aware dead-node removal.
- `graph_reflection.*`: reflected inputs, outputs, resources, node ports,
  effects, provenance, and schedule.
- `graph_serialization.*`: optional RapidJSON-backed `.oro`, editable graph
  JSON, staged solver/deformer JSON, canonical text, content hashes, and graph
  cache helpers.

`orlgraph_io` is built only when RapidJSON is available. The editable graph
formats store references to definitions by stable ID; the application-owned
`NodeRegistry` remains responsible for supplying those definitions.

### `src/orlexec`: dynamic execution API

`src/orlexec/orl_exec.hpp` exposes a backend-neutral application API:

- `OrlBuffer`: typed, growable host storage with element stride, count,
  capacity, and modification version.
- `OrlProgram::Compile`: parses and semantically checks source, identifies an
  entry function, and reflects its runtime parameters.
- `OrlExecution::Create`: builds a CPU or CUDA backend for a compiled program.
- named buffer, packed-buffer, external CUDA-device, integer, float, and
  exact-handle bindings;
- implicit solver-context and hierarchy-context binding;
- CPU evaluation, CUDA evaluation, device-only evaluation, synchronization,
  and device-buffer views;
- handle-view context binding and a computed-joint device view for the
  solver-to-deformer handoff.

`src/orlexec/orl_graph_exec.*` wraps graph lowering and the same execution API:

1. `OrlGraphProgram::Compile` validates/lowers a `GraphModule`.
2. The generated ORL entry source is parsed and reflected as an
   `OrlProgram`.
3. `OrlGraphExecution` binds graph inputs through a resolver.
4. `evaluate_result` returns scalar status values plus aliased buffer, float,
   integer, handle, and device-view outputs where the current ABI permits.

The execution API is the bridge between compiler output and the rig/viewer
systems. It does not itself own a scene or a window.

### `src/orlexec/orlrig`: renderer-independent rig runtime

`orlrig` is a data and evaluation layer under `orlexec`. It intentionally has
no renderer or window ownership.

#### Shared ABI and value types

`abi.hpp` defines the common CPU/GPU storage contract:

- `Joint`: 128 bytes.
- `Weight`: 16 bytes.
- `point`: four doubles in a buffer slot.
- `matrix`: sixteen doubles in row-major ORL order.
- `SolverContext`: six 64-bit fields containing counts and byte offsets into a
  packed arena.
- `HierarchyContext`: fourteen 64-bit fields describing packed hierarchy data.
- `SolverDispatchContext`: six 64-bit fields describing dirty dispatch data.

`joint.hpp`, `locator.hpp`, `controller.hpp`, `weight.hpp`, `mesh.hpp`, and
`deformer.hpp` provide C++ data types and packing helpers. Controllers are
authoring payloads; new compiled graphs use joint and locator handles rather
than a controller graph type.

#### Component and hierarchy state

`ComponentStore` owns metadata and payloads for joints, controllers, locators,
weights, constraints, deformers, and curves. It uses process-local
`ComponentId` values plus deterministic joint/locator order vectors. Runtime
`Joint.parent` values are packed-array indices.

`hierarchy.*` turns packed joints into a `HierarchyPlan`:

- stable component IDs in hierarchy preorder;
- depth and subtree intervals;
- optional level buckets;
- either parent-chain or flattened ancestor storage;
- cycle, invalid-parent, and self-parent validation.

`pack_hierarchy_plan` turns that plan into `HierarchyContext` plus an integer
data buffer for ORL and CUDA.

#### Typed handle registry

`handle_registry.*` supplies the rig-specific interpretation of compiler
handles:

- `orlrig::joint_handle` maps to packed `Joint` storage.
- `orlrig::locator_handle` maps to packed `Locator` storage.
- `WorldTransform` is a registered joint view.
- CPU reads/writes use registered C ABI symbols and a thread-local active
  `HandleViewContext`.
- CUDA reads/writes use the `HandleDeviceContext` arena ABI, with world
  transforms delegated to device-visible hierarchy helpers.

The runtime token is a canonical handle type ID plus a dispatch-local slot.
It is not a serialized component ID and not a permanent packed index.

#### Evaluation and runners

`evaluation.*` compiles graph and hierarchy information into `EvaluationPlan`
and `SolverRegion` records. It resolves typed handle effects to components
when graph `find_*` parameters are constant, expands ancestor/descendant
footprints, adds graph/resource dependencies, detects writer conflicts/cycles,
and creates dynamic or CUDA dispatch plans. Missing or opaque metadata is
conservative and forces broader evaluation.

`runners.*` contains convenience paths:

- `LbsRunner`: capture bind matrices and evaluate linear blend skinning.
- `AutoWeightRunner`: execute supported auto-weight source modules.
- `SolverRunner`: direct two-bone IK convenience API, including hierarchy and
  typed-handle setup.
- `dispatch_cpu_levels`: run independent solver regions by level.
- `compute_world_matrices_parallel`: host hierarchy helper.

`two_stage_runner.*` makes the solver/deformer boundary explicit. On CUDA,
varying buffers can stay device-resident between stages.

`graph_resources.*` registers:

- scene input definitions for joint, locator, and mesh lookup;
- runtime computed-joint, LBS bind-capture, and LBS evaluation nodes;
- public solver, constraint, and auto-weight ORL definitions compiled from
  `resource/stdlib`;
- the standard LBS graph builder.

The standard LBS graph itself instantiates the two LBS runtime nodes. The other
registered definitions are available for authored graphs; registration does
not imply that every algorithm has a dedicated C++ runner.

### `src/orlviewer`: editor and scene integration

The viewer is an executable, not a reusable viewer library. Its target is
`orl_viewer` and it is enabled by default with `ORL_BUILD_VIEWER=ON`.

#### Scene and component adapter

`ComponentManager` wraps `orlrig::ComponentStore` and adds viewer-only state:

- display links to vkkk objects;
- controller shapes and curve links;
- controller attachments;
- selection and viewport interaction integration.

The standalone rig store remains the authoritative runtime data source.

#### `SceneGraphContext`

`scene_graph_context.*` owns the relationship between one scene and its
scene-independent graph documents. It tracks:

- active legacy or staged solver/deformer graphs;
- the node registry;
- graph, edit, scene-input, evaluation, topology, and pose revisions;
- explicit graph-to-scene input mappings;
- the compiled hierarchy and evaluation plans;
- packed hierarchy runtime data;
- pending `ChangeSet` dirty state;
- computed-joint device views;
- host scene writeback.

`ChangeSet` separates topology invalidation, full evaluation, and changed
joint/controller/locator sets.

#### `SceneInputCatalog`

`graph_scene_inputs.*` converts the current scene into graph bindings. It
provides namespaced bindings for mesh positions, weights, inverse binds,
joints, locators, controller transforms, counts, and typed joint/locator
handles. It keeps stable component IDs in the catalog, then resolves each
handle to a current type ID and packed slot during binding.

For CUDA evaluation it can pack solver inputs into shared host storage,
upload them through `OrlExecution`, retain an active `HandleViewContext`, and
borrow a computed-joint device view for the downstream deformer. Host
writeback is explicitly rejected after device-only evaluation so stale CPU
transforms are not committed.

#### `GraphSceneRuntime`

`graph_scene_runtime.*` is the main staged execution bridge:

1. validate and schedule the active graph;
2. compile consecutive ORL-function nodes into generated ORL segments;
3. keep runtime adapters as separate execution steps;
4. bind scene inputs, hierarchy data, and typed handle views;
5. build dirty solver dispatch plans;
6. skip clean segments when the plan permits;
7. execute CPU or CUDA ORL code;
8. run LBS/runtime adapters;
9. commit host writes or pass device-resident joint data to the deformer.

The intended data path is:

```text
scene edit
  -> SceneInputCatalog refresh
  -> hierarchy/evaluation plan
  -> dirty graph segments and runtime adapters
  -> solver
  -> computed joint buffer
  -> deformer
  -> CPU mesh update or CUDA/Vulkan shared output
```

#### Project and viewport systems

- `project_serialization.*` stores staged graph JSON, components, attachments,
  constraints, weights, deformer data, evaluation settings, and optional
  editor-only graph layout. Definitions and vkkk scene objects are not
  embedded.
- `control_map.*` maps JSON-configured keyboard/mouse input to immediate or
  modal viewport operations.
- `selection.hpp` resolves selected joints, controllers, locators, objects,
  and transform attributes.
- `ops/`, `vp/`, and related headers implement authoring and viewport
  operations.
- `resource/shaders` contains the viewer's GLSL and compute shaders.

### `resource/stdlib`: ORL-authored rig algorithms

The standard library currently contains 30 ORL files. It is source code
compiled by the compiler, not a C++ plugin API.

| Group | Main contents |
| --- | --- |
| Core | `joint.orl`, `weight.orl`, `locator.orl`, `controller.orl`, `hierarchy.orl` |
| Rig handles | `rig/handles.orl`, exact joint/locator declarations and `WorldTransform` |
| Solvers | FK, typed two-bone IK, history-dependent IK, full-body IK, spline IK |
| Constraints | parent, aim, locator aim, copy transform/translation/rotation/scale, common helpers, handle union helpers |
| Auto-weight | closest joint/distance/hierarchy, envelope, heat, harmonic, geodesic, bounded biharmonic, common helpers |
| Deformer | `deformer/lbs.orl` for bind capture and linear blend skinning |

The core ABI conventions are important:

- `Joint` matches the C++ 128-byte layout.
- `Weight` is vertex-major with fixed influence slots.
- matrices are row-major in ORL; GLM matrices are packed with transposition.
- `solver_context` and `hierarchy_context` are implicit globals injected by
  the compiler when referenced.
- exported functions return integer status/count values and mutate output
  buffers.

The standard library is intentionally not a class/object system. It supplies
data-only structs, functions, and exported graph-node procedures.

### `tests`: behavior map

The root test build adds these executable families:

| Directory | Primary subject |
| --- | --- |
| `frame_tests` | frame utilities |
| `lexer_tests` | tokens, literals, comments, aliases, `use`, handles, exports |
| `parser_tests` | ASTs, precedence, arrays, structs, context flags, handles, metadata, loops |
| `syntax_tests` | accepted/rejected language syntax |
| `analysis_tests` | semantic types, effects, handle narrowing/views, node import, `.oro` |
| `codegen_tests` | LLVM lowering, intrinsics, structs, contexts, handles, stdlib, JIT, TBB parallelism |
| `graph_ir_tests` | graph types, validation, schedules, optimizer, reflection, serialization/cache |
| `graph_lowering_tests` | graph-to-ORL source, conversions, typed handles, adapters, CPU/CUDA |
| `rig_graph_tests` | standard resources, typed scene inputs, public stdlib definitions |
| `gpu_tests` | PTX, CUDA driver, reflection, launch, device handles/views |
| `exec_tests` | source execution, caches, bindings, packed buffers, contexts, handles |
| `skinning_tests` | CPU/CUDA LBS examples |
| `orlrig_tests` | ABI, hierarchy, evaluation plans, rig runners, two-stage execution |
| `scene_graph_tests` | viewer graph context/runtime, handle graphs, writeback, revisions, input/control behavior |
| `vkkk_tests` | shader cache and vkkk integration |

CUDA/device-dependent tests are conditional in practice and generally skip or
report unavailable hardware/runtime conditions. The current Catch2 CMake files
create executables; the repository does not visibly register them with
`add_test()` from the root test setup.

## Build topology and prerequisites

The root project:

- requires C++20;
- defaults to `Debug`;
- enables compile-command generation;
- requires GLM and Eigen3;
- enables tests by default when Catch2 is available;
- includes `src/orlgraph`, `src/orlcomp`, and `src/orlexec`;
- includes the viewer and `thirdparty/vkkk` when `ORL_BUILD_VIEWER` is on.

The compiler frontend and graph core can exist without LLVM. When LLVM is
found, `src/orlcomp/CMakeLists.txt` additionally requires:

- CUDA Toolkit, even though the CPU compiler target is also built;
- TBB;
- zlib and zstd CMake targets;
- LLVM components, with NVPTX components added when that target is built.

`orlexec`, `orlc`, and `orlrig` are skipped when `orlcomp_codegen` is absent.
The viewer additionally needs RapidJSON, vkkk, and Eigen3. Graph I/O
(`orlgraph_io`) is conditional on RapidJSON, while the viewer requires it.

Typical commands, assuming dependencies and the vkkk submodule are available:

```bash
cmake -S . -B build
cmake --build build
```

The available executable/target names are the test names above plus
`orlc`, `orl_viewer`, `orlcomp_*`, `orlgraph`, `orlgraph_io`, `orlexec`, and
`orlrig`.

## Identity, storage, and ABI rules

Several identifiers that look like integers are intentionally different:

| Identity | Owner | Meaning |
| --- | --- | --- |
| `orlgraph::StableId` | graph IR | serialized graph/node/port identity |
| `orlrig::ComponentId` | rig store | process-local scene component identity |
| `HandleValue` | compiler/runtime | dispatch token `{type_id, slot}` for an exact handle |
| packed joint index | runtime buffer | current dense position used by `Joint.parent` and weights |
| file component ID | project JSON | file-local ID remapped on load |

Do not serialize a packed index as a component ID. Do not treat a runtime
handle slot as a stable asset identity.

The shared `Joint` layout is:

```text
offset 0   parent       i64
offset 8   selected     i64
offset 16  pad0/pad1    16 bytes
offset 32  translation  4 doubles
offset 64  rotation     4 doubles
offset 96  scale        4 doubles
total      128 bytes
```

`SolverContext` uses byte offsets into a packed arena, not host pointers, so
the same logical layout can be used by CPU JIT and CUDA. Typed handle views
use a separate `HandleDeviceContext` for device arena addresses, counts,
strides, topology revision, and writable flags.

The matrix conventions cross the C++/ORL/viewer boundary:

- ORL matrices are row-major with translation at indices `[3]`, `[7]`, `[11]`.
- GLM matrices are column-major in memory and are transposed while packing.
- viewer/project JSON stores GLM-style column-major arrays.

## Runtime flows

### Direct CPU ORL

```text
source
  -> OrlProgram::Compile
       Parser + semantic analysis + runtime signature reflection
  -> OrlExecution::Create(Cpu)
       cache lookup or LLVM codegen -> O2 -> native LLJIT
  -> bind named buffers/scalars/handles/contexts
  -> evaluate()
       __orl_host_entry_<entry> wrapper
```

`OrlBuffer` versions allow a CUDA execution to upload only modified host
allocations. CPU execution invokes generated wrappers with grouped buffer,
integer, float, handle-lane, and optional context arguments.

### Direct CUDA ORL

```text
source
  -> Parser + semantic analysis
  -> LLVM AST-to-IR for CUDA target
  -> O2
  -> OrlGpuEngine
       __orl_global_id lowering
       orl_cuda_entry kernel
       orl_cuda_result i32 global
       PTX or optional cubin
  -> CUDA driver load/bind/launch/synchronize
```

The driver is loaded dynamically through the CUDA driver API. Host-backed
buffers may be copied to device, and device-only evaluation can omit result
and buffer readback. ROCm has target-selection code but no implemented driver
execution path.

### Graph-authored execution

```text
GraphModule + NodeRegistry
  -> validate()
  -> topological_schedule()
  -> OrlGraphLowerer
       typed ORL entry source
  -> OrlGraphProgram::Compile
  -> OrlGraphExecution
       graph input resolver and runtime ABI binding
  -> CPU JIT or CUDA execution
```

Graph lowering preserves typed handle identities by generating private nominal
handle names and passing canonical identity overrides to the parser, analyzer,
codegen, and runtime signature. Generic union/open-handle node inputs are
specialized from exact incoming graph edges; the generated ORL path does not
insert a runtime type switch.

### Viewer staged execution

```text
ComponentManager / vkkk scene edits
  -> SceneGraphContext ChangeSet
  -> SceneInputCatalog refresh/packing
  -> hierarchy plan + evaluation plan
  -> ordered runtime adapters and ORL segments
  -> solver stage
  -> computed-joint host/device view
  -> deformer stage
  -> scene/Vulkan mesh output
```

The solver and deformer are separate graph stages. Runtime nodes such as
scene lookup, computed joints, LBS capture, and LBS evaluate are handled by
application adapters, while runs of ordinary ORL-function nodes are compiled
into generated ORL segments.

## Current design boundaries and caveats

### Source is the authority over plan documents

`plan_docs/graph_ir_design.md` explicitly says it is a design proposal. Several
other files in `plan_docs/` describe intended phases or earlier problems. They
are useful for history and rationale, but they are not a complete description
of the current implementation.

Examples of current implementation that older design text may understate:

- graph validation, scheduling, optimization, reflection, serialization, and
  cache helpers exist;
- semantic analysis and graph-node import exist;
- typed handles and handle-view effects exist;
- scene graph runtime segmentation and dirty evaluation planning exist;
- CUDA solver-to-deformer device handoff exists.

Conversely, registration does not mean universal runtime support. A stdlib
function can be imported as a graph definition without having a dedicated C++
runner or full sparse dispatch implementation.

### Important current limitations

- `README.md` is not a useful onboarding document yet.
- The direct runtime ABI accepts exact `int` entry returns and has restricted
  parameter forms. Some language types are accepted by the lexer/analyzer but
  are not mapped by the active LLVM code generator.
- Graph scalar return handling is still centered on a legacy integer status;
  buffers, floats, and handles generally have to alias bound graph inputs.
- Fixed-array, buffer, and context indexing has no generated bounds checks.
- `&&` and `||` are lowered as eager LLVM operations rather than
  short-circuit control flow.
- Binary cache material includes the top-level source and include paths, but
  not the contents of every resolved `use` file. A changed stdlib file at the
  same path therefore deserves cache invalidation care.
- CUDA result storage is an `i32` global even though the public execution
  result is widened to `int64_t`.
- CUDA driver execution is implemented; ROCm driver loading is not.
- Viewer sources are compiled directly into `orl_viewer` and selected tests,
  rather than through a reusable viewer library target.
- The current headless tests cannot fully prove a window-backed Vulkan/CUDA
  interop path.
- Project JSON deliberately excludes node definitions and vkkk scene objects;
  the application must register definitions and reload scene assets.

## Suggested reading order

For learning the repository without assuming the plans are current:

1. `src/orlgraph/graph_ir.hpp` and `graph_types.hpp` for graph vocabulary.
2. `src/orlcomp/orl_ast.h`, `orl_parser.cpp`, and `orl_analysis.cpp` for the
   language and semantic model.
3. [`orlcomp.md`](orlcomp.md) for the complete compiler path.
4. `src/orlexec/orl_exec.hpp` and `orl_graph_exec.hpp` for the runtime ABI.
5. `src/orlexec/orlrig/abi.hpp`, `component_store.hpp`, `hierarchy.hpp`, and
   `evaluation.hpp` for rig data and incremental execution.
6. `src/orlexec/orlrig/graph_resources.cpp` for the graph/runtime boundary.
7. `src/orlviewer/scene_graph_context.hpp`,
   `graph_scene_inputs.hpp`, and `graph_scene_runtime.hpp` for the scene
   adapter and staged evaluation.
8. `project_file_details.md` and `node_refs.md` for serialized and authored
   graph conventions.
9. The matching test directory after each subsystem is understood.

The next module documents should follow the dependency order above. The
current detailed module analysis starts with `src/orlcomp`.
