# `src/orlexec`: execution and rig runtime module

## Scope and source-of-truth

This document describes the current implementation of `src/orlexec` at
commit `4e0fb3d`. It covers:

- the application-facing ORL execution API;
- graph execution over generated ORL entry points;
- the renderer-independent `orlrig` data/runtime layer;
- packed CPU/CUDA ABI contracts;
- hierarchy and incremental evaluation planning;
- standard rig node registration and convenience runners;
- viewer integration boundaries;
- tests and current limitations.

The source and tests are authoritative. The older material under
`plan_docs/` describes intended architecture and migration stages; it is not
always an exact description of the current execution behavior.

`orlexec` is the bridge between compiler output and application data:

```text
ORL source or GraphModule
          |
          v
      orlcomp
  parse/analyze/lower/codegen
          |
          v
       orlexec
  reflect/bind/evaluate
          |
          +-----------------------------+
          |                             |
          v                             v
       CPU JIT                       CUDA driver
          |                             |
          +--------------+--------------+
                         v
             orlrig packed data and ABI
                         |
                         v
             viewer or other application
```

The module deliberately has no window, renderer, or vkkk ownership. The
viewer adds those concerns in `src/orlviewer`.

## Repository inventory

### Top-level `src/orlexec` files

| File | Responsibility |
| --- | --- |
| `CMakeLists.txt` | Builds `orlexec`, `orlc`, and `orlrig` behind the LLVM/codegen target |
| `orl_exec.hpp/.cpp` | Source compile reflection, bindings, CPU/CUDA execution, caches, device views |
| `orl_graph_exec.hpp/.cpp` | Graph lowering wrapper, graph input resolver, graph output reconstruction |

### `src/orlexec/orlrig`

| File | Responsibility |
| --- | --- |
| `orlrig.hpp` | Aggregate include for the rig runtime |
| `abi.hpp` | Shared CPU/GPU strides, context layouts, ABI constants |
| `joint.hpp` | 128-byte joint, local/world transform helpers |
| `locator.hpp` | World-space locator and matrix packing |
| `controller.hpp` | Authoring controller payload and transform packing |
| `mesh.hpp` | Renderer-independent positions, indices, and CSR data |
| `weight.hpp` | 16-byte weight cell and vertex-major weight buffer |
| `deformer.hpp` | LBS deformer state and bind buffers |
| `component_store.hpp/.cpp` | Stable component storage and deterministic packed arrays |
| `handle_registry.hpp/.cpp` | Rig handle types, views, CPU callbacks, storage validation |
| `hierarchy.hpp/.cpp` | Hierarchy plan, ancestor storage, runtime packing |
| `evaluation.hpp/.cpp` | Solver regions, dependency/evaluation plans, dirty dispatch |
| `graph_resources.hpp/.cpp` | Rig graph resources, runtime nodes, stdlib registration, LBS graph |
| `runner_status.hpp` | Common runner result type |
| `runners.hpp/.cpp` | LBS, auto-weight, two-bone solver, CPU dispatch, world matrices |
| `two_stage_runner.hpp/.cpp` | Explicit solver-to-deformer stage execution |
| `ik.hpp` | C++ two-bone chain and pole helpers |

## Build boundary

`src/orlexec/CMakeLists.txt` begins with:

```cmake
if (NOT TARGET orlcomp_codegen)
    return()
endif()
```

Therefore the public executor and renderer-independent rig runtime are not
built in a frontend-only/no-LLVM configuration. When enabled:

```text
orlexec
  links: orlcomp_codegen, orlcomp_graph_lowering, GLM

orlc
  source: ../orlcomp/orlc_main.cpp
  links: orlexec, orlcomp_analysis, optional orlgraph_io

orlrig
  sources: component store, hierarchy, evaluation, runners,
           graph resources, two-stage runner, handle registry
  links: orlexec, optional TBB, GLM
```

The `orlgraph` core remains independent of this layer. `orlexec` is where
logical graph values become runtime parameters, host buffers, device pointers,
or rig-specific handle contexts.

## Public execution API

### Backend and parameter kinds

`ORL::exec::Backend` currently has:

```text
Cpu
Cuda
```

The executor's reflected `ParameterKind` has:

```text
Buffer
Int64
Float64
Handle
Unsupported
```

`ParameterDesc` contains:

- source parameter name;
- ORL type spelling;
- reflected kind;
- runtime element stride for buffers;
- canonical handle type name;
- handle type ID.

The public executor does not expose a general arbitrary-struct buffer ABI.
The stride table is intentionally finite and is implemented by
`element_stride_for` in `orl_exec.cpp`.

### Runtime stride table

The active executor recognizes:

| ORL type | Runtime stride |
| --- | ---: |
| `int`, `float` | 8 bytes |
| `point`, `vector`, `normal`, `vec3`, `dvec3`, `vec4`, `dvec4`, `quat` | 32 bytes |
| `matrix` | 128 bytes |
| `Joint` | 128 bytes |
| `Locator` | 128 bytes |
| `Weight` | 16 bytes |
| `SolverContext` | 48 bytes |
| `HierarchyContext` | 112 bytes |

An unknown custom struct buffer is reflected as unsupported even though LLVM
codegen can lower some custom struct values.

## `OrlBuffer`

`OrlBuffer` is an owning, growable host allocation for one ORL buffer
parameter. It stores:

- `orl_type_`;
- fixed `element_stride_`;
- active `count_`;
- byte vector `storage_`;
- modification `version_`.

### Capacity and version behavior

- `reserve(element_capacity)` grows storage and zero-initializes new bytes.
  Growing increments the version.
- `resize(element_count)` reserves first, zeroes newly active elements, updates
  the active count, and increments the version when the count changes.
- `clear()` sets the active count to zero but retains capacity.
- mutable `data()` calls `mark_modified()` before returning the pointer.
- `mark_modified()` increments the version.
- `write()` requires a non-null source, an exact full-element byte count, and
  an in-range element index.
- `read()` has the corresponding exact-size and range checks without changing
  the version.

The version is used by CUDA execution to avoid uploading an unchanged host
allocation. It is not an element-level dirty range; any version change can
cause the associated device allocation to be uploaded.

`OrlExecution` borrows `OrlBuffer` references. The buffer remains owned by the
application or rig data structure.

## `OrlProgram`

### Compile options

`CompileOptions` contains:

- `entry_function`, default `"compute"`;
- `source_name`;
- include directories for `use`;
- local-to-canonical `handle_type_identities` used by generated graph source.

The identity map matters when graph lowering creates a private local spelling
for an exact or generic handle while retaining its public canonical identity.

### Compile sequence

`OrlProgram::Compile`:

1. Retains source and options in a shared implementation object.
2. Registers the current rig handle views in the global compiler registry.
3. Performs preliminary source-pattern handle-view detection.
4. parses source through `orlcomp::Parser`.
5. runs `SemanticAnalyzer` with handle identity overrides.
6. reports semantic diagnostics before backend codegen.
7. registers analyzed canonical handle types and detects type-ID collisions.
8. calls `DescribeRuntimeFunction` for the selected entry.
9. requires an exact `int` return type.
10. reflects explicit and hidden runtime parameters.
11. rejects unresolved universal `handle` entry parameters.
12. reports unsupported buffer strides through `ParameterKind::Unsupported`.

The `OrlProgram` object retains source so each backend-specific
`OrlExecution` can independently generate or load its own artifact.

### Hidden parameters

`DescribeRuntimeFunction` appends hidden solver/hierarchy parameters when the
parsed program flags are set:

```text
__orl_solver_context
__orl_hierarchy_context
__orl_hierarchy_data
```

These are reflected as buffer-like runtime parameters. Handle-view context is
different: host execution uses registered C ABI symbols, while CUDA codegen
adds a hidden device context parameter when a CUDA module actually uses views.

Context-use flags are program-wide rather than selected-function-local. A
helper function can therefore be reflected with context parameters because
another function in the same expanded program references an implicit global.

## `OrlExecution`

`OrlExecution` is move-only and owns one backend-specific execution state.
Creation returns an object even when invalid so diagnostics are not lost in
an empty `optional`.

### Owned versus borrowed resources

The execution object owns:

- CPU JIT state or loaded object;
- CUDA module and driver allocations created for host buffers;
- CUDA allocations for packed storage;
- CUDA allocations for handle arenas and contexts;
- implicit solver/hierarchy storage created internally;
- binding maps and cache state.

It borrows:

- `OrlBuffer` objects;
- host memory referenced by `PackedBufferView`;
- external CUDA device allocations bound through `bind_device_buffer`;
- `HandleViewContext` objects;
- graph output buffer aliases.

Imported external CUDA pointers are not freed as underlying application
allocations by ORL, although the executor owns its wrapper handle.

## Buffer binding models

### Ordinary host buffers

```cpp
execution.bind_buffer("positions", positions);
```

The executor validates:

- parameter name;
- `ParameterKind::Buffer`;
- ORL type spelling;
- exact stride;
- active byte range.

On CPU, the generated wrapper receives the host pointer directly. On CUDA,
the executor allocates/reuses a device allocation keyed by the host buffer
and uploads it when the buffer version changed.

### Packed host views

`PackedBufferView` describes a range of one mutable host allocation:

```text
data
storage_bytes
offset
bytes
version
```

Several ORL parameters can bind to ranges of one packed allocation. CUDA
uploads the complete packed allocation once and passes each parameter a device
pointer plus its range offset.

The current implementation requires all packed views participating in one
execution to share the same allocation. The version belongs to the storage
allocation, not an individual view.

### External device buffers

`bind_device_buffer(parameter, device_ptr, bytes)` is CUDA-only. The executor
imports the pointer into the GPU engine, records the declared byte size, and
does not copy it from a host `OrlBuffer`. This is used for device-to-device
stage handoff and viewer/Vulkan allocations.

### Scalars

`bind_int` stores an `int64_t`; `bind_float` stores a `double`. Scalars are
placed into grouped arrays in reflected parameter order when invoking the CPU
wrapper or constructing CUDA arguments.

### Exact handles

`bind_handle` requires:

- reflected `ParameterKind::Handle`;
- a valid nonzero type ID and nonnegative slot;
- exact equality between bound type ID and reflected canonical handle ID.

Handle parameters are not persistent bindings across evaluation. The current
`evaluate()` and `evaluate_device()` paths clear handle values after an
evaluation, so callers must bind them for each dispatch.

## Packed contexts

### Solver context

`SolverContext` is:

```text
joint_count
controller_count
locator_count
joints_offset
controllers_offset
locators_offset
```

The offsets are byte offsets from the beginning of the bound solver context
storage. They permit a shared packed arena on CPU and CUDA:

```text
SolverContext
aligned Joint array
aligned Locator array
aligned Controller/matrix array
```

`set_solver_context` updates the value-only fields. `bind_solver_context`
binds a `PackedBufferView` containing the actual arena. The high-level viewer
uses `SceneInputCatalog::bind_solver_context`.

### Hierarchy context

`HierarchyContext` describes data stored in an `int64` `hierarchy_data` array.
Unlike `SolverContext`, its offsets are element indexes into the hierarchy
array, not byte offsets.

The context contains:

- joint and level counts;
- ancestor count/storage mode;
- preorder/depth offsets;
- subtree interval offsets;
- level bucket offsets;
- parent/ancestor offsets;
- total data count.

`set_hierarchy_context` binds the fixed context value and
`bind_hierarchy_data` binds the static data buffer.

### Handle view context

`bind_handle_view_context` associates a rig `HandleViewContext` with the
execution. It validates that the topology revision is nonzero and activates
the context around CPU evaluation.

For CUDA, the executor copies the joint/locator storage into GPU allocations
and constructs a `HandleDeviceContext` containing:

```text
joint_arena
locator_arena
joint_count
locator_count
joint_stride
locator_stride
topology_revision
flags (writable bits)
```

The handle context is separate from `SolverContext`: solver context describes
implicit source buffers, while handle context describes typed view storage.

## Backend creation and cache flow

`OrlExecution::Create` performs:

1. program validity check;
2. backend and source cache-material construction;
3. implicit context initialization;
4. CPU object or CUDA cubin/PTX cache lookup;
5. source reparse and target-specific `LlvmIrCodegen` on miss;
6. O2 optimization;
7. native `OrlJitEngine` or `OrlGpuEngine` initialization;
8. device module loading for CUDA;
9. artifact save on successful compile.

The executor prints JIT and kernel timing information to stdout. Timing
includes parse, codegen, backend compilation/loading, upload, kernel, and
download phases depending on path.

The cache material includes source text, entry, source name, include paths,
LLVM/version/target data, handle ABI, and view-registry fingerprint. It does
not hash the contents of transitive `use` files; a changed stdlib module at
the same path can therefore require manual cache invalidation.

## CPU evaluation

`evaluate_impl`:

1. installs a scoped handle-view context if bound;
2. validates all required bindings;
3. builds ordered buffer pointer, integer, float, and handle-lane arrays;
4. chooses one of the JIT invocation signatures based on handles and hidden
   contexts;
5. invokes `__orl_host_entry_<entry>`;
6. returns an `optional<int64_t>`.

The CPU ignores the public `element_count` argument. CPU ORL programs use
their own scalar count parameters and loops.

The host JIT registers:

- `__orl_parallel_for` from the TBB runtime;
- registered application symbols, including rig handle-view callbacks.

## CUDA evaluation

`evaluate_impl` on CUDA:

1. prepares/reuses device storage for ordinary host buffers;
2. uploads buffers whose versions changed;
3. prepares/reuses the packed allocation for `PackedBufferView`s;
4. creates argument records in reflected declaration order;
5. encodes exact handles as two 64-bit scalar lanes;
6. ensures device handle arenas/context when required;
7. calls `SetupCudaKernelArguments` for `orl_cuda_entry`;
8. launches `element_count` work items;
9. synchronizes;
10. optionally downloads host-visible buffers, writable handle arenas, and
    the result global.

`evaluate_device()` follows the same launch path but intentionally skips host
buffer and result readback. `synchronize()` can be called separately.

The public result is `int64_t`, but the CUDA backend stores
`orl_cuda_result` as `int32_t`; large return values are truncated before
being widened.

### Device views

`device_buffer_view(parameter)` returns a device pointer and byte count for:

- ordinary host buffers;
- packed-buffer subranges;
- external device buffers;
- selected context/arena allocations where supported.

`handle_joint_device_view()` returns the device allocation used by typed joint
views. The returned view is borrowed and its producing execution/context must
remain alive while a consumer uses it.

## Graph execution API

### `GraphInputBinding`

`GraphInputBinding` can contain:

- host `OrlBuffer*`;
- packed host view;
- external device pointer and byte size;
- integer or float scalar;
- exact `HandleValue`;
- expected runtime kind and element count.

`GraphInputResolver` receives an `InterfacePort` and fills this binding. The
application decides how a logical graph input maps to scene data.

### `OrlGraphProgram`

`OrlGraphProgram::Compile`:

1. calls `OrlGraphLowerer`;
2. retains generated source and entry name;
3. retains lowered output descriptors;
4. retains scene revision from lowering options;
5. compiles the generated source as an `OrlProgram`.

It exposes:

- `valid`;
- generated source;
- entry;
- scene revision;
- reflected parameters;
- graph output descriptors;
- diagnostics.

The scene revision lets the viewer reject a compiled program whose runtime
scene snapshot is no longer the one used by output resolvers.

### `OrlGraphExecution`

This is a thin graph-aware wrapper around `OrlExecution`. It stores:

- host buffer aliases;
- host scalar values;
- host handle values;
- reflected parameter descriptions;
- output descriptors and last output values;
- errors.

`bind_graph_inputs`:

1. maps graph interface ports to generated ORL parameter names;
2. invokes the resolver;
3. verifies logical/runtime kind compatibility;
4. validates buffer counts/ranges and exact handle type;
5. delegates to the underlying execution.

`evaluate_result` reconstructs:

- returned integer status;
- buffer aliases;
- float/int scalar aliases;
- handle aliases;
- device views where available.

Graph output values do not own aliased buffers.

## Rig ABI and data model

### Joint

`Joint` is `alignas(32)` and exactly 128 bytes:

```text
0    parent       int64
8    selected     int64
16   pad0         int64
24   pad1         int64
32   translation  4 doubles
64   rotation     4 doubles
96   scale        4 doubles
```

`parent` is a packed joint index; `-1` means root. Translation/rotation/scale
are local-space values. `joint_local_matrix`, `joint_world_matrix`,
`write_joint_local_matrix`, and `write_joint_world_matrix` provide C++ helper
operations.

The host helper bounds parent traversal at 64 links. The ORL stdlib helper
traverses until a negative parent; the hierarchy compiler rejects cycles, but
raw malformed buffers can bypass that validation.

### Locator

`Locator` contains one GLM matrix. It is treated as a 128-byte ORL matrix
buffer and represents a world-space transform. `pack_xform` converts GLM's
column-major layout to ORL's row-major double matrix.

### Controller

`Controller` is authoring data:

- setup `xform`;
- animation `input_xform`.

Its effective world transform is setup times input. New compiled graph inputs
do not expose controllers as typed graph handles; the viewer packs controller
transforms into solver inputs when needed.

### Weight and deformer

`Weight` is:

```text
double weight
int64 joint
```

Weight buffers are vertex-major:

```text
weights[vertex * weight_cnt + slot]
```

`DeformerData` owns bind positions and inverse-bind buffers, mesh name, bind
model, type (`"lbs"`), and bound state.

### Mesh

`MeshData` is renderer-independent:

- object-space positions;
- triangle/index data;
- model matrix.

`MeshCsrData` stores offsets and neighbor indices for surface-based auto-weight
algorithms.

## `ComponentStore`

`ComponentStore` owns renderer-independent component records:

```text
Joint
Weight
Curve
Constraint
Deformer
Controller
Locator
```

Each record has:

- stable process-local `ComponentId`;
- unique non-empty name;
- `ComponentKind`;
- variant payload where applicable.

It maintains:

- ID and name maps;
- deterministic joint order;
- deterministic locator order;
- topology revision.

Creation/destruction changes topology revision. Packed joint and locator
vectors are built from the order vectors. `joint_index` and `locator_index`
resolve stable IDs to current packed slots.

The store does not own viewer display resources or GPU allocations.

## Handle-view registry

### Registered types

`register_rig_handle_views` installs:

| Source | Destination | Storage |
| --- | --- | --- |
| `orlrig::joint_handle` | `Joint` | joint arena |
| `orlrig::locator_handle` | `Locator` | locator arena |
| `orlrig::joint_handle` | `WorldTransform` | joint arena + hierarchy |

Joint fields include parent, selected, padding, translation, rotation, and
scale. Locator and world views expose one matrix field.

### CPU access

The active CPU context is thread-local. Every read/write validates:

- handle is nonzero and slot nonnegative;
- type ID equals expected nominal type;
- storage data exists;
- slot is within count;
- stride is large enough;
- topology revision exists;
- writes use writable storage.

World matrix access copies the current joint arena into a temporary vector
before computing hierarchy-aware transforms. World writes decompose a matrix
back to local translation/rotation/scale.

### CUDA access

CUDA handle views use byte offsets and strides supplied by
`HandleDeviceContext`, not host callback symbols. The generated code loads
arena addresses and performs packed field reads/writes. World transforms call
device-compatible hierarchy helpers.

## `orlrig::HierarchyPlan`

### Compile

`compile_hierarchy_plan` consumes packed joints and stable IDs. It rejects:

- ID/data vector size mismatch;
- parent index below `-1`;
- parent index outside joint count;
- self-parenting;
- cycles or unreachable nodes.

It computes:

- stable-ID preorder;
- depth;
- subtree begin/end intervals;
- optional level buckets;
- either parent-chain storage or flattened ancestor lists.

### Runtime pack

`pack_hierarchy_plan` writes one `int64` data array and a
`HierarchyContext`. Offsets are element indices into the data array. Empty
hierarchies retain a dummy data element while all logical counts remain zero,
so CUDA receives a non-null buffer.

### Parallel world matrices

`compute_world_matrices_parallel` uses level buckets and TBB to compute
independent same-level world matrices after parents have completed. It is
separate from ORL's generated `parallel for` runtime.

## `EvaluationPlan`

### Region discovery

`compile_evaluation_plan` validates and schedules a graph, then creates
`SolverRegion`s for:

- solver/constraint definitions;
- definitions with partial footprints;
- non-identity ORL functions.

Each region records:

- definition/node identity;
- declared/global/stateful/sparse flags;
- propagation mode;
- read/write/affected joints;
- read/write controllers and locators;
- resource effects;
- packed indices;
- region dependencies.

Typed handle effects are resolved to components when graph inputs are
constant `find_joint`/`find_locator` nodes. Unresolved or mismatched effects
become global.

### Hierarchy propagation

Reads add ancestors so world-space dependencies are preserved. Writes add
themselves plus descendants/ancestors according to the declared propagation:

```text
none
ancestors
descendants
ancestors_and_descendants
full
```

Global/stateful/full regions affect the full preorder joint set and mark the
plan as requiring full evaluation.

Two-bone nodes receive additional direct-parent validation for root/mid/end.
Constraint operations default to descendant propagation when they write joints.

### Dependencies

Region dependencies come from:

- graph connections;
- shared resource reads/writes;
- joint writers/readers;
- controller/locator mappings;
- hierarchy overlap;
- global region ordering.

The planner reports:

- competing writers without a deterministic order;
- dependency cycles;
- invalid graph/hierarchy references.

It emits graph order and independent batches.

### Dirty plans

`build_dynamic_dispatch_plan`:

1. selects regions intersecting dirty joints/controllers/locators;
2. closes over dependencies;
3. adds regions whose affected sets overlap selected inputs;
4. switches to full evaluation when selected density exceeds the threshold
   (default `0.6`);
5. groups selected regions into CPU level batches.

`build_cuda_dispatch_plan` can emit dirty ranges only when every selected
region supports sparse dispatch. Otherwise it returns a full-evaluation plan.

`pack_dynamic_dispatch_plan` emits `SolverDispatchContext` plus integer data:
range pairs followed by region indices.

Current viewer code computes this data, but the dispatch context is not yet a
general implicit ORL runtime parameter. The viewer primarily uses the plan to
skip clean segments and to decide when full evaluation is required.

## Node registration and standard graph resources

`graph_resources.cpp` defines:

- stable scene binding names;
- LBS resource IDs;
- typed scene input definitions;
- computed-joints runtime definition;
- LBS capture/deform runtime definitions;
- dynamic compilation/registration of public stdlib nodes;
- `make_lbs_graph`.

### Scene input definitions

Current runtime scene definitions:

```text
orlrig.input.find_joint
  parameter: name:string
  output: handle:orlrig::joint_handle

orlrig.input.find_locator
  parameter: name:string
  output: handle:orlrig::locator_handle

orlrig.input.find_mesh
  parameter: name:string
  output: handle:int64
```

There is no `find_controller` definition in the current registry. Controllers
remain viewer authoring/packing data.

### LBS graph

`make_lbs_graph` creates resources:

```text
bind_positions
posed_positions
joints
inverse_binds
weights
```

It adds:

```text
capture_bind -> inverse_binds -> deform
bind_positions -----------------> deform
weights -------------------------> deform
deform.posed_positions -> graph output
```

The runtime nodes are intentionally non-pure and non-inlineable. Standard
stdlib solver, constraint, and auto-weight definitions are registered for
authored graphs but are not all instantiated in the standard LBS graph.

### Dynamic stdlib registration

`register_compiled_stdlib_nodes` scans:

```text
resource/stdlib/solver
resource/stdlib/constraint
resource/stdlib/auto_weight
```

It compiles requested exports to `.oro`, ignores helper-only files that do not
contain the requested export, and registers the result. Solver stems
`hd_id`, `spline_ik`, and `full_body_ik` are explicitly skipped.

## Runners

### `LbsRunner`

Owns:

- capture and deform `OrlProgram`s/executions;
- packed joints;
- solver context and auxiliary arrays;
- output positions;
- last vertex count.

Capture computes inverse bind matrices from current world joints. Evaluation
binds inverse binds, weights, count parameters, solver context, and output
positions. CPU and CUDA paths share the ORL source.

`device_only=true` avoids host output readback. `output_device()` exposes the
result allocation when available. `readback()` provides a host-visible path.

### `AutoWeightRunner`

Current runner names:

```text
closest_distance
closest_hierarchy
heat
geodesic
```

It dynamically compiles the matching `resource/stdlib/auto_weight` module and
allocates:

- positions;
- packed joints;
- offsets/neighbors where required;
- radii;
- scratch;
- output weights.

It rejects unsupported names and requires CSR data when the reflected program
has `offsets` or `neighbors` parameters.

### `SolverRunner`

The direct C++ convenience runner currently exposes two-bone IK. It supports:

- vectors of C++ joints;
- an `OrlBuffer` of joints;
- locator or controller targets;
- component-store IDs;
- CPU or CUDA backend;
- optional compiled hierarchy validation.

The typed ORL path binds exact joint and locator handles and a
`HandleViewContext`. It checks root/mid/end indices and direct parent
relationships before execution.

### `TwoStageRunner`

This runner explicitly models:

```text
solver program -> named varying -> deformer program
```

Stage bindings support host buffers, imported device views, integers, and
floats. A logical varying can use different source parameter names in the two
stages. CUDA execution can import solver-produced device allocations directly
into the deformer stage.

## Viewer boundary

The viewer uses `orlexec` in three ways:

1. `GraphSceneRuntime` compiles and runs generated graph ORL segments.
2. `LbsRunner`, `AutoWeightRunner`, and `SolverRunner` provide convenience
   execution paths for viewport features.
3. `SceneInputCatalog` binds current scene storage and exposes device views.

The `orlrig` layer itself never owns:

- a window;
- a Vulkan context;
- a vkkk scene;
- a display object;
- an editor layout.

This separation is important for running `orlrig_tests` and executor tests
without the full viewer.

## Tests

### Direct executor

`tests/exec_tests/exec_tests.cpp` covers:

- exact handle reflection and type rejection;
- CPU storage-backed Joint views;
- semantic diagnostics before backend creation;
- object/PTX/cubin cache behavior;
- buffer growth and versioning;
- named binding errors;
- solver context;
- packed solver arena;
- hierarchy context/data;
- CUDA buffers and packed ranges.

### Graph execution/lowering

`tests/graph_lowering_tests/graph_lowering_tests.cpp` covers:

- exact handle lanes;
- graph source generation;
- resolver binding;
- output aliases;
- typed scene handles;
- typed IK view writes;
- union specialization;
- optional CUDA parity.

### Rig runtime

`tests/orlrig_tests/orlrig_tests.cpp` covers:

- ABI layout;
- handle registry/storage guards;
- component packing;
- hierarchy plans and invalid input;
- LBS;
- two-stage execution;
- CUDA device varying handoff;
- auto-weight runner names;
- two-bone IK;
- evaluation plans, dirty dispatch, writer conflicts, and cycles.

### Scene integration

`tests/scene_graph_tests/scene_graph_tests.cpp` covers:

- active LBS graph;
- typed solver graph;
- host writeback;
- computed joint device view;
- shared CUDA arena;
- hierarchy cache/invalidation;
- pose versus topology changes;
- stable handles after repacking.

CUDA and window/device integration tests are conditional or headless; they do
not fully prove a production Vulkan/CUDA interop frame.

## Current limitations and risks

### ABI and output limits

- Runtime entry functions must return exact `int`.
- `OrlBuffer` stride reflection has no generic custom-struct buffer ABI.
- Graph outputs cannot allocate arbitrary computed buffers; buffer/float/handle
  outputs generally alias bound graph inputs, and only one integer status is
  returned.
- CUDA result storage is 32-bit.

### Runtime coverage

- Runtime nodes require application adapters.
- `AutoWeightRunner` exposes four algorithms even though the registry contains
  a broader set of imported definitions.
- `SolverRunner` exposes two-bone IK only.
- `hd_id`, `spline_ik`, and `full_body_ik` are skipped from standard registry
  registration.

### Device handoff

- Device views are borrowed; producer ownership must outlive consumers.
- The viewer exposes a computed-joints device view, but the built-in LBS
  adapter can still repack host `components.packed_joints()`, so zero-copy
  solver-to-LBS behavior is not universal.
- Host component writeback is rejected after device-only evaluation to avoid
  committing stale transforms.

### Incremental evaluation

- The plan and dirty-region selection are implemented.
- CUDA sparse dispatch falls back to full evaluation when selected regions do
  not support sparse operation.
- Viewer code packs `SolverDispatchContext`, but it is not a general implicit
  ORL context ABI and is not universally passed to kernels.

### Cache invalidation

The backend cache key includes the top-level source and include-path strings,
but not transitive `use` file contents. Graph IR cache and backend artifact
cache are separate; a valid `.oro` cache hit does not imply a valid CPU/CUDA
artifact hit.

### Storage identity

`ComponentId`, packed index, runtime handle slot, file component ID, and graph
`StableId` are distinct namespaces. Stable host writeback is maintained by
mapping IDs, but ORL buffers still use packed indices at runtime.

## Recommended reading order

1. `orl_exec.hpp` and `orl_exec.cpp` for the public compile/bind/evaluate path.
2. `orl_graph_exec.hpp` and `orl_graph_exec.cpp` for graph bindings/output
   aliases.
3. `abi.hpp`, `joint.hpp`, `component_store.hpp`, and `handle_registry.hpp` for
   data and handle contracts.
4. `hierarchy.hpp/.cpp` for topology compilation and packing.
5. `evaluation.hpp/.cpp` for region footprints and dirty dispatch.
6. `graph_resources.cpp` for graph/runtime registration.
7. `runners.cpp` and `two_stage_runner.cpp` for convenience paths.
8. `tests/exec_tests`, `tests/orlrig_tests`, and
   `tests/graph_lowering_tests` while tracing a concrete execution.
9. `docs/orlcomp.md` for the compiler ABI that this module consumes.
