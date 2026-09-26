# `src/orlgraph`: target-independent graph IR

## Scope and source-of-truth

This document describes the current `src/orlgraph` implementation at commit
`4e0fb3d`. It covers the graph data model, logical types, registry,
validation, scheduling, optimization, reflection, serialization, graph-IR
cache, and the boundaries to `orlcomp`, `orlexec`, `orlrig`, and the viewer.

The graph layer is implemented, but some older documents still describe it as
future work. In particular, `plan_docs/graph_ir_design.md` begins with
“Status: design proposal”. Use current source and tests as the authority:

```text
ORL source
  -> parser / semantic analysis
  -> NodeRegistry definitions

graph authoring
  -> GraphModule
  -> validation
  -> deterministic schedule
  -> GraphOptimizer
  -> .oro / editable JSON / staged JSON
  -> OrlGraphLowerer
  -> generated ORL entry
  -> orlexec CPU/CUDA execution
```

`orlgraph` deliberately does not depend on LLVM, CUDA, TBB, GLM, Eigen,
Vulkan, `orlexec`, `orlrig`, or the viewer. It describes logical dataflow and
resource contracts; runtime storage and device pointers live downstream.

## Build and dependency boundary

`src/orlgraph/CMakeLists.txt` creates:

```text
orlgraph
  graph IDs, logical types, graph IR, scheduling, validation,
  optimization, reflection

orlgraph_io
  optional RapidJSON-backed serialization and graph cache
```

`orlgraph` is C++20 and has only local include paths. `orlgraph_io` is created
when `find_package(RapidJSON CONFIG QUIET)` produces a `RapidJSON` target.
Serialization is therefore optional, while the core IR and tests can remain
independent of JSON/LLVM/CUDA.

Downstream layering:

```text
orlgraph
  -> orlcomp_analysis / orlcomp_graph_lowering
  -> orlexec graph execution
  -> orlrig graph resources/evaluation
  -> orlviewer graph editor/runtime
```

The root build currently configures `orlgraph` before the compiler and
executor. `orlgraph_io` is required by the viewer target.

## File inventory

| File | Responsibility |
| --- | --- |
| `CMakeLists.txt` | Core and optional RapidJSON targets |
| `orlgraph.hpp` | Aggregate public include |
| `graph_ids.hpp/.cpp` | Stable IDs and versions |
| `graph_types.hpp/.cpp` | Logical values, types, handles, domains, shapes |
| `graph_effects.hpp` | Effect aliases for downstream users |
| `graph_ir.hpp/.cpp` | Definitions, registry, instances, ports, connections, modules |
| `graph_schedule.hpp/.cpp` | Deterministic topological schedule |
| `graph_validation.hpp/.cpp` | Contract checking and diagnostics |
| `graph_optimizer.hpp/.cpp` | Graph-level constant specialization, flattening, CSE-like merging, DCE |
| `graph_reflection.hpp/.cpp` | Current graph reflection view |
| `graph_serialization.hpp/.cpp` | `.oro`, editable/staged JSON, hashes, graph cache |

`orlgraph.hpp` includes the public core headers except serialization. Code
that needs `.oro` or JSON includes `graph_serialization.hpp` and links
`orlgraph_io`.

## Identity and versioning

### `StableId`

`StableId` is the serialized graph identity. It stores a string, supports
ordering/equality, and has a deterministic `StableId::from(domain, name)`
constructor.

The generated identity retains readable seed text plus a version-independent
FNV-1a digest:

```text
<domain>:<name>:<16-digit-hex-hash>
```

The hash avoids platform-dependent `std::hash` behavior, while the readable
seed makes serialized IDs inspectable.

`StableId` is deliberately not the same as:

- `orlrig::ComponentId`;
- a packed joint/locator index;
- a runtime `HandleValue`;
- a host/device pointer.

### `Version`

`Version` stores major/minor/patch values and implements equality and
lexicographic ordering. It is carried by graph modules, node definitions,
conversions, and serialized documents.

Current limitations:

- registry lookup is primarily by stable ID or qualified name;
- node instances do not store the definition version they were authored
  against;
- no version negotiation or compatibility resolver is implemented;
- serialization checks document format/language/ABI versions, but not a
  general definition compatibility policy.

## Logical types

### Type kinds

`LogicalTypeKind` currently includes:

```text
Void
Bool
Int64
Float64
String
Vector
Point
Normal
Vec4
Quaternion
Matrix
Struct
Array
Buffer
Unknown
Handle
```

`LogicalType` contains:

- kind;
- name;
- lane count;
- fixed extent;
- optional element type for arrays/buffers;
- accepted handle leaves;
- open-handle flag.

Factory helpers exist for scalar/vector/matrix/struct/handle/array/buffer
types. `from_orl_name` maps compiler source names into graph logical types.

### Aggregate types

`LogicalType::Struct` stores a name only. The graph core also has
`StructField` and `StructType` value types, but there is no graph-owned
struct-schema registry connecting a named logical struct to its field list.

This means the current graph IR can carry a named `Joint` or `Weight` type and
validate name equality, while the detailed storage schema remains owned by
the compiler/rig ABI.

`Array` carries an element type and fixed extent. `Buffer` carries an element
type and no fixed runtime count; shape metadata supplies count information.

### Values and constants

`Value` is a variant of:

```text
monostate
bool
int64
double
string
vector<int64>
vector<double>
```

`ConstantValue` pairs a logical type with a `Value`.

There is no handle alternative in `Value`. Handles are graph type contracts,
not serializable constant values. Validation rejects handle constants and
handle collections.

## Typed handles

### Exact, union, and open handles

The graph type system supports:

- exact nominal handle: `LogicalType::handle("orlrig::joint_handle")`;
- closed union: named canonical union plus accepted exact leaves;
- open/universal handle: a handle with `open_handle = true`.

Handle canonical names must be nonempty and contain only the accepted
identifier/namespace characters. Exact leaves are sorted and deduplicated
when a union is constructed.

### Assignability

`is_assignable(source, destination)` implements the current direction:

- exact to identical exact: valid;
- exact to a union containing that exact leaf: valid;
- exact to open universal handle: valid;
- union to a destination union only when the source leaves are a subset;
- union/open universal to an exact destination: invalid;
- unrelated exact handles: invalid.

Handle equality/compatibility is separate from general numeric/logical type
conversion. The graph validator treats handles as scalar-only values.

### Validation restrictions

The graph validator rejects:

- handle ports with array or buffer cardinality;
- handles nested in arrays/buffers or other collections;
- invalid canonical handle names;
- handle constants;
- incompatible exact/union/open handle connections.

The runtime token that eventually represents a handle is created later:
`{type_id, slot}`. It is not serialized in a graph file. Scene binding resolves
stable scene identity to a current packed slot at dispatch time.

## Domains and shapes

### Domains

`DomainKind` includes:

```text
Constant
Rig
Joint
Vertex
Edge
Face
Instance
Buffer
Custom
```

`Domain` stores kind plus a custom name where applicable. Built-in helpers
create canonical `constant`, `rig`, `joint`, `vertex`, `edge`, `face`,
`instance`, and `buffer` domains.

Current compatibility behavior:

- equal domains are compatible;
- constant is treated as broadly compatible with nonconstant domains;
- two different nonconstant domains require an explicit conversion;
- custom domains compare by canonical name.

This is intentionally a simple contract. It is not a complete dependency
planner for domain transforms.

### Shapes

`Shape` stores zero or more `ShapeDimension`s. A dimension is either:

- a constant unsigned value;
- a symbolic name.

An empty dimension vector is scalar. `Shape::one("joint_count")` is the common
one-dimensional buffer shape.

Current shape compatibility is conservative but shallow:

- equal shapes pass;
- scalar shapes act as wildcards;
- rank must match for non-scalars;
- constant dimensions must match;
- symbolic dimensions need matching kind/rank, but distinct symbol names are
  not unified or solved.

The graph IR therefore carries shape annotations without implementing a full
symbolic shape algebra or bounds solver.

### Semantics and coordinate spaces

Ports and interface ports carry:

- semantic string;
- coordinate-space string.

Core validation compares semantic strings when both sides provide one. Core
validation does not currently compare coordinate-space strings. The viewer
scene-input catalog performs stricter metadata checks, so graph behavior can
be stricter at the application boundary than in the target-independent core.

## Graph IR object model

### Graph stages

Only two stages are defined:

```text
GraphStage::Solver
GraphStage::Deformer
```

`GraphStageMask` supports none, solver, deformer, and all. A
`NodeDefinition` declares its allowed stages; validation can be run for one
stage or without a stage restriction.

### Ports

`Port` contains:

- stable ID and display name;
- direction: input, output, or in/out;
- cardinality: scalar, array, or buffer;
- logical type;
- domain and shape;
- required/default information;
- semantic and coordinate-space tags;
- optional output adapter;
- access mode.

`Port::OutputAdapter` refers to:

- a conversion definition;
- a source port;
- optional writeback conversion;
- optional writeback source port.

`ParameterSpec` describes a named node parameter, compile-time flag,
logical type, default, and semantic.

### Resource effects

`ResourceEffect` declares:

- stable resource ID;
- access mode: read, write, read/write;
- observable flag.

Effects are independent of dataflow. A node can remain live because it writes
an observable resource even when its returned scalar is unused.

### Partial-evaluation footprints

`PartialEvaluationFootprint` stores:

- whether a footprint was declared;
- global/stateful flags;
- sparse-dispatch support;
- hierarchy propagation mode;
- typed handle effects;
- read/write resource IDs.

Each handle effect identifies:

- formal parameter stable ID;
- exact handle type;
- view type;
- view field;
- access mode.

The rig runtime resolves these logical effects to concrete components and
packed indices.

### Implementations

`ImplementationRef` supports:

```text
OrlFunction
Subgraph
Builtin
Runtime
```

It can store module/function names, subgraph ID, and runtime name. The graph
core does not execute any of these forms; the downstream lowerer/runtime
interprets them.

### Node definitions

`NodeDefinition` contains:

- stable ID and qualified name;
- hidden flag;
- allowed stage mask;
- metadata and version;
- implementation reference;
- typed inputs/outputs/parameters;
- resource effects;
- backend/application capabilities;
- inline policy;
- operation/category;
- purity and statefulness;
- provenance;
- optional partial footprint.

Definitions can be found by stable ID or qualified name. `definitions_for`
filters by stage and excludes hidden definitions according to the current
implementation.

### Node instances

`NodeInstance` contains:

- stable instance ID;
- referenced definition ID;
- display name;
- constant parameter values;
- provenance;
- inline policy override;
- parameter mappings.

It does not retain a definition-version snapshot. Registry lookup at
validation/lowering time supplies the current definition.

### Endpoints and connections

An `Endpoint` is one of:

```text
NodePort
GraphInput
GraphOutput
```

A `Connection` stores:

- source endpoint;
- destination endpoint;
- optional conversion ID;
- shape metadata;
- provenance;
- feedback flag.

`feedback = true` excludes the edge from ordinary DAG scheduling, subject to
validation rules.

### Parameters and mappings

`ParameterMapping` supports source kinds:

```text
Constant
GraphInput
NodeOutput
ResourceAttribute
Derived
State
```

It stores source endpoint, optional constant, resource attribute, conversion,
compile-time flag, and provenance.

Current validation is strongest for constant, graph-input, and node-output
mapping. `Derived` and `State` are representational categories without a
complete runtime state model.

### Interfaces and resources

`InterfacePort` describes a graph input/output:

- stable ID/name;
- direction;
- logical type;
- domain/shape;
- required/default;
- compile-time flag;
- application binding;
- semantic/coordinate-space tags.

`Resource` describes a logical external value:

- stable ID/name;
- logical type/domain/shape;
- access;
- observability;
- application binding.

The graph owns declarations, not the actual allocation or scene resource.

### `GraphModule`

`GraphModule` stores:

- module ID;
- version;
- language version (`orl-0`);
- logical ABI version (`orlgraph-1`);
- ordered maps of nodes, inputs, outputs, resources;
- connection vector.

Mutation methods:

```text
add_node
add_connection
add_input
add_output
add_resource
remove_input
remove_output
```

The current implementation has no general `remove_node`. Callers and
optimizer passes use mutable node/connection accessors directly.

Construction methods enforce basic nonempty IDs/duplicate checks, while
semantic endpoint/type/shape/domain checks are deferred to `validate`.

## `NodeRegistry`

`NodeRegistry` owns:

- `NodeDefinition` values keyed by `StableId`;
- `ConversionDefinition` values keyed by `StableId`;
- shared-pointer `GraphModule` subgraphs keyed by stable ID.

Registration rejects missing IDs/names and duplicate IDs. Lookup is available
by stable ID or qualified name for definitions/conversions.

The registry also answers:

- whether a definition is available in a stage;
- definitions available for a stage;
- conversion lookup;
- subgraph lookup.

Current ownership boundaries:

- editable graph JSON refers to definitions in an application-owned registry;
- `.oro` embeds node definitions/conversions;
- subgraphs can be registered in memory, but current `.oro` serialization does
  not serialize the registry's subgraph map;
- no version negotiation resolves multiple definitions with the same logical
  name.

## Graph construction invariants

`graph_ir.cpp` enforces basic invariants:

- IDs must be nonempty where required;
- maps reject duplicate node/interface/resource IDs;
- directions are checked for interface ports;
- node definitions/conversions need names and IDs;
- endpoints need nonempty owner/port fields;
- removing an input removes associated source connections and mappings;
- removing an output removes associated destination connections.

`add_connection` intentionally does not fully resolve owners or ports. This
keeps graph construction lightweight; `validate` is the semantic gate.

## Validation

### API and diagnostics

`ValidationResult` contains:

- `Diagnostic` vector;
- `ScheduleResult`.

Each diagnostic has:

- severity: error, warning, note;
- stable diagnostic code;
- message;
- node and port IDs;
- provenance.

`ValidationResult::ok()` requires no error diagnostics and a successful
topological schedule.

### Validation sequence

`validate` checks:

1. node definition presence;
2. selected-stage availability;
3. definition port IDs/names/directions;
4. handle type validity and scalar-only handle contracts;
5. parameter definitions/mappings;
6. graph interface shape/direction;
7. endpoint owner and port existence;
8. required inputs/outputs;
9. single-writer/input connection rules;
10. conversion and output-adapter contracts;
11. type, handle, semantic, shape, and domain compatibility;
12. feedback eligibility;
13. schedule/cycle validity.

### Type compatibility

Ports first compare cardinality and then `LogicalType` assignability. Handle
compatibility uses exact/union/open rules from `graph_types.cpp`. Non-handle
types generally require equal logical types unless an explicit conversion is
attached.

`Port::compatible_value` handles source/destination type and cardinality
contracts. The validator adds diagnostic context such as node/port IDs and
provenance.

### Resource validation gap

Node definitions can refer to resource IDs through effects, but the current
validator does not fully ensure every effect resource is declared in the
module. Runtime/evaluation planning has additional resource logic, so a graph
can pass core validation while still requiring downstream resolution.

### Other current gaps

Core validation does not yet provide:

- definition-version compatibility;
- graph-level struct field/schema validation;
- symbolic shape unification;
- coordinate-space comparison;
- complete validation of `Derived`/`State` mappings;
- backend capability negotiation;
- explicit iteration-region semantics;
- full state-slot validation.

## Scheduling

`topological_schedule` implements deterministic Kahn sorting:

1. initialize node dependency counts in ordered maps;
2. add edges only for ordinary node-to-node connections;
3. skip graph-input/output edges;
4. skip `feedback` connections;
5. choose ready IDs in lexical order;
6. emit stable order;
7. fail when not all nodes are emitted.

Normal cycles produce `"Graph contains a dependency cycle"`.

Feedback is not a complete iterative scheduler. It is a boolean edge flag
that removes one dependency from the ordinary DAG and is allowed only when
validation sees a stateful/feedback/solver-style node contract.

There are no graph-core objects for:

- iteration count;
- convergence condition;
- persistent state slot;
- feedback barrier;
- state invalidation;
- reduction/accumulation region.

The viewer and rig evaluation planner add additional stage/dependency rules
outside the core scheduler.

## Optimization

`GraphOptimizer::optimize` mutates the input `GraphModule` in place.
`OptimizationOptions` controls:

```text
specialize_constants
simplify_identity
eliminate_dead_nodes
eliminate_common_subgraphs
```

### Current pass sequence

The implementation:

1. validates the input;
2. specializes compile-time parameter mappings;
3. flattens registered subgraphs;
4. simplifies identity nodes;
5. merges pure equivalent nodes;
6. removes dead nodes while preserving graph outputs/effects/stateful nodes;
7. removes duplicate connections where applicable;
8. revalidates and reschedules.

### Constant specialization

Compile-time parameter mappings are copied into node
`parameter_values`. The optimizer removes mappings that were specialized.

### Identity simplification

Nodes whose definition operation is `"identity"` are bypassed by redirecting
incoming/outgoing connections. The node is removed when it no longer serves a
required contract.

### Common-node elimination

Pure nodes with matching:

- definition;
- inline/parameter constants;
- incoming endpoint signature;
- feedback status;

can be redirected to one representative. The current signature is not a
complete semantic identity: it does not comprehensively include all shape,
domain, semantic, coordinate-space, conversion, effect, or debug metadata.

### Dead-node elimination

The optimizer retains:

- graph-output ancestors;
- nodes with writes/effects;
- observable nodes;
- stateful nodes;
- nodes needed through remaining connections.

It removes nodes that are neither output-reachable nor effect roots. This is
why `ResourceEffect`/observable flags are part of the graph contract.

### Subgraph flattening

Registered subgraph nodes are expanded into the parent graph with prefixed
IDs. Interface connections are remapped to parent connections. Flattening is
iterative and stops after a finite safety limit (64 rounds).

Current flattening does not provide a complete source-to-optimized debug map
and is shallow around some nested resources, state mappings, defaults, and
interface semantics.

### Not yet implemented as graph passes

The current optimizer is not yet a full compiler optimization pipeline. It
does not provide general:

- automatic conversion insertion;
- constant expression folding;
- semantic simplification;
- policy-driven inlining;
- fusion;
- map/reduce region planning;
- feedback/iteration optimization;
- retained source-versus-optimized graph;
- replacement provenance/debug mapping;
- optimization profile/cache keys.

LLVM optimization happens later in `orlcomp`; it is separate from these graph
passes.

## Reflection

`reflect` returns:

- module ID/version;
- graph inputs/outputs;
- resources;
- reflected nodes;
- node definition effects;
- resolved ports with types/domains/shapes/semantics/access/adapters;
- node provenance;
- topological schedule.

It is a read-only projection of the graph plus registry definitions. It does
not currently expose:

- connections;
- parameter values/mappings;
- registry definitions/conversions;
- definition versions/capabilities;
- partial footprints;
- coordinate spaces;
- graph stages as a separate reflected field;
- feedback/iteration metadata;
- optimizer replacements/removed-node provenance;
- source-to-optimized debug maps;
- complete resource/effect provenance.

The viewer/editor obtains additional UI and connection information directly
from `GraphModule`; reflection is not the complete editor model.

## Serialization

### Common constants

```text
ORO magic: ORO
graph JSON magic: ORL_GRAPH
staged JSON magic: ORL_GRAPH_STAGES
format version: 1
language version: orl-0
logical ABI: orlgraph-1
```

The `.oro` magic constant is `ORO`; serialized documents use the current
versioned graph format.

### `.oro`

`.oro` is a compiled graph-object format, not ORL source, LLVM IR, PTX, or a
Vulkan resource file.

It stores:

- format/language/ABI metadata;
- module metadata;
- graph inputs/outputs/resources/nodes/connections;
- embedded node definitions;
- embedded conversion definitions;
- content hash.

The embedded registry makes `.oro` more self-contained than editable graph
JSON. Registered subgraphs are not currently serialized into `.oro`.

Deserialization checks JSON shape, magic, versions, required sections,
definitions/conversions, canonical content hash, and graph validity using the
embedded registry.

The serializer uses canonical compact JSON for hashing. It traverses ordered
maps deterministically and computes the hash with the hash field empty.

### Editable graph JSON

`ORL_GRAPH` stores one `GraphModule` and references definitions by stable ID.
It preserves:

- module/language/ABI/version;
- inputs and outputs;
- resources;
- node instances and parameter values/mappings;
- connections, feedback, conversions, shapes, provenance;
- bindings, semantics, and coordinate spaces;
- content hash.

It does not embed the caller's `NodeRegistry`. Loading code must provide the
registry and validate the module.

### Staged graph JSON

`ORL_GRAPH_STAGES` stores:

```text
stages.solver
stages.deformer
```

Each stage is a `GraphModule`. Definitions remain external. Format/language/
ABI/hash checks happen at deserialization; registry validation belongs to the
caller.

The viewer embeds this staged document as `project.graph`.

### Graph IR cache

`OrlGraphIrCache` stores complete `.oro` documents:

```text
<directory>/graph_ir/<content_hash>.oro
```

It caches graph IR plus embedded definitions/conversions. It does not cache
LLVM IR, CPU objects, PTX, cubin, or runtime device allocations.

This cache is separate from `OrlBinaryCache` in `orlcomp`/`orlexec`, which
caches backend artifacts from generated ORL source.

### Serialization limitations

Current serializer/format limitations:

- subgraph registry entries are not embedded in `.oro`;
- source/optimized graph pairs are not stored;
- schedule/reflection/debug-map artifacts are not stored as first-class data;
- definition-version compatibility is absent;
- enum/field range validation is not uniformly exhaustive;
- serialization does not always force full semantic validation before writing;
- editor layout is viewer/project data, not graph-core data.

## Compiler integration

### ORL import

`orlcomp::import_node_definitions` converts semantic function summaries into
graph definitions:

- exported functions only;
- scalar parameters become scalar ports;
- buffers become buffer ports with symbolic shape;
- `Joint`-style buffers can receive joint domain;
- metadata supplies stages, shapes, semantics, statefulness, and partial
  footprints;
- inferred parameter access becomes resource effects;
- handle view effects become typed partial-evaluation effects;
- results become an optional `result` output.

The graph registry is therefore populated from semantic analysis rather than
raw AST syntax.

### Graph lowering

`OrlGraphLowerer`:

1. validates the graph;
2. uses the deterministic schedule;
3. collects ORL modules;
4. emits private handle declarations with canonical identity overrides;
5. emits graph input parameters;
6. specializes generic handles from exact edges;
7. emits conversions, ORL function calls, runtime callback expressions, and
   writebacks;
8. emits graph output descriptors.

Current output ABI:

- one scalar integer output can be returned directly;
- float, buffer, and handle outputs must alias a graph input parameter;
- unsupported computed scalar outputs fail;
- more than one returned integer output fails.

`OrlGraphProgram::Compile` passes the generated source back through normal
`OrlProgram` parsing, semantic analysis, runtime reflection, and backend setup.

## Runtime/rig integration

### Standard resources

`orlrig::add_lbs_resources` creates logical resources:

```text
bind_positions
posed_positions
joints
inverse_binds
weights
```

It annotates domain/shape/access/binding metadata but does not own actual
buffers.

### Standard definitions

`register_rig_node_definitions` registers:

- `orlrig.stage.computed_joints`;
- `orlrig.deformer.lbs.capture_bind`;
- `orlrig.deformer.lbs.evaluate`;
- typed `find_joint`;
- typed `find_locator`;
- `find_mesh`;
- compiled public solver/constraint/auto-weight ORL nodes.

The standard `make_lbs_graph` instantiates capture and evaluate nodes, while
the broader registry is available to authored graphs.

### Evaluation plans

The rig runtime interprets graph effects and partial footprints into
`EvaluationPlan`:

- maps typed handle effects to stable component IDs when find-node names are
  constant;
- expands hierarchy ancestors/descendants;
- tracks resource/joint/controller/locator readers/writers;
- detects competing writers and dependency cycles;
- creates CPU level batches and CUDA dirty ranges;
- conservatively marks unresolved/stateful/global regions for full evaluation.

The graph core does not own this evaluation plan; it is a rig-runtime
projection of a validated graph.

## Tests

### Core graph tests

`tests/graph_ir_tests/graph_ir_tests.cpp` covers:

- typed connections and exact/union/open handles;
- malformed handle collections/constants;
- stage masks;
- interface removal cleanup;
- deterministic schedule/cycle failure;
- optimizer identity/CSE/DCE/subgraph flattening;
- reflection;
- `.oro` determinism/hash/import;
- conversions/adapters/writebacks;
- partial footprints;
- graph IR cache;
- editable and staged JSON.

### Analysis/import tests

`tests/analysis_tests/analysis_tests.cpp` covers graph import, stage metadata,
resource effects, typed handle views, transitive helper effects, unsafe handle
operations, `.oro` registration, and old partial-metadata rejection.

### Lowering/runtime tests

`tests/graph_lowering_tests/graph_lowering_tests.cpp` covers generated ORL
source, CPU graph execution, graph input ABI checks, aliases, typed scene
handles, typed IK writes, union specialization, and conditional CUDA parity.

### Rig/viewer graph tests

- `rig_graph_tests` checks standard resources, registration, typed scene
  inputs, exact incompatibility, and typed IK effects.
- `orlrig_tests` checks ABI, hierarchy, runners, evaluation plans, conflicts,
  cycles, and fallback.
- `scene_graph_tests` checks staged runtime, scene bindings, device arenas,
  hierarchy revisions, host writeback, and stable handles.

The test CMake primarily creates Catch2 executables. No visible root
`add_test()` registration is present.

## Current design gaps and proposal drift

| Area | Current source | Intended/older design |
| --- | --- | --- |
| Graph type schemas | Named structs without graph-owned fields | Field/type declaration registry |
| Versioning | Version fields and format checks | Definition compatibility/resolution |
| Shapes | Rank/basic symbol checks | Symbolic expression/unification |
| Coordinates | Stored; core validation largely ignores them | Explicit coordinate contracts |
| Conversions | Explicit user/definition adapters | Safe automatic insertion |
| Optimization | Specialization, flattening, identity, CSE-like merge, DCE | Folding, inlining, fusion, iteration-aware passes |
| Feedback | Boolean edge and eligibility rule | Explicit iteration/state/convergence regions |
| Reflection | Interfaces/resources/nodes/ports/effects/provenance/schedule | Connections, mappings, capabilities, footprints, debug maps |
| Serialization | `.oro`, editable/staged JSON, hashes | Rich source/optimized/cache/debug package |
| Subgraphs | In-memory registry | Complete serialized subgraph ownership |
| Partial evaluation | Rig-side dirty plans and sparse fallback | Per-node generations/output caches/evaluation islands |
| Registry catalog | Current source skips several solver stems | Older `node_refs.md` lists them as registered |

Important source/test discrepancies:

- `node_refs.md` can be stale about skipped `hd_id`, `spline_ik`, and
  `full_body_ik`;
- graph core validation is weaker than viewer scene-input compatibility checks;
- feedback scheduling and rig evaluation dependency construction should be
  audited together;
- project files intentionally omit definitions and scene assets.

## Recommended reading order

1. `graph_types.hpp/.cpp` for values, handles, domains, and shapes.
2. `graph_ir.hpp/.cpp` for definitions, instances, ports, registry, and module.
3. `graph_validation.cpp` for actual graph contracts.
4. `graph_schedule.cpp` for deterministic ordering/feedback behavior.
5. `graph_optimizer.cpp` for mutation and effect-aware elimination.
6. `graph_reflection.cpp` for the current reflection boundary.
7. `graph_serialization.hpp/.cpp` for formats, hashes, and cache ownership.
8. `tests/graph_ir_tests` for supported behavior and failure cases.
9. `orlcomp/orl_graph_import.cpp` and `orl_graph_lowering.cpp` for compiler
   integration.
10. `orlexec/orl_graph_exec.cpp` and `orlrig/evaluation.cpp` for runtime
    interpretation.
11. `orlviewer/scene_graph_context.cpp` and `graph_scene_runtime.cpp` for
    scene-stage execution.
