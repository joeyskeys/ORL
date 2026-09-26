# ORL Graph IR (`.oro`) design

Status: design proposal

This document defines a target-independent intermediate representation for
graph-authored ORL programs. The intended user model is similar to OSL shader
groups and Houdini APEX: users assemble typed nodes and connections, while the
system analyzes, simplifies, specializes, and lowers the graph to executable
ORL code.

The `.oro` extension means an ORL graph object. It is the serialized form of
the graph IR and its reflection/provenance data. It is not ORL source, LLVM
IR, PTX, or a Vulkan resource file.

The IR is node-centric, graph-aware, and geometry-aware. "Geometry-aware"
means that it can describe domains such as joints, vertices, edges, faces,
and instances without depending on a renderer or a graphics API.

## 1. Goals

The graph IR must:

1. Represent node definitions, node instances, typed ports, parameters,
   connections, graph inputs, and graph outputs.
2. Preserve enough identity and provenance to report errors against the
   original graph and ORL source.
3. Support AST analysis, semantic type resolution, constant propagation,
   parameter mapping, graph flattening, inlining, and output-driven dead-code
   elimination.
4. Express data domains and shapes without exposing CUDA, Vulkan, LLVM, or
   host pointers.
5. Lower the same graph to CPU and GPU implementations.
6. Provide deterministic serialization, versioning, reflection, and cache
   keys.
7. Represent pure computation and observable side effects distinctly so that
   optimization cannot change rig behavior.
8. Keep rig data separate from rendering data. A joint, controller transform,
   mesh attribute, or weight buffer is an input/output resource; it is not a
   viewer object.

## 2. Non-goals

The first version should not:

- introduce a general-purpose bytecode virtual machine;
- encode LLVM types, LLVM instructions, CUDA address spaces, thread IDs,
  Vulkan handles, or TBB objects;
- replace the ORL procedural language;
- make rendering or window creation part of graph evaluation;
- require every ORL statement to become an individual graph node;
- define a renderer material or shading-closure system;
- make process-local component IDs persistent asset IDs.

The graph compiler may use LLVM as a backend. LLVM is not the graph IR.

## 3. Current ORL state

The current compiler is a procedural source compiler:

```text
.orl source
    -> textual `use` expansion
    -> handwritten lexer/parser
    -> procedural AST
    -> direct LLVM IR
    -> LLVM optimization pipeline
    -> native JIT or CUDA/PTX lowering
```

The main implementation points are:

- [`orl_ast.h`](src/orlcomp/orl_ast.h) contains expressions, statements,
  functions, structs, arrays, and buffers. It has no graph node, port, edge,
  resource, or graph-output representation.
- [`orl_preprocessor.cpp`](src/orlcomp/orl_preprocessor.cpp) implements
  `use` as textual source expansion. It detects circular inclusion and
  suppresses duplicate files, but it is not semantic module linking.
- [`orl_codegen.cpp`](src/orlcomp/orl_codegen.cpp) walks the AST directly
  into an LLVM module. There is no persistent typed semantic IR between the
  AST and LLVM.
- [`orl_optimizer.cpp`](src/orlcomp/orl_optimizer.cpp) runs LLVM's default
  per-module optimization pipeline at O1/O2/O3. There are no ORL graph
  optimization passes.
- [`orl_exec.hpp`](src/orlexec/orl_exec.hpp) exposes source compilation,
  named buffer/int/float bindings, CPU execution, CUDA execution, and device
  buffer views. It does not expose a graph execution plan or a general
  reflected value model.
- [`component_store.hpp`](src/orlexec/orlrig/component_store.hpp) stores
  rigging data such as joints, controllers, constraints, deformers, and
  weights. It is a data registry, not an executable graph.
- [`runners.hpp`](src/orlexec/orlrig/runners.hpp) and
  [`runners.cpp`](src/orlexec/orlrig/runners.cpp) manually select and execute
  ORL stdlib programs for LBS, auto-weighting, FK, and IK.
- [`resource/stdlib`](resource/stdlib) contains useful ORL implementations,
  but those functions currently have no graph-node metadata or port
  reflection.

The existing ORL language is a good implementation language for node bodies.
It is not yet a graph authoring or graph optimization language.

### 3.1 Current language types and ABI

The current compiler has logical types including integers, floating-point
values, vectors, points, quaternions, matrices, structs, fixed arrays, and
buffer parameters. The rigging runtime also defines stable packed layouts,
including:

- `Joint`: 128 bytes;
- `Weight`: 16 bytes;
- `point`: four-double storage slot;
- `matrix`: sixteen-double storage slot.

These layouts are valuable backend ABI information. They must not become the
definition of the `.oro` logical type system. The graph IR should describe a
logical `Joint`, `Weight`, `point`, or `matrix`; a backend lowering step maps
that type to a host or device layout.

The current runtime reflection is narrower than the language. It primarily
supports buffers, `int`, and `float` parameters with hard-coded strides. A
graph IR must provide reflection for arbitrary supported node ports, structs,
attributes, arrays, shapes, effects, and execution domains.

### 3.2 Current optimization boundary

LLVM can perform backend-level constant folding, CFG simplification, generic
dead instruction elimination, and function inlining after ORL has been lowered.
Those passes do not know:

- which graph outputs are observable;
- which nodes are pure;
- which graph parameters are compile-time constants;
- whether a rig resource is read or written;
- whether two nodes are the same reusable subgraph;
- which node or port produced a machine instruction.

Therefore `.oro` needs graph-level optimization before backend lowering.

## 4. Compilation architecture

The proposed pipeline is:

```text
ORL source modules       Graph authoring data
        |                         |
        v                         v
  ORL AST + AST analysis    Graph import and node binding
        |                         |
        +------------+------------+
                     v
             Typed semantic graph
                     |
       normalize, map parameters, validate
                     |
       flatten, inline, simplify, prune, schedule
                     |
              optimized `.oro` graph
                     |
          target-independent lowering
                /              \
               v                v
       ORL executable plan   other backend plan
               |                |
               v                v
          CPU LLVM/JIT       GPU LLVM/PTX
```

The graph compiler should be able to consume:

1. A graph node whose implementation is an ORL function.
2. A built-in node implemented directly by the graph compiler.
3. A host/runtime node implemented by a controlled external adapter.

All three forms must expose the same node metadata and effect model to the
optimizer.

## 5. IR layers

The implementation should keep three related but distinct representations.

### 5.1 ORL AST

The AST is syntax-oriented and should remain useful for parsing, source
diagnostics, and code generation. It should not become the serialized graph
format because it currently:

- owns syntax through C++ object pointers;
- has no stable IDs for expressions or declarations;
- has no resolved types or symbols;
- has no effect/resource summaries;
- has no graph connections;
- has no normalized constant representation;
- does not preserve source mapping after textual `use` expansion.

### 5.2 Semantic analysis model

AST analysis produces a typed module summary used to import ORL functions as
node definitions. The summary should include:

- resolved symbols and overload-independent function signatures;
- logical parameter and return types;
- struct layouts at the logical level;
- constant values and constant expressions;
- call graph information;
- purity and observable-effect summaries;
- buffer/resource reads and writes;
- execution-domain requirements;
- source spans and include provenance;
- whether a function is safe to inline;
- whether a function contains a structured loop or parallel region.

This layer is also where unsupported language constructs must be rejected
explicitly. An unknown function should not silently become an unresolved
external call in an optimizable graph.

### 5.3 Graph IR

The graph IR is a typed dataflow/control representation. It contains node
instances and explicit edges, not parser-specific C++ objects. It can retain
structured regions for loops, reductions, and solver operations that should
not be flattened into scalar nodes.

The optimizer operates on this layer. Backend lowering consumes the optimized
graph and produces ORL/LLVM-specific structures.

## 6. Core graph model

### 6.1 Graph module

A `GraphModule` is the root object in an `.oro` file. It contains:

- module identity and version;
- required ORL language and ABI versions;
- logical type declarations;
- node definition references;
- graph interface inputs and outputs;
- node instances;
- connections;
- resource declarations;
- optimization/provenance metadata;
- optional source graph data for editor round-tripping.

A module may contain nested subgraphs. A subgraph has an explicit interface
and is not allowed to capture undeclared resources.

### 6.2 Node definition

A `NodeDefinition` describes a reusable operation. It is analogous to a
shader type or layer implementation in an OSL-style system.

Required fields:

- stable definition ID;
- qualified name;
- semantic version;
- implementation kind;
- input and output port declarations;
- parameter declarations;
- domain and shape rules;
- purity/effect summary;
- supported backend capabilities;
- inline and specialization policy;
- source/provenance reference.

Examples of implementation kinds:

- `orl.function`: an analyzed ORL function;
- `orl.subgraph`: another graph module;
- `builtin`: a target-independent graph operation;
- `runtime`: a controlled external operation with declared effects.

The definition ID must not be a process-local pointer or an incrementing
runtime component ID.

### 6.3 Node instance

A `NodeInstance` is a use of a definition inside a graph. It contains:

- stable instance ID;
- definition reference and version;
- user-facing name;
- constant parameter values;
- parameter mappings;
- optional instance metadata;
- source graph location;
- optimization policy overrides.

Instance names are for diagnostics and tools. Optimization must use stable
IDs and resolved port references.

### 6.4 Ports

Each port has:

- stable port ID and display name;
- direction: input, output, or in/out resource;
- logical value type;
- cardinality: scalar, array, buffer, or stream-like extent;
- domain/rate;
- shape expression;
- required/optional status;
- default value or default-producing node;
- conversion policy;
- semantic tags such as `position`, `normal`, `joint_transform`, or
  `inverse_bind`.

Ports are named for authoring and reflection, but serialized edges should
refer to stable port IDs plus definition versions.

### 6.5 Connections

A connection is directed:

```text
source node output port -> destination node input port
```

It may also connect a graph input to a node input, or a node output to a graph
output. A connection records:

- source and destination IDs;
- optional field path or component selection;
- explicit conversion operation;
- shape/broadcast rule;
- source provenance.

Implicit conversion should be limited to declared safe conversions. A graph
optimizer must not silently reinterpret a matrix layout, coordinate space, or
buffer element stride.

### 6.6 Graph interface

Graph inputs and outputs are first-class ports. An interface port includes:

- logical type;
- domain and extent;
- default value;
- runtime/compile-time classification;
- resource binding name;
- semantic and coordinate-space metadata.

The set of graph outputs defines the initial liveness roots for dead-code
elimination. Declared side-effect resources are additional roots.

## 7. Logical type, domain, and shape system

The graph IR needs a logical type system above the current physical buffer
ABI.

### 7.1 Logical types

The initial type registry should cover the current language and rigging needs:

- `bool`;
- signed and unsigned integer types;
- scalar floating-point types;
- `vec2`, `vec3`, `vec4`;
- `point`, `vector`, and `normal`;
- `quat`;
- `mat4`;
- named structs;
- fixed arrays;
- bounded or runtime-sized buffers.

Named types must carry field names and logical field types. Physical alignment,
padding, and address space belong to backend layout descriptions.

### 7.2 Domains

A domain describes the set of logical elements over which a node operates.
The initial domain vocabulary should include:

- `constant`: one graph value;
- `rig`: one value for the complete rig;
- `joint`: one value per joint;
- `vertex`: one value per mesh vertex;
- `edge` and `face`: topology domains;
- `instance`: one value per instance;
- `buffer`: an explicitly sized linear resource.

Node definitions state the relationship between input and output domains.
For example, an LBS node may consume a `vertex` position buffer, a `joint`
pose buffer, and a vertex-major influence buffer, then produce a `vertex`
position buffer.

### 7.3 Shapes and extents

Shapes describe dimensions without exposing backend launch details. Examples:

- `scalar`;
- `array<N>`;
- `array<vertex_count>`;
- `matrix[ joint_count ]`;
- `weights[ vertex_count * influence_count ]`.

Shape expressions may reference graph inputs, resource metadata, or node
outputs. Bounds must be validated before backend lowering.

### 7.4 Coordinate spaces and semantics

Rig and geometry nodes need semantic metadata for:

- object, world, local, parent, and bind spaces;
- position versus direction vectors;
- row-major versus column-major logical matrix convention;
- unit and handedness conventions;
- interpolation/rate of mesh attributes.

These tags are part of graph validation and parameter mapping. They are not
rendering handles.

## 8. AST analysis and ORL node definitions

An ORL function can be imported as a graph node when its analyzed signature
and effect summary are available.

For example, the existing
`resource/stdlib/deformer/lbs.orl` functions can provide node definitions for:

- bind capture;
- LBS evaluation.

The current `LbsRunner` manually knows the source module and function names.
The graph system should replace that string-based knowledge with a node
definition registry and explicit port mappings.

### 8.1 Function import

For each eligible ORL function, analysis creates:

```text
ORL formal parameter -> node port
ORL return value     -> node output
ORL buffer access    -> resource/effect declaration
ORL struct type      -> logical graph type
ORL source span      -> provenance record
```

Functions with unsupported or unresolved parameters are rejected during node
registration rather than discovered at runtime.

### 8.2 Structured ORL bodies

Not every statement should become a graph node.

- Pure scalar/vector expressions may be represented as expression regions that
  can be simplified and inlined.
- `while` and `for` loops should remain structured regions unless a dedicated
  expansion pass can prove that flattening is safe.
- `parallel for` should become an abstract `map`/domain operation. The graph
  IR must not contain `global_id()`, CUDA block dimensions, or TBB calls.
- Reductions, atomics, and synchronization require explicit graph operation
  semantics before they can be reordered or fused.
- Solvers and iterative constraints should normally remain opaque structured
  nodes with declared reads, writes, and convergence behavior.

### 8.3 Purity and effects

Every node definition needs an effect summary:

- `pure`: output depends only on inputs and constants;
- `read(resource)`;
- `write(resource)`;
- `read_write(resource)`;
- `stateful`: depends on persistent evaluation state;
- `external`: effect cannot be modeled internally.

Pure nodes may be folded, duplicated, removed, or reordered subject to data
dependencies. Effectful nodes cannot be removed merely because their return
value is unused.

## 9. Parameter mapping

Parameter mapping is a primary responsibility of `.oro`. It converts graph
values and exposed controls into ORL function parameters and eventually into
backend bindings.

Each mapping should record:

- destination node and port;
- source kind;
- source reference;
- conversion;
- default and validation constraints;
- compile-time versus runtime status;
- resource/attribute binding;
- provenance.

Supported source kinds should include:

1. `constant`: stored directly in the graph;
2. `graph_input`: supplied by the application;
3. `node_output`: produced by another node;
4. `resource_attribute`: a field or buffer from rig/mesh data;
5. `derived`: generated by a declared conversion or adapter;
6. `state`: read from a persistent graph state slot.

The mapping must be logical. It should not store a host pointer, Vulkan buffer
handle, CUDA device pointer, or LLVM value.

At backend lowering time, the logical mapping becomes:

```text
graph input/resource
    -> packed ORL argument or buffer
    -> backend buffer/scalar binding
```

This permits the same `.oro` graph to bind CPU arrays, CUDA buffers, or a
future backend's resources.

## 10. Optimization pipeline

The graph optimizer should run before LLVM lowering. Each pass must declare
whether it preserves node identity, source mapping, and observable ordering.

### 10.1 Resolve and normalize

- resolve node definitions and versions;
- resolve logical types and field paths;
- canonicalize port and parameter references;
- insert explicit conversions;
- normalize constant encodings;
- validate domains, shapes, coordinate spaces, and resource declarations;
- produce deterministic node and edge ordering.

### 10.2 Graph flattening

Flatten nested graph instances into the containing graph when the subgraph
does not need to remain an execution boundary.

Flattening must:

- remap internal IDs without collisions;
- map subgraph inputs/outputs to outer ports;
- preserve source/provenance paths;
- preserve effect and state boundaries;
- retain a debug link to the original subgraph instance.

An opaque boundary may be retained when a node is stateful, externally
implemented, or explicitly marked as non-flattenable.

### 10.3 Parameter specialization

Parameters marked `compile_time` may be substituted before simplification.
Runtime parameters must remain graph inputs or resource reads.

Specialization keys should include:

- graph/module content hash;
- node definition versions;
- compile-time parameter values;
- logical ABI version;
- optimization profile.

Runtime animation values, joint poses, and mesh data must not accidentally
become specialization constants.

### 10.4 Constant folding and simplification

Apply graph-level simplification to:

- constant-only nodes;
- identity transforms;
- zero/one arithmetic;
- dead branches with known conditions;
- redundant conversions;
- known empty or zero-sized domains;
- field extraction and reconstruction;
- transform-space conversions with known identities.

The simplifier should use logical types and semantics, not physical byte
layouts.

### 10.5 Inlining

Inline a node or subgraph only when its definition allows it. The decision
should consider:

- purity;
- estimated cost;
- graph size growth;
- backend capability;
- debug/provenance requirements;
- resource and state effects.

Inlining is graph-level first. LLVM function inlining remains a later backend
optimization.

### 10.6 Output-driven dead-code elimination

Start from:

- graph outputs;
- writes to observable output resources;
- explicitly retained debug/profiling nodes;
- state updates and external effects.

Remove nodes that cannot reach a root and have no observable effect. This is
the graph equivalent of OSL shader-group DCE and cannot be implemented
reliably by the current LLVM pass alone.

### 10.7 Common-subgraph elimination

Pure equivalent nodes may be shared when:

- definitions and versions match;
- all inputs and compile-time parameters match;
- domains and semantics match;
- no state or effect is involved.

The pass must not merge two transforms merely because their physical bytes
happen to match if their coordinate-space semantics differ.

### 10.8 Scheduling and fusion planning

After optimization, derive a deterministic schedule:

- topological order for acyclic dataflow;
- explicit stages for barriers and resource writes;
- map/reduce regions for data domains;
- solver/feedback regions for permitted cycles;
- optional abstract fusion groups.

Fusion groups are target-independent execution regions. The CPU and GPU
lowerers decide whether they become a function, loop, or kernel.

## 11. Cycles, state, and solvers

A plain dataflow graph should reject cycles. A rig system still needs
controlled iteration for IK, constraints, feedback, and animation state.

The IR should distinguish:

- accidental dependency cycle: validation error;
- explicit feedback edge: reads a previous evaluation value;
- iteration region: bounded or convergence-controlled loop;
- solver node: opaque operation with declared inputs, outputs, effects, and
  convergence metadata;
- persistent state resource: value retained across evaluations.

This prevents a generic optimizer from reordering or deleting a solver step
as if it were a pure arithmetic node.

## 12. `.oro` serialized format

The logical `.oro` file should contain a versioned header and deterministic
sections. The physical encoding may begin as canonical JSON for inspection and
tests, then gain a compact binary encoding without changing the logical model.

Required top-level information:

```text
header
  magic = "ORO"
  format_version
  orl_language_version
  logical_abi_version
  module_id
  content_hash

types
node_definitions
graph_interface
nodes
connections
resources
optimization_metadata
provenance
```

An optimized `.oro` may additionally contain:

```text
source_graph        optional editor-preserving graph
optimized_graph
schedule
parameter_maps
reflection
debug_map
```

Target-specific code must not be required to load or validate the graph IR.
Optional backend caches may be stored separately and referenced by content
hash; they must never change the meaning of the `.oro` graph.

### 12.1 Determinism

Serialization must be deterministic:

- stable ordering of definitions, nodes, ports, edges, and fields;
- canonical numeric encoding;
- no memory addresses or process-local IDs;
- explicit version fields;
- content hash over canonical logical content.

The same graph and specialization inputs should produce the same `.oro`
content independent of editor insertion order.

### 12.2 Compatibility

The loader must distinguish:

- format incompatibility;
- missing node definition;
- incompatible node definition version;
- logical ABI mismatch;
- unsupported backend capability.

Node definitions should support explicit version compatibility instead of
silently loading a newer implementation.

## 13. Reflection, diagnostics, and debugging

Graph reflection should expose:

- graph interface ports;
- node instances and definitions;
- port types/domains/shapes;
- parameter maps;
- resource read/write sets;
- optimized-away nodes and their replacement;
- schedule stages;
- backend capability requirements.

Diagnostics should be structured rather than only strings. A diagnostic
should have:

- severity;
- stable code;
- message;
- graph/node/port IDs;
- ORL source file and range when available;
- related node/port notes;
- suggested fix where practical.

Every optimized node should retain a provenance set pointing to the original
node(s), ORL function, source range, and graph instance path. This is necessary
for an OSL-like graph debugger and for explaining why a node was removed or
merged.

## 14. Rigging and rendering separation

The graph IR should reference renderer-independent resources:

- joint transforms and hierarchy;
- controller transforms;
- mesh positions and topology;
- weights and influence indices;
- inverse bind matrices;
- animation values;
- solver targets and poles.

It should not own:

- `vkkk::Context`;
- windows or swapchains;
- Vulkan meshes or descriptor sets;
- viewer curve display resources;
- selection UI state.

The existing `orlrig` types are suitable as the first runtime resource
adapters:

- `orlrig::Controller` remains an xform-only logical value;
- `orlrig::Joint` and `orlrig::Weight` provide packed ABI adapters;
- `MeshData` and `MeshCsrData` provide geometry inputs;
- current LBS, auto-weight, and solver runners can temporarily implement
  runtime node definitions.

The graph must not depend on the viewer's component manager or viewport
feature order. Evaluation order must come from graph dependencies and
declared effects.

## 15. Backend lowering

The optimized `.oro` graph is target-independent. A lowerer maps it to an
execution plan for a selected backend.

### 15.1 ORL/LLVM lowerer

The first lowerer can:

1. create one or more ORL entry functions;
2. map graph buffers and scalar parameters to ORL parameters;
3. emit structured ORL operations for map/reduce regions;
4. retain solver and external nodes as registered calls;
5. compile the resulting ORL module through the existing parser/codegen
   pipeline;
6. apply LLVM optimization only after graph optimization is complete.

This allows the graph system to reuse the current compiler while keeping
graph semantics outside LLVM.

### 15.2 CPU and GPU neutrality

The graph IR may express an abstract map over `vertex` or `joint` domains,
but must not express:

- CUDA `global_id`;
- block/grid dimensions;
- Vulkan storage buffers;
- CPU thread-pool handles;
- device pointers.

The CPU lowerer may emit a loop or a parallel runtime call. The GPU lowerer
may emit a kernel and dispatch plan. Both must implement the same logical
domain and buffer semantics.

The current end-to-end runtime supports CPU and CUDA. ROCm and other backends
should be represented as capabilities, not hard-coded into `.oro`.

## 16. Execution model

An `OrlGraphProgram`-style runtime should eventually separate:

1. load/validate `.oro`;
2. select a backend;
3. lower or load a backend artifact;
4. bind graph inputs and resources;
5. evaluate a graph instance;
6. inspect outputs, profiling, and diagnostics.

Compilation must not repeat when only runtime buffer contents or animation
values change.

Bindings should include logical element count and shape, not only byte size.
The current execution layer validates names and strides but does not provide
complete logical bounds validation. The graph runtime should validate that
node shape expressions are compatible with bound resources before dispatch.

## 17. Proposed implementation phases

### Phase 1: semantic function/node registry

- Add AST analysis separate from LLVM emission.
- Resolve types, functions, calls, and struct fields.
- Produce function summaries and source provenance.
- Import selected `resource/stdlib` functions as node definitions.
- Define logical types for current joint, weight, point, matrix, and buffer
  data.

### Phase 2: in-memory graph IR

- Implement module, node definition, node instance, port, edge, resource,
  domain, shape, and parameter-map objects.
- Add graph validation and deterministic topological scheduling.
- Add graph inputs/outputs and explicit conversion nodes.
- Keep this layer independent from `vkkk` and LLVM.

### Phase 3: graph optimization

- Implement flattening and subgraph expansion.
- Implement compile-time parameter specialization.
- Implement constant folding and semantic simplification.
- Implement purity-aware inlining and output-driven DCE.
- Preserve provenance for every transformation.

### Phase 4: `.oro` serialization

- Define canonical logical encoding.
- Add versioning, content hashes, reflection, and debug maps.
- Add round-trip and deterministic serialization tests.
- Support loading optimized graphs without ORL source.

### Phase 5: ORL/LLVM lowering

- Lower graph inputs, outputs, and parameter maps to ORL entry functions.
- Lower domain maps and resource bindings to the current execution API.
- Reuse CPU JIT and CUDA lowering.
- Add backend parity tests for the same `.oro` graph.

### Phase 6: rig graph integration

- Replace hard-coded runner sequencing with graph nodes and dependencies.
- Add explicit solver/feedback regions.
- Add dirty propagation and incremental evaluation.
- Bind graph resources to standalone `orlrig` data, not viewer objects.

## 18. Tests and acceptance criteria

The graph IR should have tests independent of the viewer:

1. Parse/import an ORL function and inspect its node definition.
2. Reject unresolved types, calls, incompatible ports, and invalid shapes.
3. Verify deterministic node/edge IDs and `.oro` round trips.
4. Flatten a nested graph while preserving provenance.
5. Fold constants and verify the optimized graph.
6. Remove an unused pure branch while retaining effectful nodes.
7. Inline a pure node and preserve graph outputs.
8. Map graph parameters to ORL function parameters by stable port identity.
9. Compile the same `.oro` to CPU and CUDA plans.
10. Compare CPU/GPU outputs for LBS, FK, constraints, and auto-weight cases.
11. Verify cycles are rejected unless represented by an explicit feedback or
    solver region.
12. Verify optimization is independent of node insertion order.

## 19. Relationship to OSL

The intended similarities to OSL are:

- reusable node/shader-like definitions;
- graph instances with named typed parameters;
- explicit connections;
- graph-level linking and flattening;
- output-driven optimization;
- a serialized compiled object format;
- late backend compilation.

The differences are intentional:

- ORL operates on rig and geometry data rather than shading closures;
- domains such as joints and vertices are first-class;
- mutable buffers and resource effects must be explicit;
- solver and feedback regions need defined iteration semantics;
- CPU and GPU execution must share a logical data model;
- viewer/rendering objects remain outside the graph IR.

`.oro` should therefore be inspired by the role of `.oso`, but it should not
be a copy of OSL bytecode. It is primarily a serialized, typed, optimized
graph representation with enough information to lower to multiple devices.

## 20. Decisions to make before implementation

The following choices should be resolved before freezing the schema:

- canonical JSON first, binary first, or both;
- whether `.oro` stores the editable source graph and optimized graph together;
- exact logical numeric precision rules;
- whether custom structs are versioned by name, structural hash, or both;
- domain/shape expression syntax;
- explicit conversion and coordinate-space rules;
- effect granularity for buffer fields and mesh attributes;
- policy for graph cycles and iterative solvers;
- node definition package/discovery mechanism;
- backend capability negotiation;
- whether an optimized graph is portable across ORL compiler versions;
- whether target-specific caches use a separate extension or external cache.

The core decision is stable: `.oro` is a target-independent graph object, while
ORL AST analysis and LLVM/PTX lowering are compiler stages around it.
