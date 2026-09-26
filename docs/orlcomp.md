# `src/orlcomp`: ORL compiler module

## Scope and current status

This document describes the implementation in `src/orlcomp` at commit
`4e0fb3d`. It covers the active handwritten frontend, semantic analysis,
graph import/lowering, LLVM backend, CPU JIT, CUDA backend, cache, CLI, and
the compiler-facing ABI. It also records the boundaries where runtime and
viewer code take over.

`orlcomp` is not one monolithic compiler target. Its CMake file deliberately
builds a dependency ladder:

```text
orlcomp_lexer
      |
      v
orlcomp_parser
      |
      v
orlcomp_analysis
      |
      v
orlcomp_graph_lowering
      |
      +--> orlcomp_codegen   (LLVM/CUDA/TBB dependent)
```

The first four targets are usable without LLVM. The last target is only
created when CMake finds LLVM, CUDA Toolkit, TBB, zlib, and zstd. `orlexec`,
`orlc`, and `orlrig` are consequently also conditional on `orlcomp_codegen`.

The module currently serves three related compilation paths:

```text
1. Direct source execution
   ORL text -> parser/analysis -> LLVM IR -> native JIT or CUDA/PTX

2. ORL node import
   ORL text -> parser/semantic summaries -> NodeDefinition/.oro

3. Graph execution
   GraphModule -> validation/schedule -> generated ORL source
              -> ordinary parser/analysis/runtime reflection
              -> LLVM IR -> CPU/CUDA execution
```

The frontend and graph import are language/compiler services. The executable
runtime is implemented in `src/orlexec`; the rig-specific handle view registry
is implemented in `src/orlexec/orlrig`.

## File inventory

| File | Responsibility |
| --- | --- |
| `CMakeLists.txt` | Layered libraries and optional LLVM/CUDA/TBB configuration |
| `orl_lexer.h/.cpp` | Token kinds, source-aware lexical scanning, diagnostics |
| `orl_ast.h` | Polymorphic expressions, statements, declarations, and program flags |
| `orl_parser.h/.cpp` | Handwritten recursive-descent parser and grammar state |
| `orl_preprocessor.h/.cpp` | Line-oriented textual `use` expansion and module markers |
| `orl_analysis.h/.cpp` | Semantic types, function summaries, effects, purity, diagnostics |
| `orl_graph_import.h/.cpp` | Exported-function to graph-node import and `.oro` bridge |
| `orl_graph_lowering.h/.cpp` | Typed graph-to-ORL source generation and output adapters |
| `orl_handle_views.hpp` | Generic handle-view descriptors and registry |
| `orl_runtime_signature.h` | Runtime parameter reflection and handle ABI definitions |
| `orl_codegen.h/.cpp` | AST-to-LLVM lowering for host and CUDA targets |
| `orl_intrinsics.h/.cpp` | Direct LLVM lowering for vector/matrix math built-ins |
| `orl_optimizer.h/.cpp` | LLVM O0–O3 module optimization |
| `orl_cache.h/.cpp` | Versioned binary artifact cache |
| `orl_jit.h/.cpp` | Native LLJIT and CPU object loading |
| `orl_gpu.h/.cpp` | LLVM device code generation and CUDA driver API |
| `orl_parallel_runtime.h/.cpp` | C ABI host parallel loop runtime backed by TBB |
| `orlc_main.cpp` | `orlc` command-line compiler/export tool |
| `orlgram.y`, `orllex.l` | Inactive legacy Flex/Bison experiments |
| `test.orl` | Inactive small language/codegen example |

`orl_runtime_signature.h` is header-only at present. The handle-view registry
is generic: it knows field descriptors and ABI metadata, but does not know
what a joint or locator means. Rig-specific storage and symbols are registered
by `orlrig::register_rig_handle_views`.

## Build structure and dependencies

`src/orlcomp/CMakeLists.txt` creates:

```text
orlcomp_lexer
  sources: orl_lexer.cpp

orlcomp_parser
  sources: orl_parser.cpp, orl_preprocessor.cpp
  public definition: ORL_STDLIB_DIR
  links: orlcomp_lexer

orlcomp_analysis
  sources: orl_analysis.cpp, orl_graph_import.cpp
  links: orlcomp_parser, orlgraph
  optionally links: orlgraph_io

orlcomp_graph_lowering
  sources: orl_graph_lowering.cpp
  links: orlcomp_analysis, orlgraph

orlcomp_codegen
  sources: LLVM codegen, intrinsics, optimizer, cache, JIT, GPU, TBB runtime
  links: orlcomp_parser, LLVM, CUDA runtime, TBB
```

The graph import target gets `ORL_HAS_GRAPH_IO=1` only when `orlgraph_io`
exists. `.oro` import/export then reports a deliberate diagnostic instead of
silently using a different path when RapidJSON is unavailable.

The CMake configuration adds NVPTX LLVM components only when `NVPTX` appears
in `LLVM_TARGETS_TO_BUILD`. CUDA Toolkit is still required to configure the
LLVM codegen target. The active compiler therefore has a stronger configure
dependency than the source/frontend layering alone suggests.

## End-to-end pipelines

### Direct ORL source execution

The public runtime path is `ORL::exec::OrlProgram::Compile` followed by
`ORL::exec::OrlExecution::Create`.

```text
source string
  -> Parser::Parse
       Preprocessor::Process
       Lexer::NextToken
       Parser AST
  -> SemanticAnalyzer::analyze
       type/effect/handle diagnostics
  -> DescribeRuntimeFunction
       entry return and parameter reflection
  -> OrlProgram
  -> OrlExecution::Create
       cache lookup
       Parser::Parse again on cache miss
       LlvmIrCodegen::Generate
       LlvmOptimizer::Optimize(O2)
       OrlJitEngine or OrlGpuEngine
```

The second parse is intentional in the current execution implementation:
`OrlProgram` retains source and compile options so a backend-specific
`OrlExecution` can generate its own module. Semantic analysis is now part of
`OrlProgram::Compile`; it is not merely an optional node-import pass.

The selected entry must return exact ORL `int`. The application-facing
wrapper ABI groups explicit parameters into buffer pointers, `int64_t` slots,
`double` slots, and, when needed, flattened two-lane handle values. Implicit
solver/hierarchy context pointers are appended after explicit parameters.

### Exported ORL function import

`import_node_definitions` follows a different terminal path:

```text
ORL source/file
  -> Parser
  -> SemanticAnalyzer
  -> retain only exported functions
  -> NodeDefinition per exported function
  -> optional stdlib stamping
  -> optional serialize_oro
```

Only functions with the `export` keyword become graph definitions. Helper
functions remain in the source module and are available to imported ORL node
implementations, but are not exposed as graph nodes.

### Graph lowering

`OrlGraphLowerer::lower` first validates the graph through `orlgraph`, obtains
the deterministic schedule, gathers `use` modules, emits private handle type
declarations, and writes a generated entry function. Normal ORL nodes become
direct function calls in schedule order. Runtime nodes and output adapters
are resolved through callbacks supplied by the application.

The generated source is passed back through `OrlProgram::Compile` by
`OrlGraphProgram::Compile`. Thus graph lowering deliberately reuses the
ordinary parser, semantic checker, runtime reflection, and backend ABI rather
than creating a separate graph-only backend.

### Backend selection

`LlvmIrCodegen` receives `OrlCodegenTarget::Host` or `OrlCodegenTarget::Cuda`.
Host output is loaded by `OrlJitEngine`, which is an LLJIT wrapper. CUDA output
is passed to `OrlGpuEngine`, which emits PTX, optionally links it to cubin at
runtime, loads the CUDA driver dynamically, and launches an `orl_cuda_entry`
kernel.

`OrlJitTarget::Cuda` and `OrlJitTarget::Rocm` exist as API enum values for
target selection, but `OrlJitEngine` itself executes only native host code.
Device execution uses `OrlGpuEngine`.

## Text preprocessing

### `use` expansion

`Preprocessor::ProcessText` recognizes line-oriented statements such as:

```orl
use joint;
use solver/ik_two_bone;
```

It appends `.orl`, searches include paths in order, canonicalizes the selected
file with `weakly_canonical`, recursively expands the file, and inserts
`#orl_module <module> <line>` markers around the expansion.

The preprocessor provides:

- active include-stack cycle detection;
- global duplicate-file suppression for one processing operation;
- configured include directories;
- the compiled-in `ORL_STDLIB_DIR`;
- source-origin/module markers consumed by the lexer.

It does not provide:

- namespaces;
- import aliases;
- qualified symbols;
- semantic module linking;
- a persistent module dependency object;
- a source map richer than the inserted module markers.

`Parser::Parse` adds the compiled stdlib path plus a few current-working
directory candidates. `orlc` and `import_node_definitions_file` prepend the
input file's parent directory to the include search path.

`use` statements are recognized by the preprocessor before normal lexing.
`#orl_module` is an internal reserved directive; user source containing one
is rejected by the preprocessor, while the lexer consumes generated markers
and updates token source origins and line numbers.

### Preprocessor state and caveat

`Preprocessor::Process` clears its own include stack, processed-file set, and
diagnostics. `ProcessText` appends to the caller-provided output string rather
than clearing it. Reusing one output string across independent processing
operations without clearing it can append stale expanded source. Normal
parser usage creates a fresh output string.

The include cache key used later by `orlexec` includes the top-level source
text and include-path strings, not the contents of every resolved module.
Changing a `use`d file at the same path therefore requires care when a binary
cache directory is enabled.

## Lexer

`Lexer` is a single-pass scanner with line, column, and `source_origin`
tracking. It produces `Token` objects with:

- `TokenKind`;
- original lexeme;
- line and column;
- source origin;
- parsed integer/float values where applicable;
- an error message for invalid tokens.

### Token categories

The active lexer recognizes:

- identifiers;
- integer literals, including hexadecimal;
- floating literals with decimal and exponent notation;
- quoted strings;
- arithmetic, comparison, assignment, logical, punctuation, brackets, and
  member-access tokens;
- comments beginning with `//`;
- backslash-newline continuations;
- parser-generated `#orl_module` directives.

Keywords include:

```text
if else parallel for while do break continue
or and not is
vector normal point matrix
int float string struct handle use return export
```

The lexer also classifies a broader GLSL-like vocabulary as `TypeName`,
including `bool`, `uint`, `double`, `quat`, `vec2/3/4`, `dvec2/3/4`,
`mat2/3/4`, and boolean/integer/unsigned matrix/vector spellings. The
parser/analyzer vocabulary is broader than the active LLVM type map; accepting
a spelling lexically does not guarantee backend support.

There are no dedicated `true` and `false` literal tokens. Boolean contexts
are formed by numeric values and codegen conversions.

Malformed hex literals, exponents, strings, unexpected characters, and a
single `&` are reported as invalid tokens. `&&` and `||` are separate tokens;
single `|` is used in handle union declarations.

## AST and parser

### AST ownership model

`orl_ast.h` defines a polymorphic `unique_ptr` tree. The main categories are:

#### Expressions

- identifier and literal;
- unary and binary operators;
- assignment;
- direct function call;
- index and index assignment;
- component/member access and member assignment;
- `HandleTestExpression` for `value is exact_handle`.

#### Statements

- block and expression statement;
- declaration and return;
- `if`;
- `while`, `do while`, and C-style `for`;
- restricted `ParallelForStatement`;
- `break` and `continue`.

#### Top-level declarations

- data-only `StructDefinitionStatement`;
- `HandleDefinitionStatement`;
- `FunctionDefinitionStatement`;
- `Parameter`, `StructField`, and literal metadata records.

`Program` contains top-level items plus two program-wide flags:

- `uses_solver_context`;
- `uses_hierarchy_context`.

The AST mostly has no source locations. Parser errors retain token positions;
handle declarations retain a source location so semantic diagnostics can
identify imported handle declarations. Most semantic summaries use a source
name with zero line/column because expression nodes do not carry positions.

### Top-level grammar behavior

`Parser::ParseTopLevel` recognizes:

1. `struct Name { fields }`;
2. exact handle declarations, `handle Name;`;
3. handle unions, `handle Name = LeafA | LeafB;`;
4. exported functions beginning with `export`;
5. non-exported function definitions;
6. top-level statements accepted by the implementation.

Structs and handles are declaration-before-use. A name cannot be both a
struct and a handle. `SolverContext` and `HierarchyContext` are reserved
struct names. Handle unions can contain only previously declared exact handle
types; nested unions are flattened and sorted.

Exact handle canonical identity is derived from the source/module origin:

- `rig/handles` is normalized to the `orlrig::` namespace;
- other slash-separated module names become C++-style `::` names;
- the local source spelling remains the type used by the generated ORL.

### Functions and metadata

Functions have:

- return type and name;
- scalar or buffer parameters (`Type name[]`);
- optional export marker;
- optional `[[ ... ]]` metadata;
- a block body.

Metadata is only legal on exported functions. Each entry has a simple type,
name, and literal value. Semantic analysis accepts `int`, `float`, or
`string` metadata values and converts them to `orlgraph::ConstantValue`.

Common stdlib metadata includes:

```orl
string stage = "solver";
string partial_propagation = "descendants";
int partial_global = 1;
string partial_sparse = "1";
string stateful = "1";
```

The parser only checks the syntactic metadata shape. Semantic analysis and
node import interpret the keys and reject invalid values.

### Statements and expressions

Expression precedence is encoded by the recursive-descent call chain:

```text
assignment
  logical-or
    logical-and
      equality / `is`
        comparison
          term
            factor
              unary
                postfix (call, index, member)
                  primary
```

The parser accepts:

- positional constructors;
- fixed arrays such as `Type values[16];`;
- buffer parameters such as `Type values[]`;
- struct fields and vector-style `.x/.y/.z/.w` access;
- assignments to names, indexes, and member expressions;
- direct function calls only;
- the control-flow forms listed above.

The parser allows some forms that later codegen may reject, including
assignments to vector component expressions.

### Restricted parallel loop

The only accepted parallel-loop header is:

```orl
parallel for (int i = 0; i < bound; i = i + 1) {
    ...
}
```

The parser requires:

- an `int` index;
- literal initial value `0`;
- the index on the left of `<`;
- an identifier bound;
- exact increment `i = i + 1`;
- a braced body.

The bound restriction is useful for backend lowering and graph import, but it
is a language-level limitation rather than a general parallel-loop design.

### Implicit context detection

When the parser sees identifiers named `solver_context`, `hierarchy_context`,
or `hierarchy_data`, it sets the corresponding `Program` flag. This is a
program-wide lexical use flag, not a per-function data-flow result. Codegen
therefore adds the hidden context ABI to every generated function in the
program when one function requires it.

The parser rejects declarations, parameters, functions, and assignments that
shadow or write the reserved names at several syntactic sites. Semantic
analysis repeats the important checks.

## Semantic analysis

### Public model

`orl_analysis.h` exposes:

- `ParameterAccess`: none, read, write, or read/write bit combination;
- `SemanticTypeKind`: unknown, builtin, struct, universal handle, nominal
  handle, or handle union;
- `SemanticType`: local name, canonical handle name, accepted leaves, and
  open-handle flag;
- `HandleViewEffectSummary`;
- `HandleCallSiteSummary`;
- `FunctionParameterSummary`;
- `FunctionSummary`;
- `AnalysisDiagnostic`;
- `AnalysisResult`;
- `SemanticAnalyzer`.

For each function, `FunctionSummary` records:

- return and parameter logical/resolved types;
- calls;
- parsed metadata;
- source provenance;
- export status;
- purity and statefulness;
- loop and parallel-loop presence;
- external/unknown-call presence;
- direct and transitively propagated handle-view effects.

### Type vocabulary

`is_supported_orl_type` currently recognizes:

```text
bool int int64 float double float64 string
vector normal point vec3 dvec3 vec4 dvec4
quat matrix mat4 dmat4
```

`is_known_orl_builtin` recognizes:

```text
global_id print dot cross length normalize
clamp lerp mat_identity mat_transpose mat_inverse mat_mul
quat_mul quat_conjugate quat_normalize quat_rotate min max
```

This list is an analyzer contract, not a complete proof that the LLVM
backend has a native mapping for every spelling or call. In particular,
`MapTypeName` in `orl_codegen.cpp` has a narrower active type map, and `min`
and `max` do not have dedicated `OrlIntrinsicCodegen` implementations.

### Function analysis

`FunctionAnalyzer` performs a conservative body walk:

1. resolve return and parameter types;
2. validate metadata;
3. create variable and parameter maps;
4. visit declarations, statements, and expressions;
5. record parameter reads/writes;
6. record direct calls and handle argument provenance;
7. record loops and parallel loops;
8. derive initial purity;
9. after all functions are summarized, propagate view effects and impurity
   through internal calls to a fixpoint.

The initial purity rule is:

```text
pure =
    no unknown/external call
    && not stateful
    && no parallel for
    && no parameter writes
```

An imported node also becomes non-pure when stdlib stamping requires an
observable runtime operation, even if the lower-level local function summary
was otherwise conservative.

Parameter effects are inferred from identifier bases, indexed buffers, and
struct/member assignments. Direct writes to the implicit context globals are
diagnosed as errors.

### Handle types and assignability

The analyzer distinguishes:

- **universal handle**: open `handle` type;
- **nominal handle**: one exact canonical identity;
- **handle union**: a named finite set of exact leaves;
- **open handle**: an explicitly open graph/compiler handle representation.

Compatibility rules include:

- a universal expected handle accepts any handle;
- a union accepts an exact leaf in its set;
- a union accepts another union only when the source leaves are included;
- exact handles require equal canonical identity;
- non-handle implicit conversions are not used to make handles compatible.

Handles cannot be:

- arrays or buffer elements;
- indexed;
- used with arithmetic or unary operators;
- directly member-accessed;
- implicitly converted to integers, structs, or other handles.

Handle equality and inequality are supported when the types are compatible.
The `is` operator requires a handle on the left and an exact nominal handle
on the right.

The analyzer performs flow narrowing for `if (value is exact_handle)`:

- the true branch is treated as the tested exact type;
- for a union, the false branch removes the tested leaf;
- if only one leaf remains, the false branch narrows to that exact type;
- if the source is already an exact type, the unreachable branch is not
  visited.

### Handle-backed view analysis

The source-level view pattern is:

```orl
WorldTransform world = WorldTransform(joint);
world.xform = matrix_value;
```

The analyzer recognizes this only when a registered
`HandleViewDescriptor` exists for the exact source handle and destination
struct. It records field reads/writes against the original handle parameter,
not against the temporary view variable.

Whole-view escape is rejected. A view temporary cannot be passed around as a
normal aggregate or used as a whole value. Unknown view fields and unsupported
handle conversions are diagnostics.

`HandleCallSiteSummary` records which caller handle parameter supplied each
callee handle argument. After local analysis, `SemanticAnalyzer::analyze`
propagates callee view effects back through these mappings until no effect
changes. This is what lets a helper such as `constraint_handle_write` affect
the graph footprint of an exported constraint.

### Diagnostics

Diagnostics include explicit codes for:

- duplicate or colliding types/functions;
- unsupported types;
- unknown or indirect calls;
- invalid metadata;
- reserved context-name use;
- context writes;
- handle arrays/indexing/member access;
- handle equality/test mismatches;
- unsupported conversions;
- handle-view escape and unknown fields;
- incompatible function arguments and returns.

The direct runtime path now runs these semantic diagnostics before backend
codegen. The graph import path also treats them as a prerequisite for
creating definitions.

## Handle representation and view ABI

### Generic handle value

`orl_runtime_signature.h` defines:

```cpp
struct HandleValue {
    std::uint64_t type_id;
    std::int64_t slot;
};
```

The struct is standard-layout, 16 bytes, and two 64-bit lanes. Type IDs are
versioned FNV-1a hashes of:

```text
orl-handle-v1:<canonical-name>
```

`HandleTypeRegistry` detects hash collisions by retaining canonical names.
`DescribeRuntimeFunction` reflects exact handle parameters as
`OrlRuntimeParameterKind::Handle`, canonical type name, and type ID.

Universal/open handle parameters are graph/compiler concepts. The direct
application runtime rejects an unresolved universal `handle` entry parameter
because it cannot validate a concrete ABI type.

### Generic view descriptors

`orl_handle_views.hpp` defines:

- `HandleViewFieldKind`: `Int64`, `Float64`, `Vec4`, `Matrix`;
- `HandleViewFieldDescriptor`: field ID, source/target symbols, type name,
  byte offset, and byte size;
- `HandleViewDescriptor`: source handle, destination struct, ABI version,
  backend mask, storage identity, and fields;
- `HandleViewRegistry`: registration, lookup, field lookup, duplicate/collision
  checks, and a cache fingerprint.

The generic registry is initialized by the rig layer. Current rig views are:

| Source handle | Destination | Fields | Backend mask |
| --- | --- | --- | --- |
| `orlrig::joint_handle` | `Joint` | parent, selected, pad0, pad1, translation, rotation, scale | host + CUDA |
| `orlrig::locator_handle` | `Locator` | xform | host + CUDA |
| `orlrig::joint_handle` | `WorldTransform` | xform | host + CUDA |

### CPU view lowering

For host codegen, a view field is lowered to an external C ABI symbol. The
JIT registers the rig symbols:

```text
__orlrig_joint_read_i64
__orlrig_joint_write_i64
__orlrig_joint_read_vec4
__orlrig_joint_write_vec4
__orlrig_locator_read_matrix
__orlrig_locator_write_matrix
__orlrig_world_read_matrix
__orlrig_world_write_matrix
```

The runtime wraps evaluation in `ScopedHandleViewContext`, which makes the
current storage context available through a thread-local pointer. The rig
implementation validates nominal type, slot range, storage stride, topology
revision, and writable state.

`WorldTransform` reads and writes compute or decompose a joint world matrix
through the host hierarchy helpers rather than treating the matrix as a raw
joint field.

### CUDA view lowering

For CUDA codegen, a hidden `__orl_handle_context` pointer is appended to
functions that use registered handle views. The codegen:

1. obtains an arena base address and stride from `HandleDeviceContext`;
2. multiplies the handle slot by the registered stride;
3. adds the field byte offset;
4. emits packed loads/stores for integer, vec4, or matrix fields;
5. uses device `joint_world_matrix` / `joint_write_world_matrix` helpers for
   `WorldTransform`.

`OrlExecution` uploads the rig handle arenas and context to CUDA, reflects
the hidden buffer parameter, and can expose the joint device allocation to a
later deformer stage.

## ORL-to-graph node import

`orl_graph_import.h` exposes:

- `import_node_definitions(AnalysisResult, module_name)`;
- source and file overloads;
- `.oro` import/export overloads;
- `register_orl_node_definitions`;
- result types carrying graph registries and compiler diagnostics.

### Port mapping

For each exported function:

- each formal scalar parameter becomes a scalar input port;
- each buffer parameter becomes a buffer input port;
- buffer shape defaults to `<parameter>_count`, with stdlib-specific shape
  aliases for `positions`, `joints`, `weights`, `offsets`, `neighbors`,
  `scratch`, `targets`, `subjects`, `spline`, `effectors`, and related names;
- `joints` buffers are assigned joint domain;
- `port_shape_<name>` metadata can override a default shape;
- `port_semantic_<name>` metadata supplies semantics;
- a non-void result, including a handle result, becomes optional scalar output
  `result`.

Parameter `ParameterAccess` becomes a graph `ResourceEffect` on a stable
`orl.parameter.<function>.<parameter>` resource. Writes are observable.

### Definition metadata

Imported definitions carry:

- `ImplementationKind::OrlFunction`;
- module and function names;
- stage mask from `stage = "solver"`, `"deformer"`, or `"all"/"both"`;
- purity and statefulness;
- capabilities (`cpu` always, `cuda` when the function itself contains a
  parallel loop);
- source provenance;
- optional `PartialEvaluationFootprint`;
- inferred typed `handle_effects`.

The capability inference is syntactic and local to the imported function.
A wrapper that calls a parallel helper does not automatically gain a CUDA
capability solely from the helper call.

### Partial footprint conversion

`add_partial_footprint` converts metadata and typed effects into graph
partial-evaluation information:

- global flag;
- stateful flag;
- sparse-dispatch capability;
- ancestor/descendant/full/none propagation;
- typed handle effects;
- read/write resource IDs.

Old kind-specific metadata such as
`partial_read_joint_ports` and `partial_write_locator_ports` is deliberately
rejected. The current contract is typed handle effects instead.

For stdlib `.oro` export, `stamp_stdlib_node` changes names such as
`solver/ik_two_bone` into IDs such as
`orlrig.solver.ik_two_bone`, sets the operation/category, marks the definition
non-pure and non-inlineable, and rewrites recognized buffer shapes.

Registration preflights duplicate IDs before mutating the destination
registry. This avoids partially registering an imported module after a
collision.

## Graph-to-ORL lowering

### Lowering inputs and outputs

`GraphLoweringOptions` provides:

- generated entry name;
- source preamble;
- include paths;
- module-use emission toggle;
- runtime output resolver;
- runtime handle-index resolver;
- scene revision.

`LoweredGraph` returns:

- generated source;
- entry name;
- diagnostics;
- output descriptors;
- generated handle-type identity overrides;
- scene revision.

### Lowering steps

`OrlGraphLowerer::lower`:

1. checks the entry name;
2. validates the graph;
3. collects ORL modules from node implementations, struct types,
   conversions, and writebacks;
4. collects handle types from graph interfaces and node definitions;
5. emits private safe local names such as `__orl_handle_<name>`;
6. records local-to-canonical handle identity overrides;
7. emits graph input parameters;
8. specializes generic handle ports from exact incoming edge types;
9. walks the validation schedule;
10. emits conversions and writable adapter state;
11. emits direct ORL node calls;
12. resolves identity/runtime nodes through callbacks;
13. emits writeback calls;
14. emits graph output descriptors and the legacy return value.

Normal ORL function nodes use the first declared output as the generated local
result. Runtime nodes do not become arbitrary external calls: an application
must provide `runtime_output_expression`, or an unused runtime node is
allowed to remain only when no output is needed.

### Conversions and writable adapters

Conversions can be:

- ordinary ORL function results;
- one-element matrix buffers;
- ORL writeback functions.

For a writable output adapter, lowering keeps:

- the converted temporary;
- the stable handle selector;
- the runtime-resolved packed index if required;
- the writeback conversion ID.

When a downstream input requires writable access, lowering emits the
writeback function after the consuming node call.

### Handle specialization

An open or union handle input on an imported ORL node is specialized when its
incoming graph connection carries an exact handle. If the same generic local
type receives incompatible exact identities, lowering reports
`ORL_LOWERING_SPECIALIZATION`. It does not emit a runtime dispatch tree.

The graph lowerer declares private nominal handles in generated source and
passes canonical identity overrides into semantic analysis, runtime
reflection, and codegen.

### Current graph-output ABI

The implementation is intentionally narrower than a general reflected value
ABI:

- buffer outputs must resolve to a directly bound graph-input parameter;
- float outputs must resolve to a directly bound graph-input parameter;
- handle outputs must resolve to a directly bound graph-input parameter;
- only one scalar `int` output can be returned from the generated entry;
- additional unsupported scalar output types are rejected;
- no scalar integer output means the entry returns `0`.

`OrlGraphExecution::evaluate_result` reconstructs aliases and descriptors
around this legacy entry ABI.

## LLVM type and function lowering

### Generation phases

`LlvmIrCodegen::Impl::Generate` follows a two-pass model:

1. reset module/codegen state;
2. collect exact handle declarations and canonical identities;
3. create the LLVM `{i64, i64}` handle type when needed;
4. detect solver/hierarchy context use;
5. create hidden context struct types;
6. predeclare all user structs;
7. define all user struct bodies;
8. predeclare all functions;
9. generate every function body;
10. generate host application wrappers.

Predeclaring types and functions lets functions call helpers declared later in
the source and preserves aggregate type identity.

### Active LLVM type map

The active `MapTypeName` implementation maps:

| ORL spelling | LLVM representation |
| --- | --- |
| `int` | `i64` |
| `float` | `double` |
| `string` | pointer |
| `vector`, `normal`, `point`, `vec3`, `dvec3` | `<3 x double>` |
| `vec4`, `dvec4`, `quat` | `<4 x double>` |
| `matrix` | `[16 x double]` |
| user struct | named LLVM struct |
| buffer parameter | pointer to mapped element type |
| exact declared handle | `{ i64, i64 }` |

The analyzer recognizes additional aliases (`bool`, `int64`, `double`,
`float64`, `mat4`, `dmat4`, and GLSL-like forms) that are not all mapped by
this backend. This is a current frontend/backend vocabulary mismatch.

The code generator uses LLVM aggregate operations for structs and fixed
arrays. Member reads use `extractvalue`; member writes use GEP plus store.
Buffer and packed arena loads/stores use explicit pointer arithmetic. Packed
arena accesses are assigned alignment 1 because the arena offsets are byte
offsets rather than necessarily naturally aligned aggregate pointers.

### Hidden context ABI

When program flags indicate context use, every generated ORL function receives
hidden pointer parameters in this order:

```text
explicit source parameters
__orl_solver_context                         if solver_context is used
__orl_hierarchy_context, __orl_hierarchy_data if hierarchy globals are used
__orl_handle_context                          for CUDA handle views
```

`SolverContext` is an LLVM struct with six `i64` fields:

```text
joint_count, controller_count, locator_count,
joints_offset, controllers_offset, locators_offset
```

`HierarchyContext` is fourteen `i64` fields matching
`orlrig::HierarchyContext`. `hierarchy_data` is a pointer to `i64`.

The source-level `solver_context.joints[index]`,
`solver_context.controllers[index]`, and `solver_context.locators[index]`
forms are lowered by loading the byte offset from the context, adding it to
the context base pointer, and indexing the appropriate `Joint`, `matrix`, or
`Locator` element. Direct context assignment is rejected. `hierarchy_data` is
read-only.

Because the flags are stored on `Program`, a helper that does not mention a
context still receives hidden context parameters when another function in the
same program causes the flag to be set.

### Host wrapper ABI

For an `int`-returning function with runtime-supported explicit parameters,
`GenerateHostEntryWrapper` emits a wrapper conceptually shaped as:

```cpp
int64_t __orl_host_entry_<name>(
    void* const* buffers,
    const int64_t* integers,
    const double* floats,
    [const uint64_t* handles],
    [void* solver_context],
    [void* hierarchy_context, void* hierarchy_data]);
```

The wrapper:

- loads buffer parameters from the buffer pointer array;
- loads exact `int` parameters from the integer array;
- loads `float` parameters from the double array;
- reconstructs exact handles from two `uint64_t` lanes;
- appends hidden solver/hierarchy context pointers;
- calls the natural typed ORL function.

Only exact `int`, exact `float`, buffer parameters, and declared exact handles
are wrapper-compatible. Unsupported scalar parameter forms suppress wrapper
generation. A direct `OrlProgram` therefore reports unsupported parameter
reflection before an application tries to invoke it.

### Statements and control flow

The code generator produces explicit LLVM CFG blocks for:

- `if.then`, `if.else`, `if.end`;
- `while.cond`, `while.body`, `while.end`;
- `do.body`, `do.cond`, `do.end`;
- `for.cond`, `for.body`, `for.inc`, `for.end`;
- host parallel callback functions;
- CUDA parallel body/exit blocks.

Missing terminal returns receive a null/default value for the LLVM return
type. There are no generated bounds checks for normal arrays, buffers, or
context arrays.

`break` and `continue` use a stack of loop targets. Host parallel callback
bodies reject loop control, and nested host `parallel for` is rejected.
CUDA parallel generation does not use the host callback path, so its
loop-control validation is not identical to the host path.

### Host parallel loops

The host target does not simply serialize a parallel loop. It creates:

1. a captured context struct containing function arguments and hidden
   contexts;
2. an external C-calling-convention body function
   `orl.parallel.body.<n>(context, index)`;
3. a call to `__orl_parallel_for(0, bound, body, context)`.

`orl_parallel_runtime.cpp` implements that symbol with
`tbb::parallel_for`. The parser's function-parameter bound restriction lets
the backend capture the range predictably.

### CUDA parallel loops

The CUDA target emits `__orl_global_id()` calls. `OrlGpuEngine::LowerCudaGlobalId`
replaces them with:

```text
threadIdx.x + blockIdx.x * blockDim.x
```

The generated body compares the global index with the bound before executing.
The CUDA entry kernel is separate from the natural ORL entry function.

### Expressions and aggregate operations

The code generator supports:

- integer and floating arithmetic;
- numeric conversions between bool/int/float LLVM values where the target
  type is known;
- comparisons;
- vector arithmetic and scalar scaling;
- matrix-times-point/vector;
- struct construction/copy/member access;
- fixed-array construction/indexing;
- buffer indexing;
- quaternion operations;
- direct internal and external calls;
- handle equality and compile-time-specialized `is`.

Logical `and`/`or` are emitted as LLVM `and`/`or` after both operands are
generated. They are not short-circuit CFGs.

Direct calls must use an identifier callee. Unknown calls can be declared as
external functions by `GetOrCreateExtern`, although semantic analysis rejects
unknown calls before the normal runtime path proceeds.

Handle `is` tests are statically folded when the expression's canonical
handle identity is known. The generated code does not inspect the runtime
type lane for this graph-specialized path.

Vector component reads use `extractelement`. Struct field assignments use GEP.
The address generator does not provide a general vector-lane lvalue path, so
parser-accepted assignments such as `v.x = value` can fail in codegen.

## Intrinsics and numeric conventions

`OrlIntrinsicCodegen::TryGenerate` lowers target-independent operations
directly into LLVM:

```text
global_id
dot
cross
length
normalize
clamp
lerp
mat_identity
mat_transpose
mat_inverse
mat_mul
```

`orl_codegen.cpp` additionally lowers:

```text
quat_mul
quat_conjugate
quat_normalize
quat_rotate
```

The matrix representation is a row-major `[16 x double]` aggregate. Matrix
multiply is four-by-four. Matrix-vector multiply promotes a three-component
vector/point with homogeneous `w = 1`, so translation in matrix positions
`[3]`, `[7]`, and `[11]` affects the result.

Known numerical behavior:

- generic `normalize` divides by length without a zero-length guard;
- quaternion normalization has no zero-length guard;
- `mat_inverse` divides by the determinant without singular handling;
- the stdlib sometimes adds its own safe fallback logic, but the intrinsic
  itself does not validate input.

The semantic analyzer lists `min` and `max` as known built-ins, but they do
not have dedicated intrinsic lowering. A source path that reaches codegen can
fall through to external declaration behavior rather than a target-independent
native implementation.

## LLVM optimization

`LlvmOptimizer` maps `OrlOptimizationLevel` to LLVM's
`buildPerModuleDefaultPipeline`:

- `O0`: run no optimization pipeline;
- `O1`, `O2`, `O3`: run LLVM's corresponding default per-module pipeline;
- all levels run module verification afterward.

The optimizer is backend-agnostic LLVM optimization. It is not the graph
optimizer and does not perform graph-level constant propagation, graph DCE,
or graph schedule changes.

## Binary cache

`OrlBinaryCache` stores only machine-code artifacts, not LLVM IR. Supported
artifact kinds are:

```text
CpuObject
CudaPtx
CudaCubin
RocmObject
```

The file format contains:

- `ORLBIN1` magic;
- cache format version;
- artifact kind;
- full material-key length and contents;
- payload length and bytes.

The filename is an FNV-1a key. The full key is stored in the file and checked
on load, so the filename hash is not treated as a cryptographic identity.
Saves use a temporary file and rename. Payloads are capped at 16 GiB.

`OrlExecution`'s material key includes:

- backend and target label;
- runtime ABI version;
- handle ABI version;
- handle-view registry fingerprint;
- LLVM version;
- host triple and pointer size;
- entry name and source name;
- include-path strings;
- top-level source contents.

The key still does not include the contents of resolved `use` files. This is a
real cache invalidation boundary in the current implementation.

## Native JIT

`OrlJitEngine` owns an LLVM `LLJIT` instance for native host execution.

Initialization:

1. initialize the native LLVM target;
2. create an LLJIT builder;
3. optionally install an LLVM object-cache adapter;
4. register `__orl_parallel_for`;
5. register application runtime symbols such as the rig handle-view C ABI;
6. add an optimized IR module or cached object.

Invocation APIs cover:

- no arguments;
- one integer;
- fixed buffer argument arrays;
- dynamic buffers/integers/floats;
- dynamic arguments plus handles;
- solver context;
- hierarchy context/data;
- both contexts and handles.

`LoadModule` rejects `Cuda` and `Rocm` target selections with an explicit
diagnostic. Those enum values describe the broader API but do not turn LLJIT
into a device runtime.

The no-LLVM implementation keeps the public API shape but returns diagnostics
or empty results indicating LLVM is unavailable.

## CUDA backend

### LLVM device compilation

`OrlGpuEngine` supports `Cuda` and `Rocm` compile targets. Current target
defaults are:

```text
CUDA: nvptx64-nvidia-cuda / sm_52
ROCm: amdgcn-amd-amdhsa / gfx900
```

For CUDA, module compilation:

1. rewrites `__orl_global_id`;
2. verifies the configured ORL entry exists and returns an integer;
3. reflects entry parameters into `OrlGpuKernelParameter` records;
4. creates an external `i32` global named `orl_cuda_result`;
5. creates a void `orl_cuda_entry` kernel;
6. calls the natural entry and stores a truncated/cast result into the
   result global;
7. annotates the kernel in `nvvm.annotations`;
8. emits PTX assembly.

Generic pointer address spaces are intentionally not rewritten yet. The
backend comment explains that rebuilding the entry with address-space-one
pointers would invalidate uses under LLVM opaque pointers without a complete
cast-lowering pass.

### Reflected CUDA parameters

The kernel reflection distinguishes:

- buffers;
- `i64`;
- `double`;
- two-lane handles;
- unsupported values.

Exact handle arguments have:

```text
byte size: 16
alignment: alignof(HandleValue)
lanes: 2
lane order: type_id,slot
```

`SetupCudaKernelArguments` validates reflected count and type, and validates
the handle lane contract. Generic buffer argument validation checks that a
byte offset does not exceed the allocation. The lower-level API does not by
itself validate every offset-plus-range, element-stride, or shape condition;
the higher-level execution layer must provide those checks.

### CUDA driver layer

The driver is loaded with `dlopen`/`dlsym` on Linux and
`LoadLibrary`/`GetProcAddress` on Windows. The engine supports:

- CUDA initialization and primary/owned context setup;
- module loading;
- optional PTX-to-cubin driver linking;
- device allocation and external pointer import;
- upload/download/free;
- dynamic kernel argument setup;
- launch and synchronization;
- global `int32` reads.

The default block size is 128 threads. Element launch computes the required
block count by ceiling division.

ROCm target compilation has target selection but
`LoadToDriver` reports that ROCm driver loading is not implemented.

### Device handle views

When the generated module uses handle-backed views, `OrlExecution`:

1. validates a bound `HandleViewContext`;
2. allocates/refreshes device arenas for joints and locators;
3. uploads their host bytes;
4. builds and uploads `HandleDeviceContext`;
5. adds the hidden context buffer to CUDA kernel arguments;
6. optionally downloads the handle arenas after host-visible evaluation;
7. exposes the joint device allocation through
   `handle_joint_device_view()`.

This is the compiler/runtime contract used by the viewer's typed solver graph
path. The owner must keep the returned device view alive until the consuming
deformer has finished.

## `orlc` command-line tool

`orlc_main.cpp` builds the `orlc` executable from the `orlexec` CMake layer.
It supports:

```text
orlc <input.orl>
  -e, --entry <name>
  -I, --include <dir>
  --backend cpu|cuda
  --emit-ir <file>
  --print-ir
  --emit-oro <file>
  --module <name>
  --export <name>       (repeatable)
  -h, --help
```

Runtime compilation is selected by default. If the command is only exporting
`.oro` and no runtime-specific option is requested, it can skip the runtime
compile and use the semantic graph-import path. The input directory is
prepended to include paths. The default module name is the input stem.

`--emit-oro` requires `ORL_HAS_GRAPH_IO`; without RapidJSON it reports that
graph I/O was not built. Exported functions can be filtered with repeated
`--export` options.

## Standard-library contracts used by `orlcomp`

The compiler has no special C++ implementation for most rig algorithms. It
compiles the ORL sources under `resource/stdlib`.

| Source group | Compiler-facing behavior |
| --- | --- |
| `joint.orl` | data-only `Joint`, local/world matrices, quaternion helpers, skin matrices |
| `rig/handles.orl` | exact `joint_handle`/`locator_handle`, `WorldTransform` |
| `constraint/handles.orl` | joint/locator handle unions and `is` specialization |
| `solver/ik_two_bone.orl` | typed exact handles, descendants footprint, integer status |
| `solver/fk.orl` | implicit `solver_context`, full propagation, world buffer |
| `deformer/lbs.orl` | `parallel for`, joint context, in-place output buffers |
| `constraint/*.orl` | typed source/destination transform handles and writebacks |
| `auto_weight/*.orl` | buffer shape conventions, fixed loops, parallel final passes |

`graph_resources.cpp` dynamically compiles public ORL files into `.oro`
definitions. It skips helper-only exports and explicitly skips the current
unsupported solver stems `hd_id`, `spline_ik`, and `full_body_ik` during its
standard registration scan. A graph definition being available in a registry
and a dedicated `AutoWeightRunner`/`SolverRunner` method supporting it are
separate facts.

## Tests that explain this module

The compiler-specific tests are more authoritative than stale design prose.

| Test file | Evidence |
| --- | --- |
| `tests/lexer_tests/lexer_tests.cpp` | literals, invalid tokens, aliases, `use`, handles, unions, exports, metadata |
| `tests/parser_tests/parser_tests.cpp` | AST shape, context flags, canonical parallel syntax, arrays, metadata, handle declaration order |
| `tests/analysis_tests/analysis_tests.cpp` | function summaries, effects, purity, typed handles, union narrowing, views, transitive effects, import |
| `tests/codegen_tests/codegen_tests.cpp` | handles as two lanes, CUDA/host parallel lowering, vectors/matrices/quaternions, contexts, stdlib, JIT and TBB |
| `tests/graph_lowering_tests/graph_lowering_tests.cpp` | exact handle ports, graph outputs, conversions, adapters, typed solver writes, CPU/CUDA parity |
| `tests/gpu_tests/gpu_tests.cpp` | PTX, device launch, parameter reflection, exact handle lanes, device Joint view offsets |
| `tests/exec_tests/exec_tests.cpp` | semantic diagnostics before execution, handle transport/views, caches, packed buffers, context binding |
| `tests/rig_graph_tests/rig_graph_tests.cpp` | typed scene input contracts, exact handle incompatibility, inferred IK effects |
| `tests/scene_graph_tests/scene_graph_tests.cpp` | end-to-end typed solver graph, shared device buffers, hierarchy/revision/writeback behavior |

Tests explicitly verify the current host parallel implementation expects
`@__orl_parallel_for` and `@orl.parallel.body.0` in host IR. This is important
when reading older notes that describe host `parallel for` as sequential.

## Current limitations and reading risks

These are implementation observations, not proposed fixes.

### Frontend/backend vocabulary drift

The lexer and analyzer recognize more scalar/type aliases than
`LlvmIrCodegen::MapTypeName`. A source can therefore pass lexical or semantic
type recognition and still fail during backend mapping. The active codegen
should be treated as the final supported-type authority for direct execution.

### Optional graph semantics versus direct semantics

The graph import path exists to produce rich node definitions. Direct runtime
compilation now also runs `SemanticAnalyzer`, but the runtime signature layer
still reflects a deliberately small application ABI. Passing semantic analysis
does not mean every ORL type is bindable through `OrlExecution`.

### External call escape hatch

`SemanticAnalyzer` rejects unresolved calls for normal compilation. The
lowerer itself can create external LLVM declarations for calls that reach
`GetOrCreateExtern`. This difference is useful for low-level built-ins and
also means code paths that bypass semantic analysis can produce unresolved
symbols rather than an early diagnostic.

### Runtime output ABI is narrower than graph IR

Graph IR can describe richer typed outputs than the current generated entry
ABI can return. The lowerer consequently aliases buffer/float/handle outputs
to bound input parameters and reserves one integer return for status. This is
an implementation boundary, not a statement that graph types are limited to
integers.

### Implicit context is program-wide

The context ABI is inferred from names in the complete parsed program. It is
not a function-local reflected capability. Helpers and wrappers can therefore
receive hidden context parameters that are not visible in their source
signature.

### Cache and textual modules

`use` expansion behaves like source insertion. It does not give the cache a
content hash for each transitive file. A deployment that caches compiled
artifacts must invalidate when stdlib/module contents change.

### Device result width

CUDA writes the entry result to an `i32` global, while the public host result
type is `int64_t`. Status/count values in the current stdlib are small, but
the backend does not preserve arbitrary 64-bit return values.

### Numeric safety

The direct intrinsic implementations do not add bounds checks, zero-length
normalization guards, or singular matrix-inverse diagnostics. Some stdlib
functions add domain-specific checks, but callers should not infer that from
the generic built-in.

### CPU/GPU and host/device distinctions

`OrlJitEngine` is host-only. CUDA compilation and execution are in
`OrlGpuEngine`, and ROCm execution is not implemented. The viewer's strongest
GPU path depends on device-resident handle/joint storage and a window-backed
graphics context; headless unit tests cover only the pieces that do not require
that environment.

## Recommended source reading order

For a careful first pass through this module:

1. `orl_ast.h` to learn the actual source vocabulary.
2. `orl_parser.cpp` and `orl_preprocessor.cpp` to see how source becomes AST.
3. `orl_analysis.h/.cpp` to understand current semantic/effect rules.
4. `orl_runtime_signature.h` and `orl_handle_views.hpp` for reflection and
   handle ABI.
5. `orl_graph_import.cpp` and `orl_graph_lowering.cpp` for graph boundaries.
6. `orl_codegen.cpp` in this order:
   - `Generate`;
   - `MapTypeName`;
   - hidden context setup;
   - host wrapper;
   - declarations and aggregate generation;
   - loop generation;
   - handle views;
   - expression/call lowering.
7. `orl_intrinsics.cpp` and stdlib source to match numerical behavior.
8. `orl_optimizer.cpp`, `orl_cache.cpp`, and `orl_jit.cpp`.
9. `orl_gpu.cpp` and `tests/gpu_tests`.
10. `orl_exec.cpp` only after the compiler ABI is clear; it owns binding,
    cache orchestration, and device handoff rather than language semantics.
