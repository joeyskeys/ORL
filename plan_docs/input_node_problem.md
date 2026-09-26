# Extensible typed handles for rig graph inputs

Status: revised design proposal

## Decision summary

The scene input nodes should expose exactly one public output:

```text
find_joint.handle      : joint_handle
find_locator.handle    : locator_handle
```

There should be no public `index` or `xform` output on these nodes. There
should be no `find_controller` graph node. A graph stores stable joint/locator
identity. The runtime resolves that identity to the current storage location
only when an ORL function asks for it.

`handle` should become a core ORL value category. Concrete handle types and
their data access belong to `orlrig`, not to the core language:

```orl
handle joint_handle;
handle locator_handle;

handle xform_handle =
    joint_handle | locator_handle;
```

The exact syntax can change, but the semantic model should not:

- `handle` is the universal opaque handle type;
- `joint_handle` and `locator_handle` are nominal subtypes declared by
  `orlrig`;
- a named handle union describes the complete set of component kinds accepted
  by a parameter;
- graph socket compatibility uses those types directly;
- `is` narrows a handle to an exact nominal type, after which a registered
  struct conversion binds a storage-backed typed view;
- packed indices and pointers are never exposed as authored graph values.
- controllers remain an `orlrig`/viewer authoring utility and never enter the
  compiled rig graph as handles.

The previous rig-facade-only proposal is superseded by this design.

## Current problem

`orlrig.input.find_joint`, `find_controller`, and `find_locator` currently
expose `handle`, `index`, and `xform`. The target removes `find_controller`
entirely and reduces the joint/locator nodes to one typed output.

`handle` is a session-stable `ComponentId`. `index` is a transient location in
a packed runtime array. Both eventually look like an ORL `int`. The transform
output is matrix-like, but joints and locators have different storage and
mutation behavior. Controllers additionally belong to the authoring
attachment workflow rather than solver dataflow.

`solver_ik_two_bone` demonstrates the resulting leak:

```orl
export int solver_ik_two_bone(
    int root,
    int mid,
    int end,
    int target_index,
    int pole_index)
```

The signature does not express that `root`, `mid`, and `end` must be joints or
that `target_index` and `pole_index` must be locators. It also exposes the
storage strategy of `solver_context` to graph users.

The missing type information is reconstructed from metadata:

```orl
string partial_read_joint_ports = "root,mid,end",
string partial_write_joint_ports = "root,mid",
string partial_read_locator_ports = "target_index,pole_index"
```

Generic constraints list one port under several component-kind metadata keys.
`compile_evaluation_plan` then follows a connection back to a `find_*` node to
discover what was actually connected. This is redundant, string-based, and
easy to make inconsistent with the function signature.

## Requirements

1. Every scene input node has one output socket named `handle`.
2. The node definition determines the concrete output handle type.
3. ORL function parameters may accept:
   - any handle;
   - one exact handle type;
   - a declared union of handle types.
4. Concrete component kinds remain outside core ORL and are extensible by
   `orlrig` or another domain package.
5. A handle can resolve component data without exposing packed indices or
   host/device pointers.
6. Invalid component access is rejected at compile time.
7. A handle must have an exact nominal type, either from its declaration or
   from an `is`-guarded branch, before component data is materialized.
8. The same source must work on CPU JIT and CUDA/PTX.
9. Handle types supply component-kind information to graph validation and the
   evaluation plan, replacing the current kind-specific port metadata.
10. Runtime-selected identity remains conservative for partial evaluation
    when the compiler cannot prove which component instance it denotes.
11. Controllers remain outside the handle system: no `controller_handle`, no
    `find_controller`, and no controller component access from ORL rig nodes.

## What is realistic

### Core opaque handle category

Adding `handle` to the core type system is realistic. It requires changes to
the lexer/parser, AST type model, semantic analysis, graph logical types,
runtime reflection, host entry wrapper, and GPU parameter ABI, but it does not
require joints or locators to become core language concepts.

The core language needs to understand:

- handle declarations;
- nominal handle identity;
- union membership and assignability;
- null/invalid handles;
- equality;
- compile-time-specialized `is` narrowing;
- the opaque runtime representation.

The core language does not need to understand what a joint is or where it is
stored.

### Feasibility boundary

This is not a small lexer feature. The requested safety is realistic only if
ORL gains:

- nominal and union type checking before LLVM emission;
- a distinct reflected runtime parameter kind;
- an explicitly specified CPU/CUDA ABI;
- target-independent storage resolvers and typed-view field lowering.

It is not sufficient to parse `joint_handle` and lower it directly to the
same unbranded `i64` used by `int`. Without typed expression analysis and
assignability checks, that would preserve the current integer ambiguity under
a new spelling.

If the project is not willing to add those compiler/runtime pieces, the safe
fallback is an `orlrig` graph-facade type system that materializes existing
integer indices before ORL execution. That fallback provides graph safety but
does not satisfy the requirement that custom ORL functions declare and
manipulate handles. The two designs should not be mixed halfway.

### ORL-rig-defined handle types

`orlrig` can provide a standard module containing declarations:

```orl
handle joint_handle;
handle locator_handle;
handle mesh_handle;

handle xform_handle =
    joint_handle | locator_handle;
```

These are nominal types. Two independently declared handle types are not
compatible merely because they have the same runtime representation.

A declaration creates type identity only. It does not automatically make
storage accessible. The application/domain package must also register the
runtime tag, supported backends, storage resolver, supported conversions, and
effects for that type. A handle type with no registered resolver may still be
forwarded or compared, but attempting to read or write component data through
it is a compile/registration error.

An application may define more types, such as:

```orl
handle curve_handle;
handle camera_handle;
handle constraint_target_handle =
    joint_handle | locator_handle | curve_handle;
```

This is the intended meaning of extensibility.

### Typed access to backing data

An exact typed handle leads to component storage only through a registered
struct conversion. That conversion binds a live, storage-backed typed view:

```orl
Joint joint_data = Joint(joint);
Locator locator_data = Locator(locator);
WorldTransform world_data = WorldTransform(joint);

quaternion old_rotation = joint_data.rotation;
joint_data.rotation = solved_rotation;

matrix world = world_data.xform;
locator_data.xform = solved_locator_xform;
```

`WorldTransform` is an ordinary `orlrig` struct, for example:

```orl
struct WorldTransform {
    matrix xform;
}
```

It represents computed data that is not stored in `Joint`. The conversion
`WorldTransform(joint)` may perform hierarchy traversal. Once converted, all
data access uses ordinary field syntax. Every registered view is read/write
capable: its descriptor must provide both read and write lowering for every
exposed field. If meaningful write semantics cannot be defined, that view
conversion must not be registered.

`Joint(joint)` is valid because `joint` has exact type `joint_handle` and
`orlrig` registers one `joint_handle -> Joint` conversion. Every exact
source-handle/destination-struct pair has at most one registered conversion,
so the destination type in `Destination(handle)` fully identifies the
operation. Multiple destination structs may be supported where they represent
different semantics; do not overload one destination type with ambiguous
local/world meanings.

The following is therefore a compile-time error:

```orl
Joint invalid(locator_handle value) {
    return Joint(value);
}
```

For a union or universal handle, use `is` to select an exact type before
materializing component data. This keeps conversion lookup local and avoids
inventing a common data model for unrelated component kinds.

The converted object is not an ordinary copied struct. It is a compiler-managed
lvalue view tied to the original handle:

```orl
Joint data = Joint(root);
data.rotation = solved_rotation;
```

The field assignment writes component storage immediately; there is no
explicit writeback function. The conversion itself only establishes the view.
Actual field reads and writes determine the inferred effects.

This is a new expression/value category even though it uses normal struct
syntax. In the first implementation a handle-backed view is local and
non-escaping: it cannot be returned, stored in a buffer or persistent state,
or copied as a whole value. Individual fields remain ordinary ORL values and
may be copied or passed to helper functions. These restrictions avoid hidden
aliasing and dangling views while preserving simple authored syntax.

## Controller scope: authoring and rig GUI only

Controllers are naturally attached to a joint or locator and manipulate that
target through the viewport attachment workflow. They are not independent
solver components and do not participate in typed rig graph dataflow.

Keep the existing controller authoring model:

- `Controller::xform` remains the setup/display transform used to place the
  controller and establish its attachment-relative placement;
- `Controller::input_xform` remains the separate user animation input, so
  setup scale/orientation is not mistaken for pose input;
- attachment targets, relative offsets, reverse indices, and
  `input_drives_target` remain owned by `ComponentManager`;
- viewport setup transforms and animation transforms continue to use the
  existing attachment-aware operations;
- applying a controller updates its attached joint or locator through the GUI
  authoring pipeline.

Consequently:

- do not declare `controller_handle` in `orlrig`;
- remove `orlrig.input.find_controller`;
- do not register controller typed views or controller read/write effects;
- do not pack controller handles, controller snapshots, or controller
  count/offset lanes into the final compiled rig ABI;
- keep controller shapes, setup state, input state, and attachment validation
  as authoring/GUI utilities in `orlrig` and `orlviewer`.

An old graph containing `find_controller` is not converted to another handle
type. Project migration must move its intended behavior to controller
attachment configuration or report an actionable incompatibility.

## Necessary adjustment: not a raw pointer

A handle should feel reference-like to the user, but it must not be a raw
pointer.

Raw pointer semantics are not suitable because:

- host pointers are invalid in CUDA device code;
- device pointers are not stable scene identity;
- component arrays are repacked;
- joint world transforms are computed and are not a field in `Joint`;
- joint and locator storage do not share the same C++ payload;
- pointer arithmetic would bypass graph effects, bounds checks, and
  registered field lowering;
- serialized graphs cannot safely contain process addresses.

The language should prohibit all pointer-like and direct member operations on
handles:

```orl
value->xform
value.xform
*(value + 1)
```

Member access applies only after conversion to a normal struct, vector, or
matrix value. Existing ORL member checking then handles invalid fields without
special handle rules.

## Type declarations and parameter syntax

### Proposed declaration syntax

Use `handle` as both the built-in top type and the introducer for nominal
handle declarations:

```orl
handle joint_handle;
handle locator_handle;
handle xform_handle = joint_handle | locator_handle;
```

At top level, `handle name;` declares a type. In a parameter or declaration,
`handle value` uses the universal type.

The canonical package-prefixed name is the handle type's identity. For
example, source may use the short imported spelling `joint_handle`, while
reflection and serialization use a canonical name such as
`orlrig::joint_handle`. Two packages declaring the same short name therefore
create different nominal types.

The current textual `use` expansion must retain enough module provenance, or
the package registry must provide it, to assign that canonical name
deterministically. Graph files serialize the canonical name. Renaming the
declaration or moving it to another package is a breaking type-identity change
and requires an explicit graph/schema migration; there is no separate stable
declaration ID in the first design.

The runtime `type_id` is a versioned deterministic 64-bit hash of this
canonical name. Registry construction must detect collisions and fail rather
than silently treating two types as equal. Registration order, C++
`typeid().hash_code()`, and process addresses are not valid IDs.

### Parameter examples

Exact types provide the strongest graph checking:

```orl
export int solver_ik_two_bone(
    joint_handle root,
    joint_handle mid,
    joint_handle end,
    locator_handle target,
    locator_handle pole)
```

A generic transform constraint can opt into a controlled union:

```orl
export int constraint_copy_xform(
    xform_handle source,
    xform_handle destination)
```

An intentionally generic utility may use:

```orl
export int inspect_component(handle value)
```

but it cannot read joint or locator data until it narrows the value with `is`.

### Inferred access effects

Do not add `in`, `out`, or `inout` parameter syntax. The compiler can infer
the required component access directly from typed operations:

- copying, forwarding, comparing, or testing a handle has no component-data
  effect;
- binding a registered view such as `Joint(value)` has no data effect by
  itself;
- reading `data.field` through that view declares a read;
- assigning `data.field = new_value` through that view declares a write;
- compound assignment or code that performs both is inferred as read-write.

Each C++ view descriptor declares the field-level read/write lowering
available to semantic analysis and codegen. Semantic analysis computes a
parameter-relative effect summary from actual field operations and propagates
summaries transitively through helper calls to a fixed point. It must preserve
parameter provenance through local handle aliases and argument remapping at
call sites. Handle-backed views cannot be returned or passed as whole values.
Branches are conservatively merged, except that dead `is` branches are
removed by graph specialization before final effects are emitted.

Unknown external calls that receive a handle, or unresolved recursive effect
cycles, must fail registration or conservatively make the node global; they
must not be treated as effect-free. This requires strengthening the current
analyzer, which does not yet propagate parameter effects transitively.

The graph importer places the inferred read/write summary in the exported node
contract. This replaces kind-specific `partial_*_ports` metadata. A separate
write-only syntax is unnecessary: a parameter is write-only whenever its
transitive summary contains a write and no read.

Algorithm-specific propagation, such as `descendants`, is not implied by a
handle type and may remain an explicit function property.

## Assignability and graph compatibility

Treat each handle type as a set of possible nominal leaf types:

```text
joint_handle  = {joint}
locator_handle = {locator}
xform_handle  = {joint, locator}
handle        = all registered handle types
```

A connection is safe when every possible source type is accepted by the
destination:

```text
joint_handle -> joint_handle  valid
joint_handle -> xform_handle  valid
joint_handle -> handle        valid
xform_handle -> joint_handle  invalid without narrowing
handle       -> joint_handle  invalid without narrowing
locator_handle -> joint_handle invalid
```

This rule works for both the node editor and compiler and prevents a broad
source from reaching a narrower consumer.

`orlgraph::LogicalType` needs a `Handle` kind containing the canonical handle
type or normalized accepted leaf set. Port compatibility must use handle
assignability rather than exact `LogicalType` equality.

The input definitions then expose:

```text
orlrig.input.find_joint
    handle : joint_handle

orlrig.input.find_locator
    handle : locator_handle
```

All sockets have the same user-facing name. Their nominal type and node color
or socket style show what they represent. Controllers have no graph input
definition.

## General handles, unions, and safe narrowing

`is` is the only new narrowing operator. There is no `as` keyword. Inside a
branch dominated by an `is` test, semantic analysis flow-narrows the original
expression to the tested exact handle type, so the normal struct-conversion
syntax can materialize component data:

```orl
if (value is joint_handle) {
    Joint data = Joint(value);
    data.rotation = solved_rotation;
} else if (value is locator_handle) {
    Locator data = Locator(value);
}
```

Required rules:

- `is` is defined only for handle values and nominal handle types;
- a successful branch narrows the tested expression to the exact nominal
  handle type;
- `Destination(value)` then uses the registered conversion for that exact
  handle type and binds a non-escaping typed view;
- there is no unchecked reinterpret cast;
- an invalid or stale handle cannot be dereferenced;
- data access after narrowing contributes its registered inferred effect.

### Static graph dispatch

`is` must not become a per-element runtime RTTI branch. A graph connection
fixes the nominal handle type before execution: every `find_*` output is exact,
and a union or universal parameter is an accepted-input set rather than a
runtime-varying sum value. Graph compilation specializes the called ORL node
for the connected nominal leaf type and folds each `is` expression to `true`
or `false`; dead branches are removed before LLVM emits the executable kernel.

Standalone ORL compilation may retain a symbolic `is` condition in its typed
IR, but executable graph lowering must resolve it. Type provenance must be
preserved through generic helper calls and forwarding nodes so specialization
can continue transitively.

The first implementation should reject a graph if a source can change its
nominal handle type at runtime or if graph specialization cannot prove the
leaf type. Component identity may still vary at runtime within that one type;
for example, a `joint_handle` selector may choose different joints without
changing the compiled branch. Supporting genuinely runtime-varying handle
kinds would be a separate language feature and would require explicit runtime
sum-type dispatch.

## Runtime representation

Separate persistent graph identity from the value passed to a kernel.

### Persistent graph value

The editable graph stores:

- the `find_*` node definition;
- its selected scene name or stable `ComponentId`;
- the nominal output handle type.

It never stores a packed index.

### Dispatch-local ORL value

At bind or lowering time, the runtime converts stable identity to an opaque,
dispatch-local token:

```cpp
struct HandleValue {
    std::uint64_t type_id;
    std::int64_t slot;
};
```

- `type_id` is a stable registered nominal type tag;
- `slot` addresses the current snapshot of that type's storage;
- a reserved tag/slot represents an invalid handle;
- handles exist only for one dispatch and cannot be stored in persistent ORL
  state, serialized, or reused by a later dispatch;
- topology and packing are immutable for the lifetime of a compiled rig;
- any topology or packing change invalidates that compiled rig and requires a
  rebuild, which binds fresh dispatch-local tokens.

The first ABI is exactly these two 64-bit lanes and has no generation lane.
This is safe because no token can outlive its dispatch and a compiled rig
cannot observe repacking. A future feature that permits persistent handles
would require a new versioned ABI with generation checking or handle-table
indirection; it must not weaken this contract in place. The layout is not
permission to expose either field to ORL source.

For graph constants, `SceneInputCatalog` resolves stable identity to
`HandleValue` before execution. Packed joint/locator ordering remains an
implementation detail of the current snapshot rather than a serialized index
contract.

### Runtime context

Typed views need a storage directory. Do not enlarge the existing
`SolverContext` silently; its six-`int64` layout is shared with codegen and
CUDA.

Introduce a versioned implicit handle context, similar to the existing solver
and hierarchy contexts, or specialize access directly to an existing arena
when the handle type is statically known.

A general design needs:

- handle-type descriptor table;
- per-type storage base, count, stride, and access policy;
- stable-ID-to-slot binding on the host;
- CPU and GPU representations with offsets, not host pointers;
- field-level read/write lowering for each registered view.

Known rig handle views can lower efficiently:

- reading/writing `Joint(joint_handle).field` addresses
  `solver_context.joints[slot]`;
- reading/writing `Locator(locator_handle).xform` addresses
  `solver_context.locators[slot]`;
- reading/writing `WorldTransform(joint_handle).xform` uses registered
  hierarchy-aware world/local conversion;
- graph specialization selects the exact resolver before code generation, so
  no union-kind switch remains in the executable kernel.

## LLVM and RTTI

LLVM can implement this design, but "LLVM RTTI" should not be the runtime
contract.

LLVM's `isa`, `dyn_cast`, and `Type` APIs are compiler-side C++ utilities.
They inspect LLVM IR objects while compiling; they do not provide runtime
type information for application values.

C++ RTTI (`typeid`, `dynamic_cast`, vtables) is also unsuitable as the shared
handle model:

- it requires C++ object layouts and usually host addresses;
- those addresses and vtables are not portable to CUDA/PTX;
- it couples ORL ABI to one host compiler and linker;
- it does not describe packed GPU arenas or graph serialization.

Keep an explicit `type_id` tag in the runtime token for binding validation,
serialization, diagnostics, and resolver registration. It does not define the
execution semantics of `is`: graph specialization already knows the nominal
leaf type and folds the test before LLVM emission. Generated CPU and GPU code
therefore calls the selected exact-type resolver directly.

A CPU-only adapter may use C++ RTTI internally, but that must remain behind
the registered resolver and cannot define ORL semantics.

## Compiler work required

This is a core language feature, not only an `orlrig` registration change.

### Lexer, parser, and AST

Relevant files:

- `src/orlcomp/orl_lexer.h/.cpp`
- `src/orlcomp/orl_parser.h/.cpp`
- `src/orlcomp/orl_ast.h`

Add:

- `handle` keyword and top-level handle declarations;
- nominal and union type symbols;
- the handle-only `is` expression;
- declaration-before-use and duplicate/cyclic-union diagnostics.

The current parser tracks only `struct_names_`; it needs a unified type table
or a separate handle-type table.

### Typed semantic analysis

Relevant files:

- `src/orlcomp/orl_analysis.h/.cpp`
- `src/orlcomp/orl_graph_import.cpp`

This is the critical prerequisite. Current semantic analysis records function
signatures and coarse parameter access, while several expression type errors
are detected only during LLVM emission.

Before generic handles may access data, analysis must resolve:

- the static type of every expression;
- whether an expression is an ordinary value or a handle-backed lvalue view;
- handle union normalization and assignability;
- function-call argument types;
- registered handle-to-view conversions and field capabilities;
- flow narrowing inside `is`-guarded branches;
- graph-time propagation of the connected nominal leaf type;
- transitive effects through helper calls;
- invalid direct member access and invalid indexing/arithmetic on handles;
- conversion from a non-exact or insufficiently narrowed handle;
- illegal whole-view copy, return, buffer storage, or escape.

Unsafe handle access should fail before graph registration or LLVM codegen.

### Graph IR and serialization

Relevant files:

- `src/orlgraph/graph_types.hpp/.cpp`
- `src/orlgraph/graph_ir.cpp`
- `src/orlgraph/graph_validation.cpp`
- `src/orlgraph/graph_serialization.cpp`
- `src/orlgraph/graph_reflection.*`

Add `LogicalTypeKind::Handle`, canonical nominal IDs, accepted leaf types,
assignability, reflection, deterministic serialization, and ABI versioning.

The ORL importer maps handle parameters directly to typed scalar ports. It no
longer stamps them as `scene.array_index`.

### LLVM lowering

Relevant files:

- `src/orlcomp/orl_codegen.cpp`
- `src/orlcomp/orl_graph_lowering.cpp`

Map handles to a target-independent LLVM aggregate or an internal pair of
`i64` values. Implement:

- equality and validity;
- compile-time-folded `is` tests from graph specialization;
- exact-type view binding and field load/store lowering;
- no implicit cast between `int` and handle;
- no pointer arithmetic or direct struct member extraction from a handle.

### Runtime and GPU ABI

Relevant files:

- `src/orlcomp/orl_runtime_signature.h`
- `src/orlexec/orl_exec.hpp/.cpp`
- `src/orlexec/orl_graph_exec.*`
- `src/orlcomp/orl_gpu.h/.cpp`

The runtime currently reflects only buffer, `int64`, and `float64`
parameters. Add a handle parameter kind and binding API.

Do not rely on the platform C++ aggregate ABI. Define how the host entry
wrapper and CUDA kernel wrapper flatten or pass the two handle lanes, and test
that CPU and GPU observe identical values.

### `orlrig` registration and storage resolution

Relevant files:

- `src/orlexec/orlrig/graph_resources.*`
- `src/orlexec/orlrig/component_store.*`
- `src/orlexec/orlrig/evaluation.*`
- `src/orlviewer/graph_scene_inputs.*`
- `src/orlviewer/graph_scene_runtime.cpp`

`orlrig` uses a C++ registration API for nominal type descriptors,
handle-to-view conversions, storage resolvers, and field-level CPU/GPU
read/write lowering. ORL annotations and separate manifests are not used.
Package authors register these descriptors once; graph authors only use
`Destination(handle)` and ordinary field syntax. The three scene input nodes
expose only `handle`.

## Revised two-bone IK

The authored signature should become:

```orl
use rig/handles;

export int solver_ik_two_bone [[
    string stage = "solver",
    string partial_propagation = "descendants"
]]
    (joint_handle root,
     joint_handle mid,
     joint_handle end,
     locator_handle target,
     locator_handle pole)
{
    Joint root_joint = Joint(root);
    Joint mid_joint = Joint(mid);
    Joint end_joint = Joint(end);

    WorldTransform root_world = WorldTransform(root);
    Locator target_data = Locator(target);
    Locator pole_data = Locator(pole);

    point root_pos = root_world.xform * point(0.0, 0.0, 0.0);
    point target_pos = target_data.xform * point(0.0, 0.0, 0.0);
    point pole_pos = pole_data.xform * point(0.0, 0.0, 0.0);

    // Solve.
    root_joint.rotation = solved_root_rotation;
    mid_joint.rotation = solved_mid_rotation;
}
```

The node graph is now self-describing. The evaluation plan obtains kinds from
the signature and read/write modes from the function's inferred effect
summary. The function body never sees or validates packed indices.

Bounds, invalid-handle, and storage-availability checks belong to view
field lowering. The solver still validates hierarchy
relationships and numerical conditions that are specific to the algorithm.

## Evaluation-plan behavior

For a direct connection:

```text
find_joint("root").handle -> solver.root
```

the compiler knows:

- static type: `joint_handle`;
- inferred access: read-write, from field reads and
  `root_joint.rotation = ...`;
- stable component identity: selected by the input node;
- propagation: `descendants`.

It can populate `read_joints`, `write_joints`, and affected descendants
without `partial_read_joint_ports` or `partial_write_joint_ports`.

Type knowledge and identity knowledge are separate. A runtime selector may
choose among several joints while retaining exact type `joint_handle`; this
does not prevent compile-time folding of `is joint_handle`, but the compiler
may not know which joint instance will be selected. Such a region must remain
global or use a runtime dirty set. Strong typing alone does not make dynamic
identity statically sparse.

## Error model

Required compile-time diagnostics include:

- unknown handle type;
- duplicate or cyclic handle-union declaration;
- incompatible graph connection;
- broad handle passed to a narrower parameter;
- direct handle member access or handle arithmetic/indexing;
- view conversion used with an incompatible handle type;
- conversion attempted before the handle has an exact flow type;
- handle-backed view copied, returned, persisted, or passed as a whole value;
- unproven narrowing;
- unresolved or runtime-varying nominal handle type during graph
  specialization;
- handle type registered without a target-supported resolver;
- view field missing compatible CPU/CUDA read or write lowering.

Required runtime failures include:

- selected component no longer exists at bind time;
- stable identity cannot be resolved in the current snapshot;
- invalid/null handle dereferenced through a view;
- slot outside the registered arena bounds;
- view target removed during evaluation.

Graph compilation should report these as structured diagnostics with node and
port identity. It must not silently substitute index `0`, reinterpret an
integer, or fall back to another component kind.

## Migration

Version the ORL language, graph logical ABI, and project graph schema.
The one-socket scene-input change is intentionally breaking:

- old `find_joint`/`find_locator` `index` and `xform` ports are removed;
- old `find_controller` nodes are removed; controller behavior belongs to
  attachment and viewport authoring configuration;
- old graphs that reference those definitions or ports fail with a structured
  node/port diagnostic and must be rebuilt;
- no hidden compatibility node, implicit matrix adapter, or integer-to-handle
  reinterpretation is used.

For old ORL stdlib nodes:

- replace index parameters with typed handles;
- replace direct `solver_context.*[index]` access with typed view conversions
  and field reads/writes;
- remove kind-specific partial-port metadata;
- retain only execution properties that types cannot express.

There must be no implicit `int -> handle` or `handle -> int` conversion.

## Staged implementation plans

Implementation is split at testable compatibility boundaries. Complete and
verify each period before starting the next:

1. [Core frontend and exact handle types](extensible_typed_handle_plan_1.md)
2. [Graph type system and persistence](extensible_typed_handle_plan_2.md)
3. [Compiler and execution ABI](extensible_typed_handle_plan_3.md)
4. [`orlrig` registry and typed data access](extensible_typed_handle_plan_4.md)
5. [Scene inputs and one-socket nodes](extensible_typed_handle_plan_5.md)
6. [Effects and two-bone IK pilot](extensible_typed_handle_plan_6.md)
7. [Unions, `is`, and static specialization](extensible_typed_handle_plan_7.md)
8. [Full rig migration and legacy removal](extensible_typed_handle_plan_8.md)

Plans 1-4 are additive. Plan 5 is the breaking scene-input boundary and
removes the obsolete find-node compatibility path. Plan 6 proves one
production solver end to end. Plan 7 adds generic accepted handle sets only
after exact handles work. Plan 8 completes the remaining algorithm migration
and removes any unrelated legacy solver metadata.

## Acceptance criteria

- `find_joint` and `find_locator` each expose one socket named `handle`.
- No `find_controller` node or `controller_handle` exists.
- The final compiled rig ABI contains no controller snapshot/count/offset
  lanes.
- Joint and locator handles have distinct graph types.
- Exact, union, and universal-handle assignability follows the safe subset
  rule.
- `solver_ik_two_bone` has no index parameters and performs no packed-index
  bounds checks.
- A locator cannot connect to a joint parameter.
- A general or union handle cannot access component data until an `is` branch
  narrows it to one exact nominal type.
- `is` is folded from graph connection types, and no runtime handle-kind
  dispatch remains in generated kernels.
- Direct member access on every handle type is a compile error; member access
  begins only after conversion to a typed view.
- Handle-backed reads and writes use registered destination-struct views and
  ordinary field syntax; there are no named handle data accessors or explicit
  writeback calls.
- Handle-backed views cannot escape their local dispatch scope or be copied as
  whole values.
- Conversion from universal `handle` to component data requires narrowing to
  an exact type.
- Evaluation-plan component kinds and read/write sets no longer depend on
  `partial_*_{joint,locator}_ports`; controller port metadata is removed with
  controller graph integration.
- Repacking scene arrays does not alter serialized graph identity.
- A topology or packing change invalidates and rebuilds the compiled rig;
  dispatch-local tokens never survive that boundary.
- CPU and CUDA produce the same results for each specialized exact handle
  path.
- Unsupported dynamic identity causes conservative evaluation, not an unsafe
  guessed dependency.

## Resolved controller boundary

Controller attachment behavior is not part of the handle implementation.
Controllers remain authoring/GUI utilities that drive attached joints or
locators before compiled rig evaluation. The handle system therefore has no
remaining controller-view design decision.
