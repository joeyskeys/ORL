# Solver DSL plan 2: handle-valued topology

## Goal

Let user ORL walk the joint hierarchy using handles, without reading packed
parent slots and indexing `solver_context.joints[]`.

At the end of this period a user can write:

```orl
use rig/handles;
use joint;

joint_handle pivot = joint_parent(end);
while (handle_valid(pivot)) {
    Joint data = Joint(pivot);
    // ...
    if (pivot == root) {
        pivot = joint_handle_invalid();
    } else {
        pivot = joint_parent(pivot);
    }
}
```

`hd_id` and `spline_ik` can be redesigned as `root` / `end` / `target`
nodes and returned to the Solver menu.

## What

`Joint.parent` is an `int` at offset 0 of the 128-byte packed joint. That
is storage, not a graph identity. Typed-handle plan 8 left topology walks
on that int, then removed `hd_id`, `spline_ik`, and `full_body_ik` from
graph registration so those ints would not become public sockets.

Two-bone IK only *tests* `parent < 0`. It never steps to the parent as a
new identity. CCD, spline-on-a-chain, and full-body IK must step.

This plan adds a small, exact-handle topology API:

- `handle_valid(h)` — language-visible validity (ABI already has
  `IsValidHandleValue`);
- `joint_handle_invalid()` — the reserved invalid token for
  `orlrig::joint_handle`;
- `joint_parent(joint_handle) -> joint_handle` — wrap the packed parent
  slot as a handle, or invalid if `parent < 0` or out of range;
- `joint_is_ancestor(root, end) -> int` — 1 if `root` is `end` or an
  ancestor of `end`.

Children lists wait for [plan 1](solver_dsl_plan_1_handle_collections.md).
Locator topology does not exist (locators are not a tree).

## Why

A general solver DSL cannot tell users “ports are handles” and then
require `solver_context.joints[Joint(end).parent]` for every walk. That
second world is what the leftover solvers still use, and it is what a
user CCD would copy.

The hierarchy plan already *has* parent and ancestor tables
(`hierarchy_parent_id`, flattened ancestors). They are also packed ints.
Exposing them as user-facing indices would repeat the chain[] mistake.
Wrapping them as handles keeps one public identity type.

This plan is also the cheapest way to put two solvers back in the menu
without handle buffers: a chain that must be a contiguous parent path is
just `root` + `end`.

## Depends on

- [Typed-handle plan 4](extensible_typed_handle_plan_4.md): `Joint(handle)`
  and registered views.
- [Typed-handle plan 3](extensible_typed_handle_plan_3.md): invalid token
  (`kInvalidHandleTypeId`, `kInvalidHandleSlot`).
- [Hierarchy compilation](hierarchy_compilation.md): parent/ancestor
  tables for `joint_is_ancestor` and for a non-walk parent lookup.
- [Overview](solver_dsl_plan_0_overview.md): this is the first
  implementation slice.

Does **not** depend on handle collections.

## Owned submodule

- `resource/stdlib/rig/handles.orl` or a new `resource/stdlib/rig/topology.orl`
  — public signatures.
- `src/orlexec/orlrig/handle_registry.cpp` / `handle_registry.hpp` —
  runtime symbols that produce handles from packed parent slots.
- `src/orlcomp/orl_handle_views.hpp` — if parent is a registered
  handle-producing view rather than a C ABI helper.
- `src/orlcomp/orl_analysis.cpp` — validity intrinsic, no `int` extraction.
- `src/orlcomp/orl_codegen.cpp` — CPU/CUDA lowering of the helpers.
- `src/orlexec/orlrig/hierarchy.hpp` — optional fast ancestor query.
- `resource/stdlib/solver/hd_id.orl`, `spline_ik.orl` — redesign pilots.
- `src/orlexec/orlrig/graph_resources.cpp` — drop those two stems from
  `unsupported_solver_stems`.
- tests listed below.

## Design fixed in this period

1. `Joint.parent` remains `int` in the packed ABI. Do not change the
   128-byte layout and do not type that field as `joint_handle`.
2. `joint_parent` is the only supported way to turn a parent slot into an
   identity. User code that does `solver_context.joints[Joint(h).parent]`
   may still compile (the language cannot ban all int indexing of the
   arena) but stdlib pilots must not do it.
3. Invalid parent (root, broken index) is an invalid handle, not `-1`
   leaking into ORL. `handle_valid` is the only test. Do not allow
   `if (h)` or `if (h == 0)`.
4. `joint_parent` does not read TRS and has no component-data effect.
   It is identity arithmetic. Field effects begin only when the result is
   bound to `Joint` / `WorldTransform` and a field is used.
5. The produced handle is exact `orlrig::joint_handle`. Its `slot` is the
   packed parent index resolved against the active handle context. Its
   `type_id` is the joint type id. Users cannot read those lanes.
6. `joint_is_ancestor` may walk `joint_parent` or use the compiled
   hierarchy tables internally. The public signature takes two handles
   and returns `int` 0/1.
7. Topology helpers require a valid handle context and a matching
   `topology_revision`. A stale context is a runtime error, same as other
   views.
8. Evaluation footprints for walks stay **global / conservative** in this
   period. Proving “writes ancestors of `end`” is plan 3.

## Context the implementation must respect

### Invalid handles already exist

`orl_runtime_signature.h` reserves `type_id == 0` and `slot == -1`.
View access already rejects invalid tokens. What is missing is a
source-level way to *produce* and *test* that token without integers.

Typed-handle plan 1 allowed “a source-level validity utility” later.
This is that utility. Add it as named functions or intrinsics, not as
boolean conversion of a handle.

### Why a function, not `Joint.parent` as a handle field

Changing `Joint.parent` to `joint_handle` would:

- break the shared CPU/GPU/viewer 128-byte ABI;
- embed a dispatch-local token in packed storage;
- make `Joint` copies persist handles.

A helper that reads the int slot inside the runtime and returns a
fresh token keeps storage and identity separate. That is the point of
the handle series.

### Two implementation backends

`joint_parent` can:

- read `Joint(handle).parent` inside the runtime (slot walk), or
- index `hierarchy_parent_id(slot)` from the compiled plan.

Both must return the same handle for a valid acyclic hierarchy. Prefer
the packed joint parent as source of truth (it is what the leftover
solvers use today) and use hierarchy tables only for `joint_is_ancestor`
if that is faster and already available at dispatch.

CUDA must implement the same helper. If the device view registry can
read the parent int field, wrap it on device; do not call host-only
C++.

### Redesign vs migrate

Do not keep `int root, int end, int target_index` and “convert at the
boundary.” Change the exported signatures:

```orl
int solver_hd_id(
    Joint history[],
    joint_handle root,
    joint_handle end,
    locator_handle target,
    int iterations);

int solver_spline_ik(
    joint_handle root,
    joint_handle end,
    point spline[],
    int point_count);
```

`history[]` and `spline[]` stay data. `iterations` / `point_count` stay
counts. Derive the spline chain by walking `end -> root`; validate
`joint_is_ancestor(root, end)`.

`full_body_ik` as one node still needs collections. A one-effector CCD
export may be registered in this period if useful; do not pretend it is
the old multi-effector kernel.

## How

### 1. Public API

Add `use rig/topology;` (or extend `rig/handles.orl`) with the four
functions above. Keep them ordinary exported/non-exported ORL signatures
backed by registered runtime helpers, not new keywords.

### 2. Runtime helpers

Implement C ABI symbols, for example:

- `__orlrig_handle_valid(type_id, slot) -> i64`
- `__orlrig_joint_invalid() -> {type_id, slot}`
- `__orlrig_joint_parent(type_id, slot) -> {type_id, slot}`
- `__orlrig_joint_is_ancestor(root, end) -> i64`

`joint_parent` loads the parent int from the active `HandleViewContext`
joint arena, then builds a handle if `0 <= parent < joint_count`.

### 3. Analysis and codegen

- Treat these helpers as known symbols so analysis does not mark them
  unknown-external (unknown externals force conservative effects).
- `joint_parent` / invalid constructors have no view field effects.
- Lower to the C ABI on host and to the equivalent device symbols on
  CUDA.

### 4. Stdlib pilots

Rewrite `hd_id` and `spline_ik` to the signatures above. For spline,
count the chain with a first parent walk, then apply rotations on a
second walk using reversed `t` so no runtime-sized temp list is
required (plan 4 removes that awkwardness later).

Remove `hd_id` and `spline_ik` from `unsupported_solver_stems`.
Leave `full_body_ik` skipped unless a one-effector export is added.

Flip `rig_graph_tests` that currently require those definitions to be
absent.

### 5. Evaluation

Keep `partial_global = 1` and `partial_propagation = "full"` on both
pilots. Plan 3 replaces that.

## Work items

1. Validity / invalid / parent / ancestor API and runtime symbols.
2. Host and CUDA lowering; topology revision checks.
3. Analysis: known helpers, no int conversion.
4. Redesign `hd_id` and `spline_ik`; re-register.
5. Syntax/codegen call-site updates for the new signatures.
6. Numeric fixtures vs the old index implementations.

## Tests

Extend:

- `tests/analysis_tests/analysis_tests.cpp`
- `tests/syntax_tests/language_syntax_tests.cpp`
- `tests/codegen_tests/codegen_tests.cpp`
- `tests/exec_tests/exec_tests.cpp`
- `tests/rig_graph_tests/rig_graph_tests.cpp`
- `tests/orlrig_tests/orlrig_tests.cpp`
- `tests/gpu_tests/gpu_tests.cpp`

Required positive cases:

- `joint_parent` of a root is invalid;
- `joint_parent` of a child equals the handle of its parent joint
  (same `find_joint` identity);
- `joint_is_ancestor(root, end)` matches a known chain;
- redesigned `hd_id` / `spline_ik` are registered graph nodes with
  handle sockets, no `int` component ports;
- CPU numeric results match the pre-redesign fixtures.

Required negative cases:

- `locator_handle` passed to `joint_parent` rejected;
- invalid handle into `Joint(...)` still rejected;
- no public way to read `slot` or convert the parent handle to `int`;
- `full_body_ik` remains unregistered unless a one-effector node is
  an explicit extra.

## Exit criteria

- User ORL can walk rootward on handles only.
- `hd_id` and `spline_ik` are in the Solver menu with typed sockets.
- Packed parent ints are not graph ports.
- Walk footprints are still conservative (documented hand-off to plan 3).

## Not in scope

- `joint_children[]` or descendant lists (plan 1).
- Sparse ancestor footprints (plan 3).
- Scratch arrays to store a chain (plan 4).
- Changing `Joint.parent`’s storage type.
- Controller or locator trees.
