# Solver DSL plan 4: scratch working sets

## Goal

Give solver and deformer authors a runtime-sized temporary that is not a
graph-authored input and not a compile-time `Type values[N]`.

At the end of this period a user can gather a parent chain, reverse it,
and iterate root-to-end without encoding the algorithm as two awkward
walks:

```orl
export int solver_spline_ik [[
    string stage = "solver",
    string scratch_joint_handles = "chain"
]]
    (joint_handle root,
     joint_handle end,
     point spline[],
     int point_count)
{
    int n = 0;
    joint_handle pivot = end;
    while (handle_valid(pivot)) {
        chain[n] = pivot;
        n = n + 1;
        if (pivot == root) {
            pivot = joint_handle_invalid();
        } else {
            pivot = joint_parent(pivot);
        }
    }
    int i = n - 1;
    while (i > 0) {
        // chain[i] is nearer the root
        ...
        i = i - 1;
    }
    return 1;
}
```

## What

ORL locals are scalars or fixed arrays. Buffer parameters are caller-
owned graph or runtime inputs (`history[]`, `spline[]`, `world[]`).
There is no function-local allocation and no compiler-owned scratch.

That is why the plan 2 spline redesign must count, then walk again with
reversed `t`. It is also why `hd_id` uses a **graph** `Joint history[]`
for warm-start state: the language has no node-local persistent buffer
either.

This plan adds **dispatch-scoped scratch buffers** declared on the
exported function and allocated by the runtime from a shape the compiler
can see (`joint_count`, a parameter count, or a simple multiple).

Scratch may hold:

- ordinary data (`int`, `float`, `vector`, `point`, `Joint`);
- exact handles, if [plan 1](solver_dsl_plan_1_handle_collections.md)
  has landed.

Scratch is not:

- `malloc` in the kernel;
- a persistent history that survives the next frame (that stays an
  explicit graph buffer, like `history[]`);
- a back door to store handles in the project.

## Why

User solvers gather, then scatter:

- build a root-to-end chain;
- store previous directions for a pole or swing limit;
- mark visited joints in a CCD pass;
- deformers that need a prefix-sum or a per-vertex temp.

Forcing every temp to be a graph input pollutes the node API
(`scratch0[]` sockets the user must not wire). Forcing fixed `N`
caps chain length in the language. Reversed two-pass walks are a
compiler trick, not a DSL.

GPU kernels cannot heap-allocate per thread in this stack. The
runtime must size scratch **before** launch from hierarchy or
parameter counts that are already known at dispatch.

## Depends on

- Existing buffer ABI and shape symbols (`<name>_count`, `joint_count`).
- [Plan 2](solver_dsl_plan_2_handle_topology.md) for the spline/hd_id
  rewrite that will consume scratch.
- [Plan 1](solver_dsl_plan_1_handle_collections.md) if scratch elements
  are handles. Ship scalar/data scratch first if collections are late.
- Solver context / handle context already passed as hidden pointers.

## Owned submodule

- `src/orlcomp/orl_parser.cpp` / `orl_ast.h` — scratch declaration
  syntax or export metadata.
- `src/orlcomp/orl_analysis.cpp` — shape checking, no escape.
- `src/orlcomp/orl_graph_import.cpp` — do **not** import scratch as
  user-visible ports.
- `src/orlcomp/orl_codegen.cpp`, `orl_jit.cpp`, `orl_gpu.cpp` —
  allocate, bind, zero or leave undefined per rule.
- `src/orlexec/orl_exec.cpp`, `orl_graph_exec.cpp` — host/device
  scratch pools keyed by dispatch.
- `src/orlcomp/orl_runtime_signature.h` — scratch entries in the
  reflection list, marked non-user.
- stdlib spline (and maybe hd_id gather) rewrite.
- tests listed below.

## Design fixed in this period

1. Scratch is declared on the **exported** function, not as a user
   parameter. Import hides it. The node browser does not show a
   `chain` input.
2. Each scratch buffer has an element type and a size expression from
   a closed set: `joint_count`, a formal `int` parameter, or
   `k * that` for a small literal `k`. No `malloc(n)` computed inside
   the body.
3. Lifetime is one dispatch (CPU call or CUDA launch). Contents are
   undefined at entry unless metadata says `zero`. They do not survive
   to the next frame.
4. Scratch cannot be returned, assigned to a global, or written into
   `solver_context`. Analysis rejects escape, same spirit as
   handle-view escape.
5. CPU and CUDA see the same byte size. CUDA scratch is a device
   buffer bound like other hidden buffers, not a host pointer the
   kernel dereferences.
6. Parallel loops may write scratch `[i]` only when `i` is the
   parallel index and the size is that bound. Sequential solvers may
   use scratch as a gather list. Nested parallel + shared scratch
   races are rejected or documented as unsupported in this period
   (sequential-only scratch is enough for CCD/spline).
7. Persistent algorithm state (`hd_id` history) remains an explicit
   `Joint history[]` graph buffer. Scratch is not a hidden history.

## Context the implementation must respect

### Syntax choice

Pick one public spelling and stick to it. Two reasonable options:

**A. Export metadata (smaller frontend change):**

```orl
export int solver_spline_ik [[
    string stage = "solver",
    string scratch = "joint_handle chain[joint_count]"
]]
    (joint_handle root, joint_handle end, point spline[], int point_count)
```

The body refers to `chain` as a magic name. This is easy to parse
poorly and hard to type-check.

**B. Explicit scratch declaration (recommended):**

```orl
export int solver_spline_ik [[ string stage = "solver" ]]
    (joint_handle root, joint_handle end, point spline[], int point_count)
{
    scratch joint_handle chain[joint_count];
    ...
}
```

`scratch` is a new keyword or a statement kind. Analysis binds
`joint_count` to `solver_context.joint_count` or a formal. This is
more work in the parser and much clearer for users.

Do not use a fake parameter `joint_handle chain[]` that the importer
then hides. That collides with plan 1’s public list ports.

### Why not generic locals `T a[n]`

Unbounded dynamic locals imply a stack or heap in every function,
including helpers, including GPU. Scratch limited to the exported
entry, sized from dispatch-known symbols, matches how the runtime
already allocates `solver_context` arenas.

Helpers that need temps either take a scratch slice from the caller
or stay fixed-size. Passing scratch into a helper is allowed if it
is a buffer parameter *internal to the module* and not imported as a
graph port. Keep that module-internal for this period if it
complicates import; otherwise only the export body uses scratch.

### Sizing from `joint_count`

`solver_context.joint_count` is already on the hidden context. A
scratch sized `joint_count` is enough for any ancestor chain and for
a visited-mark array. Spline and CCD do not need a tighter bound for
correctness. A later period may allow `scratch T a[n]` where `n` is
computed and then a runtime `set_scratch_size`, but that is a second
ABI.

### GPU

`HandleDeviceContext` and other device buffers are allocated up
front. Add scratch to that list. Do not `cudaMalloc` inside the
kernel. If a program has several exported entries, size the pool to
the max of their scratch layouts for the compiled module, or
allocate per launch from the reflection table.

## How

### 1. Language

Add the `scratch` declaration (option B) at the start of an exported
function body, declaration-before-use. Multiple scratch buffers are
allowed. Names must not collide with formals or types.

### 2. Analysis

- Resolve the size symbol.
- Type indexing like a buffer parameter.
- Reject taking the address, returning the buffer, or storing it.
- If the element is a handle, apply plan 1 element rules.

### 3. Import

Function summaries include scratch descriptors for the runtime.
`NodeDefinition.inputs` does not. Graph JSON does not grow user
ports.

### 4. Runtime

On `bind` / graph launch:

- compute byte size from element ABI × count;
- allocate host or device memory;
- pass a hidden buffer pointer in the wrapper, after user buffers or
  with a dedicated scratch slot list;
- free or recycle after the launch.

Document whether contents are uninitialized. Tests must not depend
on zero unless `zero` was requested.

### 5. Pilot

Rewrite `solver_spline_ik` to gather `chain[0..n)` from `end` to
`root`, then iterate root-to-end for even `t`. Keep the public
sockets from plan 2. Numeric fixtures must match.

`hd_id` may use `scratch int visited[joint_count]` or keep using
only live pose + `history[]`. Do not replace `history[]` with
scratch.

## Work items

1. Syntax, AST, analysis, diagnostics.
2. Reflection + host/CUDA allocation and wrapper.
3. Import hides scratch; cache/ABI version bump if the wrapper
   grows.
4. Spline gather/scatter rewrite and tests.
5. Reject escape, illegal sizes, and scratch-as-graph-input.

## Tests

Extend:

- `tests/parser_tests/parser_tests.cpp`
- `tests/analysis_tests/analysis_tests.cpp`
- `tests/syntax_tests/language_syntax_tests.cpp`
- `tests/codegen_tests/codegen_tests.cpp`
- `tests/exec_tests/exec_tests.cpp`
- `tests/gpu_tests/gpu_tests.cpp`
- `tests/orlrig_tests/orlrig_tests.cpp` for spline parity

Required positive cases:

- `scratch int flags[joint_count]` compiles and is writable;
- spline gather then root-to-end matches the two-pass redesign
  numerically;
- scratch does not appear as a node input in the imported
  definition;
- CUDA parity or explicit skip.

Required negative cases:

- `scratch T a[n]` where `n` is a local computed int rejected
  (this period);
- returning scratch or assigning it to a parameter rejected;
- `scratch` on a non-exported helper rejected (or documented
  module-internal only);
- user connection to a scratch name is impossible.

## Exit criteria

- Exported solvers can gather a joint-count-sized temp on CPU.
- Scratch is invisible in the graph menu and project sockets.
- Spline no longer depends on reversed parameterization for
  correctness.
- No persistent handle or pose state is smuggled through scratch.

## Not in scope

- General heap or recursive scratch.
- Node-local persistent state (use graph buffers).
- Parallel reduction libraries.
- User-visible scratch sockets.
- Changing `history[]` into hidden state.
