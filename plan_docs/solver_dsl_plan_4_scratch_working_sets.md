# Solver DSL plan 4: dispatch-sized working sets

## Goal

Give solver and deformer authors a dispatch-sized temporary that is not a
graph-authored input and not a compile-time `Type values[N]`.

At the end of this period a user can gather a parent chain, reverse it,
and iterate root-to-end without encoding the algorithm as two awkward
walks:

```orl
export int solver_spline_ik [[ string stage = "solver" ]]
    (joint_handle root,
     joint_handle end,
     point spline[],
     int point_count)
{
    // A non-literal, approved bound denotes a dispatch-sized working array.
    joint_handle chain[joint_count];

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

There is no new keyword in this syntax. The compiler classifies the
declaration from its bound expression.

## What

ORL locals are scalars, fixed arrays, or approved dispatch-sized arrays.
Buffer parameters are caller-owned graph or runtime inputs
(`history[]`, `spline[]`, `world[]`). A dispatch-sized local is a
compiler-owned, dispatch-scoped working array; it is not a graph input.

That is why the plan 2 spline redesign must count, then walk again with
reversed `t`. It is also why `hd_id` uses a **graph** `Joint history[]`
for warm-start state: the language has no node-local persistent buffer
either.

This plan adds **dispatch-scoped working arrays** using ordinary local-array
syntax. The graph/runtime resolves their numeric capacity before launch from
a size expression the compiler can classify (`joint_count`, a parameter
count, or a simple multiple).

Working arrays may hold:

- ordinary data (`int`, `float`, `vector`, `point`, `Joint`);
- exact handles, if [plan 1](solver_dsl_plan_1_handle_collections.md)
  has landed.

They are not:

- `malloc` in the kernel;
- a persistent history that survives the next frame (that stays an
  explicit graph buffer, like `history[]`);
- a back door to store handles in the project.

## Why

### Critical boundary: dispatch-sized, not kernel-dynamic

“Dispatch-sized” means that the numeric capacity may vary between graph
dispatches. It does **not** mean that the kernel can compute a size and then
allocate or resize an array:

```orl
joint_handle chain[joint_count]; // allowed: resolved before launch
int flags[item_count];            // allowed: formal bound is pre-launch

int n = calculate_length();
int values[n];                    // rejected in this period
```

The compiler must classify the bound as an approved pre-launch expression
and emit a hidden working-array descriptor. The graph/runtime then resolves
`joint_count` or the formal parameter, allocates host/device storage, and
launches the kernel with a fixed pointer and capacity. The kernel may index
that storage, but it cannot allocate, resize, or infer a new capacity from
in-kernel values. Inferring bounds from arbitrary loops or branches is also
out of scope.

User solvers gather, then scatter:

- build a root-to-end chain;
- store previous directions for a pole or swing limit;
- mark visited joints in a CCD pass;
- deformers that need a prefix-sum or a per-vertex temp.

Forcing every temp to be a graph input pollutes the node API
(`temporary0[]` sockets the user must not wire). Forcing fixed `N`
caps chain length in the language. Reversed two-pass walks are a
compiler trick, not a DSL.

GPU kernels cannot heap-allocate per thread in this stack. The
runtime must size working arrays **before** launch from hierarchy or
parameter counts that are already known at dispatch.

## Depends on

- Existing buffer ABI and size symbols (`<name>_count`, `joint_count`).
- [Plan 2](solver_dsl_plan_2_handle_topology.md) for the spline/hd_id
  rewrite that will consume runtime working arrays.
- [Plan 1](solver_dsl_plan_1_handle_collections.md) if working-array
  elements are handles. Ship scalar/data arrays first if collections are
  late.
- Solver context / handle context already passed as hidden pointers.

## Owned submodule

- `src/orlcomp/orl_parser.cpp` / `orl_ast.h` — dispatch-sized local-array
  bounds and AST representation.
- `src/orlcomp/orl_analysis.cpp` — size checking, no escape.
- `src/orlcomp/orl_graph_import.cpp` — do **not** import working arrays as
  user-visible ports.
- `src/orlcomp/orl_codegen.cpp`, `orl_jit.cpp`, `orl_gpu.cpp` —
  lower, allocate, bind, zero or leave undefined per rule.
- `src/orlexec/orl_exec.cpp`, `orl_graph_exec.cpp` — host/device
  working-array pools keyed by dispatch.
- `src/orlcomp/orl_runtime_signature.h` — hidden working-array entries in the
  reflection list, marked non-user.
- stdlib spline (and maybe hd_id gather) rewrite.
- tests listed below.

## Design fixed in this period

1. Dispatch-sized arrays use ordinary local-array syntax on the
   **exported** function, not a new keyword and not a user parameter.
   Import hides them. The node browser does not show a `chain` input.
2. A literal bound keeps the existing fixed-array meaning. A non-literal
   bound is dispatch-sized only when it is from a closed set:
   `joint_count`, a formal `int` parameter, or `k * that` for a small
   literal `k`. Arbitrary local expressions and `malloc(n)` are rejected.
   Exact handle locals are accepted only for an approved dispatch-sized
   bound; fixed literal handle arrays remain disallowed.
3. Lifetime is one dispatch (CPU call or CUDA launch). Contents are
   undefined at entry unless metadata says `zero`. They do not survive
   to the next frame.
4. A dispatch-sized local array cannot be returned, assigned to a global,
   or written into
   `solver_context`. Analysis rejects escape, same spirit as
   handle-view escape.
5. CPU and CUDA see the same byte size. Runtime working arrays use a
   device buffer bound like other hidden buffers, not a host pointer the
   kernel dereferences.
6. Parallel loops may write a working array `[i]` only when `i` is the
   parallel index and the size is that bound. Sequential solvers may
   use working arrays as a gather list. Nested parallel loops that share
   one working array are rejected or documented as unsupported in this period
   (sequential-only working arrays are enough for CCD/spline).
7. Persistent algorithm state (`hd_id` history) remains an explicit
   `Joint history[]` graph buffer. A dispatch-sized local array is not
   hidden history.

## Context the implementation must respect

### Syntax choice

Use normal array declarations. The bound expression determines the storage
class:

```orl
export int solver_spline_ik [[ string stage = "solver" ]]
    (joint_handle root, joint_handle end, point spline[], int point_count)
{
    joint_handle chain[joint_count];
    int flags[point_count];
    ...
}
```

The compiler classifies `chain` and `flags` as dispatch-sized working
arrays because their bounds are approved pre-launch size expressions. The
`joint_count` symbol is resolved from hidden `solver_context.joint_count`;
it is not an undeclared local variable. `point_count` is an ordinary
formal parameter.

The same syntax retains existing meanings:

```orl
int fixed_values[32];       // fixed compile-time array
point spline[];             // buffer parameter
int temporary[n];           // rejected if n is a computed local
```

Do not use a fake parameter `joint_handle chain[]` that the importer
then hides. That collides with plan 1’s public list ports.

### Why not infer an arbitrary bound from array uses

The compiler must allocate storage before launching a GPU kernel. It cannot
reliably infer a maximum from arbitrary writes such as `values[n]` after
`n` is computed through branches, loops, aliases, or helper calls.

Therefore only the approved bound forms become dispatch-sized arrays. This
keeps the syntax ordinary without turning every local array into an
implicit heap allocation.

Helpers that need temps either take a working-array slice from the caller
or stay fixed-size. Passing a working-array slice into a helper is allowed if it
is a buffer parameter *internal to the module* and not imported as a
graph port. Keep that module-internal for this period if it
complicates import; otherwise only the export body uses dispatch-sized arrays.

### Sizing from `joint_count`

`solver_context.joint_count` is already on the hidden context. An array
sized `joint_count` is enough for any ancestor chain and for
a visited-mark array. Spline and CCD do not need a tighter bound for
correctness. A later period may allow a runtime setter for computed
capacity, but that would be a second ABI.

### GPU

`HandleDeviceContext` and other device buffers are allocated up
front. Add working arrays to that list. Do not `cudaMalloc` inside the
kernel. If a program has several exported entries, size the pool to
the max of their working-array layouts for the compiled module, or
allocate per launch from the reflection table.

## How

### 1. Language

Allow a local array declaration to use an approved non-literal bound.
Literal bounds retain fixed-array semantics. Runtime-sized declarations
must be in an exported entry body, use declaration-before-use, and have
names that do not collide with formals or types. No new keyword is
introduced.

### 2. Analysis

- Resolve and classify the size expression as a pre-launch bound.
- Bind `joint_count` to the hidden solver context and formal names to
  integer parameters.
- Type indexing like a buffer parameter.
- Reject arbitrary computed bounds and in-kernel capacity inference in this
  period.
- Reject taking the address, returning the array, or storing it.
- If the element is a handle, apply plan 1 element rules and require an
  approved dispatch-sized bound.

### 3. Import

Function summaries include hidden runtime-array descriptors for the
runtime. `NodeDefinition.inputs` does not. Graph JSON does not grow user
ports.

### 4. Runtime

On `bind` / graph launch:

- compute byte size from element ABI × the pre-launch count;
- allocate host or device memory;
- pass hidden buffer pointers in the wrapper, after user buffers or
  with a dedicated working-array slot list;
- free or recycle after the launch.

Document whether contents are uninitialized. Tests must not depend
on zero unless `zero` was requested.

### 5. Pilot

Rewrite `solver_spline_ik` to gather `chain[0..n)` from `end` to
`root`, then iterate root-to-end for even `t`. Keep the public
sockets from plan 2. Numeric fixtures must match.

`hd_id` may use `int visited[joint_count]` or keep using
only live pose + `history[]`. Do not replace `history[]` with
the dispatch-sized local array.

## Work items

1. Expression-sized local-array syntax, AST, analysis, diagnostics.
2. Reflection + host/CUDA allocation and wrapper.
3. Import hides dispatch-sized arrays; cache/ABI version bump if the wrapper
   grows.
4. Spline gather/scatter rewrite and tests.
5. Reject escape, illegal sizes, and runtime arrays as graph inputs.

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

- `int flags[joint_count]` compiles and is writable;
- spline gather then root-to-end matches the two-pass redesign
  numerically;
- the dispatch-sized array does not appear as a node input in the imported
  definition;
- CUDA parity or explicit skip.

Required negative cases:

- `T a[n]` where `n` is a local computed int is rejected
  (this period);
- returning the array or assigning it to a parameter is rejected;
- dispatch-sized arrays on non-exported helpers are rejected (or
  documented module-internal only);
- user connection to an array name is impossible.

## Exit criteria

- Exported solvers can gather a joint-count-sized temp on CPU.
- Runtime-sized arrays are invisible in the graph menu and project sockets.
- Spline no longer depends on reversed parameterization for
  correctness.
- No persistent handle or pose state is smuggled through a local array.

## Not in scope

- General heap or recursive dynamic allocation.
- Node-local persistent state (use graph buffers).
- Parallel reduction libraries.
- User-visible working-array sockets.
- Changing `history[]` into hidden state.
