# General solver DSL plans: overview

ORL aims to be a language users write their own solvers and deformers in, not
only a host for a fixed stdlib. The typed-handle cutover made that goal
stricter: a public socket that names a scene component must be a handle, not
a packed index.

That rule is correct. It also showed that the current language only covers
one class of operator well.

## Current expressiveness

A user can already write a node whose component set is known when the graph
is compiled:

- a fixed list of scalar handles;
- ordinary numeric and buffer data;
- optional access to the whole joint/locator arena through `solver_context`;
- `parallel for` over a bound that is a parameter.

Two-bone IK, aim/copy/parent constraints, LBS, and FK all fit. The leftover
stdlib solvers do not: they name components the graph did not wire, in a
count the graph did not fix.

The next language work is therefore not “migrate `int chain[]`.” It is to
add the missing user-facing capabilities those algorithms expose.

## Feature set

| Plan | Feature | Primary hole |
| --- | --- | --- |
| [1](solver_dsl_plan_1_handle_collections.md) | Handle collections | Variable-sized sets of identities |
| [2](solver_dsl_plan_2_handle_topology.md) | Handle-valued topology | Walking parent/ancestor without packed ints |
| [3](solver_dsl_plan_3_computed_handle_effects.md) | Computed-handle effects | Topology walks stay globally dirty |
| [4](solver_dsl_plan_4_scratch_working_sets.md) | Scratch working sets | No gather / reverse / local temps of runtime size |
| [5](solver_dsl_plan_5_multi_writer_regions.md) | Multi-writer regions | N nodes ≠ one multi-target kernel |

Do not reopen implicit `int <-> handle` conversion. That would make the
existing stdlib files easy and make the user DSL worse.

## Suggested implementation order

Collections is the largest hole, but not the first implementation slice.

```text
2 topology
    -> 3 computed-handle effects
        -> 4 scratch working sets
            -> 1 handle collections
                -> 5 multi-writer regions
```

Reasons:

- Plan 2 unblocks a current-language redesign of `hd_id` and `spline_ik` as
  `root` + `end` + `target` nodes, without handle buffers.
- Plan 3 makes those nodes eligible for sparse evaluation using the already
  compiled hierarchy plan.
- Plan 4 makes chain algorithms read naturally (collect, then iterate
  root-to-end) instead of reversed two-pass walks.
- Plan 1 is the ABI/graph/authoring monster. It should land after the
  scalar-handle topology story is stable.
- Plan 5 depends on declared footprints from 1–3. Until then, keep the
  current fail-closed overlapping-writer rule.

Each plan is independently shippable with its own tests and a stdlib or
user-shaped pilot. Later plans may assume earlier ones exist, but they
must not require reverting earlier public contracts.

## Shared constraints

These rules apply to every plan in this series:

1. Handles remain dispatch-local opaque tokens (`HandleValue { type_id, slot }`).
   They are not stored in project files, Joint structs, or persistent ORL
   state.
2. `Joint.parent` stays an ABI `int` in packed storage. Topology features
   wrap that slot as a handle; they do not change the 128-byte Joint layout.
3. `find_joint` / `find_locator` remain the only public way to introduce a
   scene identity into the graph.
4. Runtime-unknown identity stays conservative until plan 3 can prove a
   walk. Never guess a packed slot in the evaluation plan.
5. CPU and CUDA must use the same value widths and validity rules. A
   feature that cannot be lowered to the GPU path in the same period must
   say so and stay host-only behind an explicit capability, or wait.

## Pilot solvers

Use the leftover stdlib solvers as proofs, not as the product:

| Solver | After plan 2 | After plan 1+3 |
| --- | --- | --- |
| `hd_id` | `root`/`end`/`target` handles, global walk | ancestor footprint of `end` |
| `spline_ik` | `root`/`end` + `point spline[]` | same, plus optional explicit `joint_handle chain[]` |
| `full_body_ik` | one effector per node | one node with parallel handle lists |

A redesign that only uses plans 2–4 is enough to put `hd_id` and `spline_ik`
back in the Solver menu. `full_body_ik` as a single node waits for plan 1.

## Out of scope for the whole series

- Runtime-varying nominal handle kinds.
- Persistent handle storage or project-file handle tokens.
- Spatial queries, BVH, or closest-point runtime.
- New math intrinsics unrelated to identity and working sets.
- Replacing genuine numeric indices (vertex, weight, loop, count).
