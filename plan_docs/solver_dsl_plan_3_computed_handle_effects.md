# Solver DSL plan 3: computed-handle effects

## Goal

Teach analysis and the evaluation plan that a handle obtained from
`joint_parent`, from a walk, or from a handle-buffer element still names
a real component, so those solvers can stop being globally dirty.

At the end of this period `hd_id` and `spline_ik` declare a footprint
equivalent to “read/write the ancestor chain of `end` through `root`,”
resolved against the compiled hierarchy plan, not against packed ints
authored on the node.

## What

Typed-handle plan 6 infers effects only from **incoming handle
parameters**. Provenance is “this parameter, this view field, this
access.” Two-bone IK fits: `root` and `mid` are written, `end` / `target`
/ `pole` are read.

A handle produced inside the function has no parameter id:

```orl
joint_handle pivot = joint_parent(end);
Joint data = Joint(pivot);
data.rotation = ...;
```

Plan 6’s rule for that case is: “a runtime-selected joint identity
remains conservative even though its nominal type is exact.” Import then
sets `partial_global`, and `compile_evaluation_plan` treats the region as
overlapping everything.

That is safe and useless for any topology-driven user solver. This plan
extends provenance from parameters to **derived handles**, then expands
those derivations with the hierarchy plan that already exists.

## Why

Without this, plans 2 and 1 only fix authoring. Every CCD, spline, or
“rotate ancestors” node still forces full evaluation and still hits the
“global overlaps declared region” error when placed next to two-bone IK.

The runtime already knows ancestors. `compile_hierarchy_plan` /
`pack_hierarchy_plan` produce parent ids, subtree ranges, and optional
flattened ancestor lists. The evaluation plan already expands two-bone
reads to include ancestors for dirty propagation. What is missing is a
compiler summary that says “this node’s writes are `ancestors(end)`,” so
that expansion can run for user code.

This is mostly analysis + evaluation, not new ORL syntax.

## Depends on

- [Typed-handle plan 6](extensible_typed_handle_plan_6.md): parameter
  handle effects.
- [Plan 2](solver_dsl_plan_2_handle_topology.md): `joint_parent` and
  validity, so there is something to track.
- [Hierarchy compilation](hierarchy_compilation.md) and
  [eval plan extend](eval_plan_extend.md): ancestor tables and region
  edges.
- [Plan 1](solver_dsl_plan_1_handle_collections.md) only for the buffer-
  element half. Ship scalar-walk footprints first if collections are not
  ready.

## Owned submodule

- `src/orlcomp/orl_analysis.h` / `orl_analysis.cpp` — derived-handle
  provenance and walk summaries.
- `src/orlcomp/orl_graph_import.cpp` — emit walk/list footprint records.
- `src/orlgraph/graph_ir.hpp` — extend `PartialEvaluationFootprint`
  without reviving `partial_*_joint_ports` strings.
- `src/orlgraph/graph_serialization.cpp` — new fields.
- `src/orlexec/orlrig/evaluation.cpp` / `evaluation.hpp` — expand walks
  via the hierarchy plan.
- `resource/stdlib/solver/hd_id.orl`, `spline_ik.orl` — drop
  `partial_global` once inferred.
- tests listed below.

## Design fixed in this period

1. Derived-handle provenance is a compiler fact, not user metadata of
   the form `partial_read_joint_ports = "end"`. Kind-specific port
   strings stay gone.
2. The first supported derivations:
   - alias: `joint_handle a = b;` (already implied by plan 6);
   - parent: `joint_parent(h)` → `Parent(h)`;
   - bounded walk: a loop that reassigns `h = joint_parent(h)` until
     invalid or until `h == root_param` → `Ancestors(h [, root])`;
   - buffer element (if plan 1 exists): `values[i]` → `Element(values)`.
3. `Ancestors(end)` means the packed joints on the parent path from
   `end` toward root, including `end`, excluding an invalid parent.
   `Ancestors(end, root)` stops at `root` and requires
   `joint_is_ancestor(root, end)` to have been proven or checked.
4. A walk the analyzer cannot classify (indirect calls, `joint_parent`
   of a handle with unknown provenance, parent stored in a struct)
   stays conservative. No slot guessing.
5. Evaluation expands `Ancestors` using the compiled hierarchy plan and
   the **bound** handle identities of `end` / `root` (from `find_joint`
   through the graph). It does not read ORL locals.
6. If `end` or `root` is unresolved at plan compile time, the region
   stays global.
7. `partial_propagation` remains algorithm metadata (`full`,
   `descendants`, `none`). Ancestor *writes* are the walk set;
   descendant *dirty* is still propagation after the write.
8. `HandleEffect` stays the record for direct parameter field access.
   Walks are a sibling record on `PartialEvaluationFootprint`, for
   example `HandleWalk { parameter, kind: Ancestors, stop_parameter }`.

## Context the implementation must respect

### Why not “just mark global false”

Clearing `partial_global` without a resolved set makes the evaluation
plan think the node writes nothing. Two CCD nodes on the same arm would
then look disjoint and could be scheduled wrong. Conservative is
mandatory until expansion succeeds.

### Why the hierarchy plan, not an ORL interpreter

The evaluation plan compiles before dispatch. It must not run the
solver to discover writes. The hierarchy is immutable for a compiled
rig (`topology_revision`). Therefore “ancestors of this bound joint”
is a static query on the plan, same as two-bone’s root-mid-end check.

If the algorithm could write a joint *not* on that ancestor path
(data-dependent branches that pick an unrelated index), the analyzer
must not emit `Ancestors`. Stdlib CCD/spline do not; a user who
indexes `solver_context.joints[k]` for an unknown `k` stays global.

### Interaction with `solver_context.joints[i]`

Arena indexing by a non-proven int is opaque. This plan does not
pretend to alias `joints[i]` with a handle. Pilots from plan 2 should
already have stopped doing that. If analysis sees arena writes that
are not tied to a derived handle, keep the node global even if a walk
was also inferred.

### Buffer lists

If `effectors[]` is the result of plan 1's `find_joints_by_name`, the
footprint is the union of `Ancestors(effectors[k])` for each proven match.
The same rule can cover a future collect node. If the list contents are
only known at dispatch, either:

- treat the buffer port as global, or
- if the name-filter pattern or a future collect node is graph-constant and
  the current scene snapshot is available, resolve at plan compile time.

Do not expand a list that the user can retarget every frame without
recompiling the graph unless the pattern/source parameters are graph
parameters that trigger a plan rebuild (as `find_joint` names do when they
change).

## How

### 1. Analysis provenance

Extend handle tracking:

- each handle-typed SSA-like local has a provenance:
  `Parameter(id)` | `Parent(inner)` | `Walk(inner, stop)` | `Element(buf)`
  | `Unknown`;
- `joint_parent` maps `Parameter/Parent/Walk` to `Parent` or folds into
  `Walk` when the assignment is in a loop;
- detect the idiomatic loop:

  ```orl
  pivot = end;
  while (handle_valid(pivot)) {
      ...
      if (pivot == root) break;
      pivot = joint_parent(pivot);
  }
  ```

  and emit `Walk(end, root)`.

Be conservative with `break` / multiple assignments. A helper that
returns `joint_parent(h)` must propagate through existing call-site
handle mappings.

### 2. Field effects on derived handles

`Joint(pivot).rotation = q` attaches a write to provenance(`pivot`),
not to a new synthetic parameter. Import lowers that to a walk record
plus the access mode and view/field.

Copying `pivot` has no data effect. Binding `Joint(pivot)` has no data
effect. Same rules as plan 6.

### 3. Graph IR

Add a small walk enum to `PartialEvaluationFootprint`. Serialize it.
Do not accept or emit `partial_*_joint_ports`.

`partial_global` is true iff any write/read remains `Unknown` or the
function still sets the metadata flag. Inferred walks should allow the
pilots to drop the flag.

### 4. Evaluation expansion

In `resolve_handle_effects` (or a sibling):

- resolve `end` / `root` to `ComponentId` via existing
  `connected_element`;
- look up packed indices in the component store;
- ask the hierarchy plan for the parent path;
- fill `read_joints` / `write_joints` / `affected_joints`;
- apply `partial_propagation` as today.

If the two-bone-style hierarchy check fails (`root` is not an ancestor
of `end`), report a compile error on that node, as two-bone already
does for root-mid-end.

### 5. Pilots

Remove `int partial_global = 1` from redesigned `hd_id` and
`spline_ik` once tests show inferred walks. Keep `stateful` on
`hd_id`. Keep spline/hd_id numeric fixtures.

## Work items

1. Provenance lattice and loop classification in analysis.
2. Footprint walk records + import + serialization.
3. Evaluation expansion through the hierarchy plan.
4. Fail-closed unresolved / unclassifiable walks.
5. Drop global flags on the plan 2 pilots.
6. Optional: proven collect-list union if plan 1 is present.

## Tests

Extend:

- `tests/analysis_tests/analysis_tests.cpp`
- `tests/rig_graph_tests/rig_graph_tests.cpp`
- `tests/orlrig_tests/orlrig_tests.cpp`
- `tests/graph_lowering_tests/graph_lowering_tests.cpp`
- `tests/scene_graph_tests/scene_graph_tests.cpp`

Required positive cases:

- `Joint(joint_parent(end)).rotation = ...` is a write to
  `Parent(end)`, not global;
- the idiomatic ancestor loop emits `Ancestors(end, root)`;
- evaluation-plan joint write set equals the hierarchy path of the
  bound end/root;
- two-bone on a disjoint chain can coexist with spline/hd_id without
  “global overlaps declared”;
- dirtying only an unrelated joint does not dispatch the spline node.

Required negative cases:

- `solver_context.joints[k]` write with unknown `k` forces global;
- unresolved `find_joint` on `end` forces global or a plan error;
- `root` not ancestor of `end` is a plan compile error;
- no kind-specific port metadata in imported definitions;
- guessing a slot from an integer parameter is impossible.

## Exit criteria

- Topology-walk stdlib nodes have declared, expandable footprints.
- Evaluation order uses those sets in the existing three edge rules.
- Unclassifiable user code remains conservative.
- No return of comma-separated joint-port metadata.

## Not in scope

- Data-dependent writes to unrelated joints (stay global).
- Fusing N nodes into one pass-major kernel (plan 5).
- Interpreting ORL at plan compile time.
- Children / descendant *write* sets other than existing
  `partial_propagation = "descendants"`.
