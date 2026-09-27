# Solver DSL plan 5: multi-writer regions

## Goal

Define, and then implement, how ORL treats several solvers that write the
same joints — including the case where a user hoped “one node per
effector” would equal one multi-target kernel.

At the end of this period:

- overlapping **declared** writes still cannot be silent;
- users have one explicit way to order or group those writes;
- the docs and tests state that graph composition is not pass-major
  FBIK;
- optional later fusion is specified so it is not invented ad hoc in
  the viewer.

## What

`compile_evaluation_plan` builds `SolverRegion`s from footprints, then
adds edges:

1. an existing graph wire;
2. reader depends on writer of the same element (joint reads include
   ancestors);
3. a writer of child C depends on a writer of ancestor P.

Errors include:

- two regions write the same joint/locator and no path orders them
  (two pasted two-bone nodes);
- a **global** region overlaps a declared region and no wire orders
  them (today’s leftover solvers, and any user CCD still marked
  global).

That is fail-closed and correct for two independent IK nodes on one
leg. It is hostile to a user model of “drop three FBIK effectors on
the spine” and to stacked constraints that both write a shared
parent.

This plan is a **scheduling and product-rule** feature, not new math
syntax. It decides whether ORL’s unit of algorithm is the kernel, the
graph, or both.

## Why

Plans 1–3 let one kernel own a data-dependent joint set. Users will
still build graphs from many small nodes: several aims, a CCD, a
copy, a spline. Those nodes often write overlapping ancestors.

Today the options are:

- wire them by hand (legal, obscure);
- get a plan compile error (safe, unhelpful);
- put everything in one kernel with handle lists (plan 1), which
  restores the old FBIK interleaving but does not help mixed
  user nodes.

A general DSL must pick a visible rule. The old `full_body_ik`
inner loop was:

```text
for pass in iterations:
    for effector in effectors:
        ccd(effector)
```

N separate nodes with the same `iterations` do:

```text
for effector in effectors:          # graph / region order
    for pass in iterations:
        ccd(effector)
```

Those are different when effectors share ancestors. If we do not
say this out loud, users will treat the menu as a broken FBIK.

## Depends on

- [Eval plan extend](eval_plan_extend.md): current edge rules and
  errors.
- [Plan 3](solver_dsl_plan_3_computed_handle_effects.md): declared
  walks, so overlap is computed from sets, not from “global vs
  everything.”
- [Plan 1](solver_dsl_plan_1_handle_collections.md) if the chosen
  “real FBIK” remains one listed kernel.
- Hierarchy plan for ancestor tests already used by rule 3.

Do not implement this before plan 3. Expanding walks first makes
overlap precise; otherwise every new topology node is still global
and this plan has nothing new to order.

## Owned submodule

- `src/orlexec/orlrig/evaluation.cpp` / `evaluation.hpp` — overlap
  diagnostics, optional sequence edges, optional layer batches.
- `src/orlgraph/graph_ir.hpp` — optional sequence / layer records
  on instances or connections.
- `src/orlgraph/graph_validation.cpp` — reject illegal grouping.
- `src/orlcomp/orl_graph_import.cpp` — only if export metadata
  grows a layer/sequence tag.
- `src/orlviewer/node_graph/*` — authoring of an order edge or
  layer, if exposed.
- `docs/orlgraph.md`, `node_refs.md` — user-facing rule.
- `tests/orlrig_tests/orlrig_tests.cpp`,
  `tests/scene_graph_tests/scene_graph_tests.cpp`.

## Design fixed in this period

1. **Silent last-writer-wins is forbidden.** Two regions that write
   the same joint still require an order the plan can name.
2. The default remains fail-closed. New behavior is an *explicit*
   order, not a heuristic.
3. Graph composition of one-effector nodes is **not** defined to
   match pass-major FBIK. A multi-target pass-major algorithm is
   one kernel with handle lists (plan 1), or a documented fusion
   pass that this period only specifies.
4. Overlap is computed from expanded footprints (plan 3). Global
   nodes still overlap everyone and still need a wire or to stop
   being global.
5. Existing rule 3 (child writer depends on ancestor writer) stays.
   This plan adds a rule for **the same element written by two
   regions**.
6. No implicit blending of rotations from two writers.

## Context the implementation must respect

### Three product options (pick one to implement)

**Option A — explicit sequence edge (recommended first)**

A user (or the viewer) adds a solver-order connection: “aim runs
before ccd.” The evaluation plan treats that like rule 1 even if
there is no data wire. Diagnostics name the overlapping joint and
suggest adding a sequence edge.

This is small, visible, and matches stacked Maya-style constraints
when the user cares about order.

**Option B — solver layer parameter**

Each solver instance has `int layer` (or a string group). Regions
in increasing layer get a batch edge. Same layer + same joint =
error (still fail-closed inside a layer).

Good for “all aims, then all IK.” Bad if users forget to set
layers and still paste two IKs.

**Option C — fuse same-definition nodes**

The runtime notices N `solver_full_body_ik` one-effector nodes and
launches them as the old pass-major loop. This recovers FBIK
numerics but is definition-special and lies about what the graph
means (each node’s `iterations` is no longer “this node runs to
completion”).

Specify C as a **later optimizer** with an opt-in capability
(`fuse_pass_major = 1` on the definition). Do not enable it
implicitly in this period.

This period implements A, documents the FBIK mismatch, and leaves
B as an optional extra if sequence edges are too low-level for the
viewer. Do not implement A+B+C together.

### Why not “allow overlap and run graph order”

`topological_schedule` is node-graph order, not evaluation-plan
order. Fused JIT segments may still launch in graph order while
the dispatch plan only skips clean work
(`eval_plan_extend.md`). Using “whatever the menu pasted” as the
write order would make overlap depend on instance-id sort and
break as soon as fusion splits or merges segments.

If sequence edges are added, `build_dynamic_dispatch_plan` and
segment launch must **obey** those edges. A plan that records an
order the JIT never calls is the existing footgun; do not add
another.

### Interaction with two-bone

Two two-bone nodes on the same root/mid stay an error until the
user sequences them or deletes one. That is still the right
default: they are not cooperative FBIK, they are two full solvers
on one chain.

### Interaction with global nodes

`fk` writes every world matrix (or is arena-global). Until FK has
a declared footprint, it overlaps everyone. A sequence edge from
or to FK must be allowed so a user can say “IK then FK.” This
period should accept a sequence edge as the missing “wire” in the
global-overlap error.

## How

### 1. Write the rule in graph terms

Add a connection kind or a side table:

```text
Sequence { from: node_id, to: node_id }
```

Validation: both endpoints are solver/constraint regions; no
cycle with data + sequence edges. Sequence is not a data type and
does not lower to an ORL parameter.

### 2. Evaluation

When two regions write component X and no data/ancestor edge
orders them:

- if a sequence path orders them, emit that edge and continue;
- else error:

  ```text
  Nodes 'aim1' and 'ccd1' both write joint 'Spine'.
  Add a solver order from one to the other, or use one
  multi-target kernel.
  ```

Use joint/locator **names** from the component store, not packed
indices, in the message.

### 3. Launch order

Rebuild dispatch `levels` from the topological levels of the
evaluation graph **including** sequence edges. Split fused
segments when two sequenced regions would otherwise share a
kernel launch that ignores order. If that split is too large for
this period, refuse fusion when a sequence edge crosses a
segment.

### 4. Documentation and pilots

- `node_refs.md`: FBIK as list kernel vs N CCD nodes.
- A scene-graph test: aim + two-bone on a shared joint fails
  without a sequence edge and succeeds with one; numeric order
  matches the sequence, not instance-id sort.
- A test that two one-effector CCD nodes with `iterations > 1`
  do **not** match the old pass-major fixture (locks the
  semantic warning).

### 5. Optional layer sugar

If the viewer cannot author sequence edges yet, a `layer`
instance parameter can *generate* sequence edges from lower to
higher layer after the graph is built. Same-layer overlap still
errors. Implement only if A is done and UI still blocked.

## Work items

1. Sequence IR, serialization, validation, cycle detection.
2. Overlap diagnostic that names components and the two nodes.
3. Evaluation edges + dispatch/segment obedience.
4. Sequence-as-wire for global-overlap.
5. Docs and tests for FBIK ≠ N nodes.
6. Optional layer-to-sequence lowering.

## Tests

Extend:

- `tests/orlrig_tests/orlrig_tests.cpp`
  (`evaluation plan rejects two solvers writing the same joints`)
- `tests/scene_graph_tests/scene_graph_tests.cpp`
- `tests/graph_ir_tests/graph_ir_tests.cpp` if sequence is IR

Required positive cases:

- two overlapping solvers + sequence edge produce a legal plan;
- dispatch order follows the sequence;
- aim then two-bone on a shared ancestor matches a fixture
  recorded in that order;
- FK global + sequence from IK to FK is legal.

Required negative cases:

- two overlapping solvers and no sequence/data/ancestor order
  still error;
- sequence cycle errors;
- two one-effector CCD nodes are not silently equal to listed
  FBIK;
- no implicit rotation blend.

## Exit criteria

- Overlap has a user-visible, explicit order mechanism.
- Launch order matches the plan.
- The FBIK composition trap is tested and documented.
- Last-writer-wins without an edge does not exist.

## Not in scope

- Implicit pass-major fusion (option C), except as a written
  follow-up.
- Constraint blending or weight channels.
- Changing two-bone’s hierarchy check.
- Using packed indices in diagnostics or project files.
- New solver math.
