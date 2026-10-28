# Execution-Stack Ordering and Unresolved-Plan Resolution

## Status and dependency

This plan is the first of two related rig-evaluation features.

1. Implement and stabilize this execution-stack ordering feature.
2. Only after this feature is complete and its plan/runtime tests pass, begin
   the transform-hierarchy feature described in
   `plan_docs/transform_hierarchy_plan.md`.

The execution-stack feature must therefore provide the ordering and conflict
model that the later transform hierarchy can consume. The hierarchy feature
must not invent a second, incompatible ordering mechanism.

## Goal

Make evaluation-plan compilation resolve ordinary constraint ordering
automatically and provide a structured, user-visible resolution flow when
ordering cannot be proven safe.

The result should be:

- dependency order is inferred from graph dataflow, declared read/write
  footprints, resource effects, and hierarchy relationships;
- constraints that write the same joint or locator have a deterministic local
  stack;
- a persisted explicit order is used only for the affected destination
  stack, not as a global order over every graph node;
- unresolved conflicts stop plan publication and JIT execution;
- the viewer explains the conflict and lets the user choose the order;
- the choice is stored as graph authoring data and survives project reload;
- changing an order invalidates and recompiles only the affected evaluation
  plan/artifacts;
- an actual dependency cycle remains an error rather than being “fixed” by
  assigning a priority.

## Non-goals

This feature does not yet add a parent relationship for locators. It must,
however, expose an API that the later transform hierarchy can use as a base
layer and as a source of dependency edges.

It also does not:

- silently choose an arbitrary order for non-commutative competing writers;
- replace graph dataflow edges with order metadata;
- make every solver globally sequential;
- change the packed joint or locator ABI;
- make the GPU kernel responsible for discovering ordering;
- use a message box as the only long-term conflict editor;
- make evaluation state part of project serialization.

## Current architecture and constraints

The current implementation already has most of the necessary building
blocks:

- `orlrig::compile_evaluation_plan` in
  `src/orlexec/orlrig/evaluation.cpp` constructs `SolverRegion` objects,
  resolves handle effects, adds implicit writer/reader dependencies, adds
  joint ancestor ordering, reports competing writers, and topologically
  batches regions.
- `orlrig::EvaluationPlan` in
  `src/orlexec/orlrig/evaluation.hpp` stores `regions`, `order`, `batches`,
  and reverse maps for controllers, locators, and joints.
- `PartialEvaluationFootprint` in `src/orlgraph/graph_ir.hpp` describes
  element-level read/write effects.
- `Connection::sequence` is already a graph-level ordering edge and is
  preserved by graph validation, optimization, serialization, and lowering.
- `GraphSceneRuntime` carries the evaluation-plan order into temporary
  lowered segments through ordering-only sequence edges.
- `NodeGraphEditor` already has explicit graph-sequence editing operations.
- `SceneGraphContext::ensure_evaluation_plan` fails closed when plan
  compilation returns errors.
- `GraphSceneRuntime::report_compile_error` already has one-per-revision
  diagnostic behavior and a Qt warning surface.

The new feature must preserve those behaviors. In particular, an explicit
constraint stack is not the same thing as an arbitrary `Connection::sequence`
edge. A sequence edge orders graph nodes globally within the graph. A
constraint stack orders only competing writers of a particular transform
destination.

## Terminology

### Base transform

The transform available before the constraint stack. In the first feature this
is the existing authored joint/locator value and current joint hierarchy
propagation. In the later feature it will also include locator transform
parenting.

### Constraint writer

A solver or constraint region that writes one or more joints, locators,
controllers, or other declared resources.

### Conflict set

The set of regions that write the same non-commutative destination element.
The set is formed independently for each joint, locator, controller, or
resource. A constraint that writes two destinations participates in both
corresponding conflict sets.

### Stack order

The relative order of writers in one conflict set. It is not a global
execution number. Regions that do not share a destination or dependency do
not need to be ordered by this metadata.

### Hard dependency

An edge that cannot be overridden by user priority, such as a graph dataflow
edge, a read-after-write dependency, a hierarchy parent-before-child edge, or
an explicit sequence edge.

### Order hint / explicit order

Persisted authoring metadata used to resolve an otherwise unordered conflict
set. It is a tie-breaker for competing writers and must never contradict a
hard dependency.

## Proposed data model

### Constraint instance metadata

Add an optional per-instance order field to the editable graph representation.
The first implementation can use a nullable signed integer on `NodeInstance`,
for example:

```text
execution_order: optional signed integer
```

The field is meaningful only for solver/constraint instances and only when the
instance participates in a conflict set. Lower values run earlier. The exact
field name should be selected once and used consistently in:

- `NodeInstance`;
- graph JSON;
- graph fingerprints;
- editor property/stack UI;
- evaluation-plan diagnostics;
- lowering/runtime cache invalidation.

Do not put this field into a node definition. Definitions are shared by many
instances; ordering belongs to the particular constraint instance.

Do not treat this value as an ORL function parameter. It is compile-time graph
metadata, not dynamic rig data and not a GPU input.

If a later implementation needs different order positions for one constraint
writing multiple destinations, replace the scalar with a destination-scoped
stack record. The first version may reject that case as unresolved rather than
silently assigning inconsistent orders.

### Optional richer representation

The implementation should keep the internal API open for a richer record:

```text
ConstraintOrderEntry {
    destination component ID or resource ID
    writer node ID
    rank
}
```

The scalar `NodeInstance.execution_order` is sufficient for the first version
only when one constraint's ordering is valid for every destination it writes.
The compiler should validate that assumption and report an unresolved
destination-specific conflict if it is not.

### Structured compiler result

Extend `EvaluationPlanCompileResult` so unresolved ordering is distinguishable
from a malformed graph, a dependency cycle, or a backend failure.

The result should conceptually contain:

```text
status:
    Success
    InvalidGraph
    UnresolvedOrder
    DependencyCycle

errors:
    hard compiler diagnostics

order_conflicts:
    structured unresolved conflict records

plan:
    present only when executable and fully resolved
```

Each conflict record should contain:

- destination kind: joint, locator, controller, or resource;
- destination stable/component ID;
- human-readable destination name;
- competing writer node IDs and names;
- each writer's resolved read/write footprint;
- existing hard predecessor/successor relationships;
- available order hints;
- candidate choices, such as `A before B` and `B before A`;
- a reason that automatic ordering was insufficient;
- whether the conflict is a pairwise ambiguity or a larger stack.

Do not force the UI to parse compiler strings to reconstruct this information.
Strings remain useful for logs and legacy diagnostics, but the resolver needs
structured data.

## Compile algorithm

### Step 1: Freeze the graph and collect revisions

At the start of plan compilation, capture:

- graph edit revision;
- evaluation revision;
- scene-input/topology revision;
- hierarchy-plan revision;
- graph fingerprint;
- backend capability revision if it affects the plan.

The artifact must be published only if all captured revisions still match.
Compilation occurs while the viewer is in the existing compiling/frozen state.

### Step 2: Validate graph structure first

Run the existing graph validation and graph topological schedule:

1. validate node definitions, ports, required inputs, stage legality, and
   connection shape/type;
2. validate feedback and sequence-edge rules;
3. compute the graph schedule;
4. stop immediately on a graph cycle or malformed sequence edge.

A constraint order field cannot repair a graph dataflow cycle.

### Step 3: Build solver regions

Retain the existing `SolverRegion` construction:

- identify solver/constraint/footprint-bearing nodes;
- resolve typed handle effects;
- resolve ancestor/parent walks;
- collect reads and writes;
- mark opaque/global/stateful regions conservatively;
- compute affected joint regions;
- build packed indices;
- populate reverse reader/writer maps.

The order resolver must operate on these resolved regions, not on raw node
names or UI selection order.

### Step 4: Add all hard dependency edges

Build the dependency graph before considering order hints. Add:

1. graph dataflow edges between solver regions;
2. existing explicit `Connection::sequence` edges;
3. writer-to-reader edges for joints, controllers, locators, and resources;
4. existing joint ancestor/descendant writer ordering;
5. hierarchy/base-layer edges once the later transform hierarchy is present;
6. any stateful or barrier edges;
7. backend/runtime barriers required by segmented or fused execution.

Hard edges are authoritative. A user order that contradicts one must produce
an invalid-order diagnostic rather than reverse the edge.

### Step 5: Identify conflict sets

For each destination element:

1. collect all regions that write that element;
2. remove self-pairs;
3. determine which pairs are already ordered by reachability in the hard
   dependency graph;
4. determine whether the operations are explicitly declared commutative or
   disjoint at the field level;
5. keep only unordered, non-commutative writers as unresolved candidates.

If two writers affect different fields and the footprint proves they do not
interfere, no stack order is required. If the footprint is opaque, keep the
conservative conflict.

### Step 6: Apply persisted order hints

For every conflict set:

1. read the participating instance order metadata;
2. compare only writers in that conflict set;
3. add a candidate edge from lower rank to higher rank;
4. verify that no candidate edge contradicts a hard dependency;
5. detect equal, missing, or inconsistent ranks;
6. detect a cycle introduced by the hints.

The field is a preference only until validated. It is not sufficient to append
nodes to a global vector and assume that vector is executable.

Examples:

- `constraint_A` writes locator `pole` with order 10 and
  `constraint_B` writes locator `pole` with order 20: add `A -> B`.
- `constraint_A` writes joint `hip` and `constraint_B` writes locator `pole`:
  their order fields do not order them unless another dependency connects
  them.
- `A -> B` is a hard reader/writer edge but the order fields request `B -> A`:
  report a contradiction.
- two constraints have no order and no dependency: report an unresolved
  conflict rather than picking graph insertion order.

### Step 7: Resolve deterministic safe cases

Automatic resolution is allowed only when semantics prove the result:

- one writer is already an ancestor/dependency of the other;
- one writer is the base/hierarchy layer and the other is a constraint layer;
- a declared stack layer fixes the order;
- the writes are disjoint or explicitly commutative;
- a previously persisted valid rank resolves the conflict.

Do not use stable node ID, creation order, or current vector order as a silent
semantic decision. These values may be used as deterministic display order and
diagnostic tie-breakers, but not as an unannounced transform decision.

### Step 8: Detect unresolved conflicts and cycles

Return `UnresolvedOrder` when:

- two or more non-commutative writers share a destination;
- no hard path orders them;
- order metadata is absent, equal, inconsistent, or ambiguous;
- no registered composition rule can combine them.

Return `DependencyCycle` or a hard compile error when:

- hard edges form a cycle;
- an explicit order contradicts hard dependencies and creates a cycle;
- a constraint's read/write footprint creates feedback;
- a global/opaque region cannot be legally placed relative to another
  writer.

The compiler must not produce an executable `EvaluationPlan` for either case.

### Step 9: Produce the final order and batches

After all edges are resolved:

1. topologically sort solver regions;
2. use stable IDs only to make equivalent, already-safe choices deterministic;
3. produce `EvaluationPlan::order`;
4. produce parallel `EvaluationPlan::batches`;
5. retain dependency lists on each `SolverRegion`;
6. populate reverse dirty maps;
7. include validated order metadata in the dependency revision/fingerprint.

Equivalent independent regions may share a batch. A stack edge must prevent
two competing writers from sharing a batch.

### Step 10: Publish atomically

`SceneGraphContext::ensure_evaluation_plan` should:

- keep the previous executable plan only for the already-running frame;
- not publish a new plan when unresolved conflicts exist;
- expose the structured conflict result to the viewer;
- clear the old plan before a rebuild if using the previous plan would write
  stale data;
- publish the new plan only after all revisions and signatures match.

The runtime must not enter JIT lowering when the result is unresolved.

## Runtime and JIT integration

### Failed-closed behavior

`GraphSceneRuntime::dispatch_graph` must stop before:

- `ensure_execution_plan`;
- `build_orl_segment`;
- fused CUDA artifact creation;
- segmented fallback execution.

The existing failure path should be extended to distinguish:

- invalid graph;
- unresolved constraint order;
- dependency cycle;
- JIT/backend failure.

All of them leave evaluation disabled or waiting for resolution; none may
execute a partial order.

### Execution-plan ordering

The existing segment builder already carries evaluation-plan region order into
temporary graph sequence edges. Update the order source so it uses the
resolved stack-aware `EvaluationPlan::order`.

Verify that:

- fused solver segments add sequence edges for the complete resolved region
  order;
- segmented fallback is split at barriers where the resolved order requires
  a separate launch;
- runtime-node adapters cannot bypass a required constraint stack edge;
- CPU, CUDA, fused, and fallback paths observe the same region order.

### Fusion rules

The existing fused CUDA eligibility rules remain in force. A constraint stack
edge is a scheduling dependency, not a reason to add a host readback.

For an eligible stack:

- order calls inside the single fused entry function;
- retain the stack sequence in lowering metadata;
- keep intermediate joint/locator state in the packed device arena;
- publish only the final device views;
- use segmented fallback if a barrier or host adapter prevents fusion.

The fusion diagnostic must identify when an unresolved order prevents fusion.
It must not silently fuse in graph insertion order.

## Viewer resolution workflow

### Compile-state model

Add a distinct waiting state conceptually named `AwaitingOrder` or
`OrderResolutionRequired`. It is different from a failed compile:

- `Compiling`: JIT/plan build is running; graph is frozen.
- `AwaitingOrder`: plan is valid except for user-resolvable order conflicts;
  resolver UI is available.
- `Evaluating`: executable plan is active; graph editing remains frozen.
- `Disabled`: no active evaluation; normal editing is available.
- `Failed`: hard compile/runtime failure; evaluation is disabled.

The existing overlay/state code should be extended rather than making the
compiler block indefinitely in a modal message box.

### First usable UI

Implement a focused resolver dialog or dock panel:

- title: `Constraint order required`;
- list of unresolved destinations;
- for each destination, show competing constraints and affected output;
- present candidate order buttons or a reorderable stack;
- show a preview of the selected order;
- show whether the selected choice will force a barrier or disable fusion;
- buttons: `Apply`, `Apply all similar`, `Cancel/Disable Evaluation`.

The dialog must not let the user edit arbitrary graph topology while the plan
is unresolved. It may provide only ordering controls and a link to the
affected node/constraint selection.

For multiple conflicts, apply all choices as one transaction. Recompile once
after the transaction rather than rebuilding after every row.

### Interaction rules

On `Apply`:

1. validate that the selected order does not contradict hard dependencies;
2. write order metadata to the graph instance(s);
3. touch the graph/evaluation revision;
4. close or update the resolver;
5. rebuild the evaluation plan;
6. transition to `Evaluating` only after successful JIT/artifact creation.

On `Cancel`:

1. leave order metadata unchanged;
2. disable evaluation;
3. clear device views if required;
4. unlock the graph editor;
5. preserve the diagnostic for the next explicit enable attempt.

The resolver must be safe if an authoring edit invalidates the conflict while
it is open. In that case discard the stale choices and rebuild the diagnostic.

### Existing explicit sequence operation

Keep the existing graph-level sequence operation for deliberate ordering of
arbitrary graph nodes. The new resolver should not create synthetic sequence
connections for every constraint conflict by default.

If the user explicitly requests “convert this choice to graph sequence,” it
may create a sequence edge as an advanced action, but the normal persisted
representation should remain constraint-stack metadata. This prevents a local
destination decision from imposing unrelated global order.

## Serialization and project persistence

### Graph JSON

Serialize the order metadata with the node instance, not the node definition.
The field should be omitted when unset and written deterministically when set.

Because the current graph serializer uses exact format-version checks, update
the graph format version and add a migration path:

1. read old graphs with no order metadata as unresolved when necessary;
2. preserve all existing node/connection data;
3. write the new field only for graphs using the new format;
4. do not serialize transient `RuntimeConfig::evaluate_orl` state;
5. include order metadata in canonical serialization and content hashes.

Project-level load/save should delegate order data to the graph serializer.
No separate viewer-only evaluation-state field should be introduced.

### Backward compatibility

Old projects remain valid. Their constraints receive no explicit order and
will compile automatically when dependencies prove the order. If they contain
an actual unresolved conflict, opening the file is allowed but enabling
evaluation displays the resolver.

Malformed values must be diagnostics:

- non-integer order;
- duplicate/contradictory destination-scoped ranks;
- order metadata attached to a non-constraint node;
- a serialized order that contradicts current graph/scene handles.

## Revision and invalidation rules

An order metadata change invalidates:

- graph edit revision;
- evaluation revision;
- evaluation-plan signature;
- execution-plan/fingerprint cache;
- fused CUDA artifact cache;
- dirty dispatch metadata if it embeds region order.

It must not invalidate:

- mesh assets;
- static GPU draw resources unrelated to solver state;
- the transform hierarchy plan unless the later hierarchy feature reports a
  dependency change.

Adding/removing a constraint, changing its read/write handles, changing a
constraint type, changing graph sequence edges, or changing hierarchy
relationships all invalidates the order plan.

Controller/locator pose changes do not rebuild the stack; they only create a
dirty change set after the plan is published.

## Diagnostics

Every diagnostic should identify stable IDs and user-facing names:

```text
Unresolved order for locator 'loc_leg_l_pole':
  writers: 'pole_parent' and 'pole_offset'
  no dependency or explicit stack order determines which writer runs first
```

For contradictions:

```text
Invalid order for joint 'leg_l_hip':
  explicit order requests 'constraint_B' before 'constraint_A',
  but 'constraint_A' reads a value written by 'constraint_B'
```

For cycles:

```text
Constraint dependency cycle:
  A writes locator X and reads locator Y
  B writes locator Y and reads locator X
```

Keep the same diagnostic in:

- compiler result;
- console log;
- resolver UI;
- optional runtime HUD/status text.

Deduplicate repeated display by graph revision, evaluation revision, and
diagnostic fingerprint.

## Implementation phases

### Phase 0: Baseline and test fixtures

- Capture current evaluation-plan behavior in tests.
- Add fixtures with two constraints writing one locator, two constraints
  writing one joint, independent writers, writer/reader chains, and cycles.
- Verify existing sequence edges and fused lowering remain unchanged.
- Define stable component/node labels used by diagnostics.

Exit criteria:

- current tests pass;
- fixtures reproduce unresolved competing writers deterministically.

### Phase 1: Persisted instance order metadata

- Extend `NodeInstance` with optional order metadata.
- Update graph equality/fingerprints.
- Update graph JSON serialization/deserialization and version migration.
- Add editor model accessors without changing the UI yet.

Exit criteria:

- unset metadata round-trips identically;
- set metadata round-trips deterministically;
- old graph files load.

### Phase 2: Structured conflict result

- Add conflict/status structures to `EvaluationPlanCompileResult`.
- Keep current string errors for compatibility.
- Refactor existing competing-writer diagnostics to structured records.
- Include order metadata in evaluation dependency revision.

Exit criteria:

- compiler distinguishes resolved, unresolved, and cyclic cases;
- no unresolved result contains an executable plan.

### Phase 3: Automatic stack resolution

- Build conflict sets.
- Apply hard dependency edges.
- Apply order hints.
- Validate contradictions and cycles.
- Generate final order and batches.
- Keep independent regions parallel.

Exit criteria:

- inferred safe ordering works without metadata;
- explicit order resolves only the intended destination stack;
- unrelated constraints are not globally serialized.

### Phase 4: Runtime/lowering integration

- Stop dispatch/JIT on unresolved order.
- Carry resolved order into fused and segmented execution.
- Invalidate and rebuild artifacts when order changes.
- Add CPU/CUDA/fallback parity tests.

Exit criteria:

- no path executes an unresolved plan;
- final transforms match the selected order across backends;
- fused and fallback paths agree.

### Phase 5: Resolver UI

- Add `AwaitingOrder` state.
- Add resolver panel/dialog.
- Freeze graph topology while unresolved.
- Apply choices transactionally and trigger one rebuild.
- Add cancel/disable behavior and diagnostics.

Exit criteria:

- a user can resolve a conflict without editing JSON;
- the next compile succeeds without restarting the viewer;
- cancel leaves the graph usable and evaluation disabled.

### Phase 6: Harden and hand off to transform hierarchy

- Add project reload/save tests.
- Add stale-resolver and revision-race tests.
- Add performance measurements for large conflict sets.
- Document the stable ordering API for the transform hierarchy.
- Confirm no hierarchy implementation starts before all phase gates pass.

## Test matrix

### Compiler tests

- one writer, no order required;
- two independent writers to different locators;
- reader depends on writer automatically;
- parent/child joint writer ordering;
- two competing locator writers with no order -> unresolved;
- two competing joint writers with explicit order -> resolved;
- equal/missing order -> unresolved;
- order contradiction -> hard error;
- explicit order cycle -> cycle error;
- opaque/global solver conflict -> unresolved or conservative full plan
  according to declared policy;
- commutative/disjoint writes do not prompt.

### Serialization tests

- unset order omitted;
- set order round-trips;
- old version migration;
- deterministic field order/content hash;
- malformed order diagnostics;
- project open does not restore evaluation enabled state.

### Runtime tests

- unresolved plan does not call JIT;
- unresolved plan does not publish a device view;
- applying an order invalidates the old execution artifact;
- cancellation unlocks authoring;
- selected order is honored by CPU execution;
- selected order is honored by CUDA segmented execution;
- selected order is honored inside fused CUDA lowering;
- independent regions remain parallel.

### Viewer tests

- resolver lists all conflicts;
- selecting A-before-B writes correct metadata;
- Apply recompiles once;
- Cancel disables evaluation;
- stale resolver choices are rejected after graph edits;
- diagnostics identify destination and writers.

## Acceptance criteria

The feature is ready for the transform-hierarchy implementation when:

1. ordinary dependency order is automatic;
2. competing same-destination writers cannot execute ambiguously;
3. explicit order is scoped to relevant conflict sets;
4. hard dependencies and cycles cannot be overridden;
5. unresolved plans fail closed and are user-resolvable;
6. order choices persist as graph metadata;
7. CPU, segmented CUDA, fused CUDA, and fallback execution agree;
8. graph editing/evaluation state transitions remain correct;
9. existing graph sequence behavior remains compatible;
10. all compiler, serialization, runtime, and viewer tests pass.

Only after these criteria are met should transform-parent relationships be
introduced. The next feature will use this stack as the destination-writer
composition layer rather than adding a separate priority system.
