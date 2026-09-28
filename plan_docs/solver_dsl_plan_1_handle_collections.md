# Solver DSL plan 1: handle collections

## Goal

Let an exported ORL function take a homogeneous buffer of exact handles, and
let the graph bind that buffer from authored scene identities. This is the
language form of “these N joints” or “these N locators.”

At the end of this period a user can write:

```orl
export int solver_full_body_ik [[
    string stage = "solver",
    string partial_propagation = "full"
]]
    (joint_handle effectors[],
     locator_handle targets[],
     int effector_count,
     int iterations);
```

and the graph can supply those lists without packing slot integers.

A handle buffer is only the kernel type. This period also provides one
concrete **list-valued identity source** so users can obtain a collection:
`find_joints_by_name`, which carries the subset of scene joints whose names
match a user-supplied regular expression.

## What

Today a handle is a scalar. Parser and analysis reject:

- `joint_handle chain[]` as a parameter (`Handle parameters cannot be buffers`);
- `handle values[N]` and handle fields on structs
  (`Handle declarations cannot be arrays`).

Plan 1 of the typed-handle series deferred collections on purpose: they need
a lifetime and ABI design. Scalar handles are two dispatch-local lanes
(`type_id`, `slot`). A collection is a buffer of those tokens for one
dispatch, plus a count, plus a graph story for how the tokens get there.

This plan adds **parameter-only handle buffers of one exact nominal type**.
It does not add:

- handle fields on `Joint` or user structs;
- persistent handle storage across dispatches;
- mixed-kind buffers (`handle everything[]`);
- local `joint_handle values[count]` scratch (plan 4).

## Why

Without collections, every variable-arity identity set is inexpressible as
one node:

- spline chain as an explicit joint list;
- full-body IK effectors and targets;
- multi-collider / multi-pin / “influence these joints”;
- a deformer that binds to a user-chosen subset of objects.

The old stdlib used `int chain[]` and `int effectors[]` for this. Those
ints were packed component identities, not counts. After the handle cutover
that public shape is illegal: `find_joint` emits a handle, and there is no
`handle -> int`.

Graph composition (one node per effector) is not a substitute for a list
port. It changes pass interleaving and trips the two-writers-one-joint
evaluation rule. See [plan 5](solver_dsl_plan_5_multi_writer_regions.md).

Data collections already exist (`point[]`, `Weight[]`, `Joint history[]`).
Identity collections are the missing analogue.

## Depends on

- [Typed-handle plan 1–3](extensible_typed_handle_plan_1.md): exact types,
  graph handle ports, two-lane ABI.
- [Typed-handle plan 5](extensible_typed_handle_plan_5.md): typed
  `find_joint` / `find_locator` identity resolution, extended here with the
  list-valued `find_joints_by_name` source.
- [Overview](solver_dsl_plan_0_overview.md) for sequencing. Implement this
  after [plan 2](solver_dsl_plan_2_handle_topology.md) and
  [plan 3](solver_dsl_plan_3_computed_handle_effects.md) so list elements
  have the same validity and effect rules as scalars.

## Owned submodule

- `src/orlcomp/orl_parser.cpp` — lift the parameter-buffer ban for exact
  handles only.
- `src/orlcomp/orl_analysis.cpp` / `orl_analysis.h` — type, effect, and
  indexing rules.
- `src/orlcomp/orl_runtime_signature.h` — reflection for handle buffers.
- `src/orlcomp/orl_codegen.cpp` — host/CUDA lowering of `handles[i]`.
- `src/orlcomp/orl_jit.cpp`, `orl_gpu.cpp` — bind and launch.
- `src/orlcomp/orl_graph_import.cpp` — buffer handle ports and counts.
- `src/orlgraph/graph_ir.hpp`, `graph_validation.cpp`,
  `graph_serialization.cpp` — list/buffer handle cardinality.
- `src/orlexec/orl_exec.cpp`, `orl_graph_exec.cpp` — binding and
  validation of each element.
- `src/orlexec/orlrig/graph_resources.cpp` — authoring nodes that build a
  list.
- `src/orlviewer/node_graph/*` — wiring UI for a list port, if exposed
  in this period.
- tests listed below, plus a stdlib FBIK or multi-pin pilot.

## Design fixed in this period

1. Only **exact nominal** handle types may be buffer parameters
   (`joint_handle values[]`, `locator_handle values[]`). Universal `handle`
   and union buffers are rejected.
2. A handle buffer is dispatch-local. The runtime owns the backing storage
   for one launch. ORL source may index and copy elements, not store the
   buffer into solver_context or a struct field.
3. Indexing `values[i]` yields a scalar handle of the element type. The
   usual scalar rules apply: copy, equality, view conversion, no
   arithmetic, no `int(handle)`.
4. Out-of-range index and invalid element tokens fail closed before view
   access, same as a scalar invalid handle.
5. The ABI is an array of `HandleValue` (AoS: `type_id`, `slot` per
   element), plus the existing integer count parameter or shape symbol
   `<name>_count`. Do not split into parallel `int[]` slot buffers in the
   public API.
6. Each element’s `type_id` must match the buffer’s nominal type. Mixed
   kinds in one buffer are a bind error, not a runtime `is` branch.
7. Graph import maps `T values[]` to one buffer input port of
   `LogicalType::handle(...)` with `PortCardinality::Buffer`. The count is
   the existing shape symbol, not a second identity channel.
8. A list is populated only from handle-typed identity sources. The first
   concrete source in this period is `orlrig.input.find_joints_by_name`,
   which resolves a regular expression against the current joint-name
   catalog. Integer buffers cannot be connected or implicitly converted.
   A collect node, scene groups, and topology-derived lists remain future
   sources and must use this same typed bind path if added.
9. If the graph cannot prove the element identities at plan compile time,
   the node’s handle effects for that buffer are conservative (global)
   until plan 3 expands proven lists.
10. “All joints” is not the default handle list. The whole skeleton
    remains `solver_context` / FK-style arena access unless the user
    explicitly authors the full set as a subset.

## Context the implementation must respect

### Why parameter-only

Plan 3 of the handle series forbids persistent handle tokens. Putting a
handle on `Joint` or in a project file would revive slot-in-the-asset.
Parameters and one-dispatch runtime buffers stay inside that rule.

Locals of runtime size are plan 4. Shipping both at once mixes ABI work
with scratch allocation.

### Why not SoA public lanes

The host wrapper already flattens a scalar handle into two lanes. A buffer
could be `uint64 type_id[]` + `int64 slot[]`. That is convenient for
wrappers and disastrous as a user-visible type: it reintroduces a public
integer identity channel. Keep `HandleValue[]` as the language and graph
value. Flatten to lanes only inside the host/CUDA wrapper if the backend
needs it, and never expose those lanes as ORL parameters.

### Users need a way to get a collection (a subset)

`joint_handle effectors[]` answers “what does the solver receive?” It does
not answer “how does the user pick those joints?” `find_joint` only
produces one handle. The first concrete list-valued source is therefore
`find_joints_by_name`:

```text
find_joints_by_name
    name_regex: "^arm_.*$"
    joints: joint_handle[]
    count: int
```

The node enumerates the current scene's joints, applies `name_regex` to each
stable component name, and emits one exact `joint_handle` per match. The
result is a chosen **subset of scene identities** for one dispatch. It is
not an `int[]` of storage slots and it is not “all joints” unless the
regular expression explicitly selects them.

#### `find_joints_by_name` node contract

- Definition ID: `orlrig.input.find_joints_by_name`.
- Required editable parameter: `name_regex : string`.
- Outputs: `joints : joint_handle[]` with buffer cardinality and
  `count : int`. The count is a runtime-bound scalar derived from the same
  match set, so it can connect directly to an existing solver count
  parameter. Neither output exposes packed slots.
- The expression is evaluated against the complete joint name. The initial
  implementation should use one documented, backend-independent regular
  expression dialect and full-name matching; an invalid expression is a
  graph-bind error, not an empty result. A pattern such as `^arm_.*$`
  selects the names beginning with `arm_`.
- Matches are emitted in the scene catalog's deterministic joint order
  (the order used for the current packed joint snapshot). Each component
  appears at most once, and the output contains fresh dispatch-local
  handles, never serialized slots.
- No locator equivalent is committed by this plan. A future
  `find_locators_by_name` would need a separate exact
  `locator_handle[]` contract.
- A zero-match result is legal at the collection boundary. Solvers still
  decide whether `count == 0` is a no-op or an error.

This is the only concrete collection node committed in this revision.
`collect_joints`, scene groups, and topology-derived lists are intentionally
left as future alternatives while their graph cardinality and authoring
contracts are designed.

Do not:

- ask users to type slot numbers into an `int[]`;
- convert `find_joint.handle` to `int` and pack it;
- treat the whole skeleton as a handle buffer by default.

### Graph authoring shape

Language and ABI are not enough. This period commits one public authoring
shape:

- **Name-filter node:** `orlrig.input.find_joints_by_name` has one editable
  string parameter, one variable-cardinality `joint_handle[]` output, and
  one matching `int` count output. The node is the only place in this
  period where scene discovery produces a collection.

Do not also ship a generic list port or a variable-edge `collect_joints`
node yet. Graph IR already has `PortCardinality::Buffer`, but the viewer
cardinality and authoring contracts for those alternatives still need a
separate decision.

### Effects on buffer elements

`Joint(effectors[i]).rotation = ...` is a write through a computed element.
Until plan 3, treat the whole buffer port as one conservative handle
effect: “reads/writes some joint in `effectors`.” If the
`find_joints_by_name` result is resolved against the current scene snapshot,
plan 3 (or a late work item in this plan) may union the matched stable IDs
into the footprint. A pattern or scene revision that has not been resolved
keeps the node conservative.

Never interpret `effectors[i].slot` in user code. The evaluation plan may
use slots only after resolving a bound handle through the component store.

## How

### 1. Frontend

- In the parser, allow a buffer parameter whose type is a declared exact
  handle. Keep rejecting handle arrays on declarations, struct fields, and
  non-parameter locals.
- In analysis, type `values[i]` as that exact handle. Reject `values.x`,
  pointer decay, and passing a handle buffer to a scalar handle parameter.
- Index expressions stay `int`. No handle-as-index.

### 2. Reflection and ABI

- Extend `OrlRuntimeParameterKind` with `HandleBuffer` (or mark an
  existing buffer parameter as handle-element).
- Record the element canonical name on the parameter.
- Host wrapper receives a pointer + count. Validate every element with
  `IsValidHandleValue` and `type_id` match before the kernel runs.
- CUDA: device copy of the `HandleValue` array; views still go through
  `__orl_handle_context`. Do not embed host pointers in the buffer.

Bump `kHandleAbiVersion` when the wrapper shape changes.

### 3. Graph import and validation

- Import `joint_handle joints[]` as a required buffer handle port.
- Reject connections from `int[]`, `Joint[]`, or scalar handles directly
  into the buffer port unless a declared collection source sits in between.
- Serialize element canonical name and cardinality. Old graphs with
  `int chain[]` on a solver remain unsupported-definition, same as today.

### 4. Name-filter / bind path (required identity source)

- Register `orlrig.input.find_joints_by_name` with the `name_regex` string
  parameter and a `joints : joint_handle[]` output.
- At graph bind, compile the expression in the host/runtime scene-input
  resolver, enumerate the current joint catalog, and validate the pattern.
- For every match, resolve its stable `ComponentId` against the current
  packed snapshot and create a fresh `HandleValue`. Pack those values as an
  AoS buffer and bind the buffer plus its count to the generated kernel
  entry.
- Do not compile or execute a regular expression inside ORL, LLVM, or the
  CUDA kernel. On CUDA, upload the already-materialized handle buffer as an
  ordinary device buffer before launch.
- Re-resolve when the regex parameter or scene identity/topology revision
  changes. A pose-only change does not alter the matched identities, so the
  runtime may cache the materialized list until one of those revisions
  changes.
- Empty lists are legal at the collection boundary. Default FBIK/spline
  pilots may reject `count < 1` or `< 2` after binding.

### 5. Discovery versus the kernel buffer

The collection is **discovered at runtime binding and fixed for one
dispatch**. These are different meanings of “runtime”:

1. The graph stores the user-entered regular expression, not a list of
   slots or persistent handle tokens.
2. The scene-input/runtime layer evaluates that expression against the
   current scene and materializes a dispatch-local `HandleValue[]` plus a
   count. The node exposes that count as its scalar `count` output.
3. The generated ORL entry receives that buffer directly. CPU uses the
   host allocation; CUDA receives the uploaded device allocation.
4. The kernel only indexes opaque handles and performs view access. It does
   not search names, access the component catalog, or run regex matching.

Thus the buffer is not a compile-time constant and is not permanent scene
data, but it is a fixed input while a particular kernel launch runs. The
same list may be reused between launches when the regex and scene identity
revision are unchanged; a rename, add/delete, repack, or pattern edit
causes it to be rebuilt before the next affected dispatch.

This extends the existing scalar `find_joint` workflow: the graph stores a
name, and the scene-input layer resolves that name to a fresh handle during
binding rather than embedding a packed slot in the compiled kernel.

### 6. Pilot

Reintroduce `full_body_ik` (or a slimmer `solver_multi_ccd`) as a
registered graph node using two handle buffers. Keep `iterations` as
`int`. Remove that stem from `unsupported_solver_stems` only after the
pilot’s tests pass.

An explicit `joint_handle chain[]` spline overload may wait; the root+end
redesign from plan 2 is enough for the menu.

## Work items

1. Parser/analysis exceptions for exact-handle buffer parameters.
2. Runtime reflection, host wrapper, CUDA copy, cache version bump.
3. Graph logical type + import + serialization.
4. `find_joints_by_name` runtime node and regex/bind validation.
5. Evaluation: conservative buffer-port effect; optional proven-list
   expansion if plan 3 is already in tree.
6. FBIK (or multi-CCD) stdlib + registration + tests.
7. Viewer wiring for the regex property and collection output.
8. Cache invalidation when a regex, joint name, or topology revision changes.

## Tests

Extend:

- `tests/parser_tests/parser_tests.cpp`
- `tests/analysis_tests/analysis_tests.cpp`
- `tests/syntax_tests/language_syntax_tests.cpp`
- `tests/codegen_tests/codegen_tests.cpp`
- `tests/exec_tests/exec_tests.cpp`
- `tests/graph_lowering_tests/graph_lowering_tests.cpp`
- `tests/rig_graph_tests/rig_graph_tests.cpp`
- `tests/orlrig_tests/orlrig_tests.cpp`
- `tests/gpu_tests/gpu_tests.cpp` (or documented skip)

Required positive cases:

- `joint_handle values[]` parses and types `values[i]` as `joint_handle`;
- kernel reads/writes `Joint(values[i])` on CPU;
- `find_joints_by_name` with `^arm_.*$` binds exactly the matching joints in
  deterministic order;
- invalid regular expressions fail during graph binding;
- the matched `HandleValue[]` and count are passed to the CPU/CUDA kernel,
  with no regex operation in generated ORL or device code;
- FBIK/multi-CCD pilot returns the same numeric result as the old index
  implementation on a fixture with two effectors.

Required negative cases:

- `handle values[]` and `source_xform_handle values[]` rejected;
- `int[]` connected to a handle buffer port rejected;
- wrong `type_id` in any element rejected at bind, not during a view;
- `locator_handle[]` connected to `joint_handle[]` rejected;
- struct field `joint_handle id;` still rejected;
- implicit `int <-> handle` still rejected for buffer elements.

## Exit criteria

- Exact-handle buffer parameters compile, bind, and run on CPU.
- CUDA either matches or is an explicit capability skip, not a silent
  host fallback.
- No public integer identity channel exists for the list.
- At least one registered stdlib node uses a handle buffer.
- Users can author a joint subset through `find_joints_by_name`; C++ bind is
  not the only path.
- Old `int chain[]` / `int effectors[]` graph definitions stay absent.

## Not in scope

- Union or universal handle buffers.
- Persistent / serialized handle tokens.
- Local dynamic handle arrays (plan 4).
- Pass-major fusion of N scalar nodes (plan 5).
- Variable-edge collection nodes, scene group assets, and computed subsets
  (`joint_children`, `joint_descendants`, locator name filters). These may
  write the same handle-buffer types later, but are not required for this
  first concrete source.
