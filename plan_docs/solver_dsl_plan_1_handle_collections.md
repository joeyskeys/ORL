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

A handle buffer is only the kernel type. This period also provides at least
one **list-valued identity source** so users can obtain a collection: a
chosen **subset of scene joints or locators**, carried as handles.

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
- [Typed-handle plan 5](extensible_typed_handle_plan_5.md): `find_joint` /
  `find_locator` as the only public identity source.
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
   required source is a user-selected subset (collect of `find_*`, or a
   thin scene-group front end over the same path). Integer buffers cannot
   be connected or implicitly converted.
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
produces one handle. Without a list-valued source, only C++ tests can fill
the buffer. Shipping the ABI without an identity source is not a complete
feature.

The scene already has the full joint and locator sets. A collection is a
chosen **subset of those identities**, carried as handles for one
dispatch. It is not an `int[]` of storage slots, and not “all joints”
unless the user explicitly asks for that set.

Ways a subset can be obtained (same buffer type, different sources):

| Source | What the user selects | This period? |
| --- | --- | --- |
| Collect of scalar `find_*` | Enumerated subset, one handle per incoming edge | **Required** |
| Scene group (`LeftArm`) | Named asset subset, one socket | Optional thin front end over collect |
| Topology (`children` / `descendants`) | Computed subset of a root handle | Later; needs plan 2 child/descendant APIs |
| Name filter (`arm_*`) | Computed subset by string | Later, optional |

The collect path *is* a user-facing subset: the subset is the set of
incoming find nodes.

```text
find_joint("a") ─┐
find_joint("b") ─┼→ collect_joints → joint_handle[]
find_joint("c") ─┘
```

A scene group (`find_joint_set("LeftArm")`) is the same idea with one
socket. Topology subsets are computed subsets that write into the same
`joint_handle[]` type; they are a second feature, not a requirement to
ship the buffer ABI.

Do not:

- ask users to type slot numbers into an `int[]`;
- convert `find_joint.handle` to `int` and pack it;
- treat the whole skeleton as a handle buffer by default.

### Graph authoring shape

Language and ABI are not enough. Choose one public authoring shape in this
period and do not ship both:

- **Collect node (required first):** a runtime node
  `orlrig.input.collect_joints` with a variable incoming handle list and
  one `joint_handle[]` output. Same for locators. The collect node is the
  only place arity is variable.
- **List port:** one solver input accepts N incoming handle edges. Graph
  IR already has `PortCardinality::Buffer`, but validation and the viewer
  menu today assume one edge per input.

If a scene group node appears in this period, it lowers to the same
collect/bind path, not a second ABI.

### Effects on buffer elements

`Joint(effectors[i]).rotation = ...` is a write through a computed element.
Until plan 3, treat the whole buffer port as one conservative handle
effect: “reads/writes some joint in `effectors`.” If every element is a
collect of `find_joint` nodes, plan 3 (or a late work item in this plan)
may union those stable IDs into the footprint.

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

- Import `joint_handle name[]` as a required buffer handle port.
- Reject connections from `int[]`, `Joint[]`, or scalar handles directly
  into the buffer port unless a collect node sits in between.
- Serialize element canonical name and cardinality. Old graphs with
  `int chain[]` on a solver remain unsupported-definition, same as today.

### 4. Collect / bind path (required identity source)

- Implement one runtime collect definition per exact type used in stdlib
  (`joint`, `locator`). This is the user-facing way to obtain a subset.
- Binding: resolve each incoming `find_*` to a `HandleValue`, pack the
  AoS buffer, bind count.
- Empty lists are legal only if the solver documents `count == 0` as a
  no-op. Default FBIK/spline pilots reject `count < 1` or `< 2`.
- Optional: `find_joint_set("LeftArm")` as a one-socket front end that
  expands a named scene group into the same collect/bind path.

### 5. Pilot

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
4. Collect runtime nodes (required subset source) and bind validation.
5. Evaluation: conservative buffer-port effect; optional proven-list
   expansion if plan 3 is already in tree.
6. FBIK (or multi-CCD) stdlib + registration + tests.
7. Viewer wiring for collect: add/remove incoming `find_*` edges.
8. Optional scene-group node if the asset already has named joint sets.

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
- collect of two `find_joint` nodes binds two valid tokens (enumerated
  subset);
- FBIK/multi-CCD pilot returns the same numeric result as the old index
  implementation on a fixture with two effectors.

Required negative cases:

- `handle values[]` and `source_xform_handle values[]` rejected;
- `int[]` connected to a handle buffer port rejected;
- wrong `type_id` in any element rejected at bind, not during a view;
- `locator_handle` collect into `joint_handle[]` rejected;
- struct field `joint_handle id;` still rejected;
- implicit `int <-> handle` still rejected for buffer elements.

## Exit criteria

- Exact-handle buffer parameters compile, bind, and run on CPU.
- CUDA either matches or is an explicit capability skip, not a silent
  host fallback.
- No public integer identity channel exists for the list.
- At least one registered stdlib node uses a handle buffer.
- Users can author a subset through collect (or a group that lowers to
  collect); C++ bind is not the only path.
- Old `int chain[]` / `int effectors[]` graph definitions stay absent.

## Not in scope

- Union or universal handle buffers.
- Persistent / serialized handle tokens.
- Local dynamic handle arrays (plan 4).
- Pass-major fusion of N scalar nodes (plan 5).
- Scene group assets, unless they are a thin front end over collect.
- Computed subsets (`joint_children`, `joint_descendants`, name
  filters). Those write the same `joint_handle[]` type later; they are
  not required to ship the buffer ABI or the collect subset.
