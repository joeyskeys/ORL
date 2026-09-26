# Extensible typed handles plan 8: full rig migration and legacy removal

## Goal

Migrate the remaining rig algorithms and project fixtures, then remove the
legacy public index/xform input path and kind-specific port metadata.

This is the cleanup/cutover period. It begins only after exact handles,
one-socket inputs, typed effects, the IK pilot, and static specialization are
proven independently.

## Depends on

- [Plan 5](extensible_typed_handle_plan_5.md): one-socket nodes and migration
  compatibility.
- [Plan 6](extensible_typed_handle_plan_6.md): effect-derived IK pilot.
- [Plan 7](extensible_typed_handle_plan_7.md): unions and static `is`.

## Owned submodule

- remaining `resource/stdlib/constraint/*.orl`
- remaining `resource/stdlib/solver/*.orl`
- affected `resource/stdlib/deformer`, locator, and helper modules
- `src/orlexec/orlrig/graph_resources.cpp`
- `src/orlexec/orlrig/evaluation.cpp`
- `src/orlgraph/graph_ir.hpp`
- `src/orlgraph/graph_serialization.cpp`
- `src/orlviewer/graph_scene_inputs.cpp`
- `src/orlviewer/graph_scene_runtime.cpp`
- `graph_json_format.md`
- project migration/version code and all affected fixtures/tests

## Migration rule

Audit each integer parameter by meaning. Convert only values that identify a
scene component. Keep true counts, loop indices, vertex indices, weight
indices, hierarchy table offsets, and other numeric data as `int`.

For each component parameter, record:

- accepted exact/union handle type;
- destination typed view and field read/write lowering used;
- read/write effect;
- identity/propagation behavior;
- CPU/GPU support;
- legacy graph migration rule.

Do not delete compatibility code until this matrix has no unresolved shipped
node or project entry.

## Work items

### 1. Inventory and migration matrix

Audit at least:

- copy translation/rotation/scale/xform constraints;
- parent, aim, and aim-locator constraints;
- FK, spline IK, HD-IK, and full-body IK;
- locator helpers;
- old controller graph ports/nodes, which must be removed rather than
  converted to handles;
- LBS and other deformers where a scene component identity is passed;
- viewer-created graph templates and serialized test projects.

Classify nodes as:

- exact handle only;
- named union plus compile-time `is`;
- ordinary numeric/buffer data, unchanged;
- intentionally global/unsupported pending a separate design.

### 2. Convert algorithms in small batches

Use one logical commit/test batch per family:

1. copy and parent constraints;
2. aim constraints;
3. remaining exact-type solvers;
4. generic/union constraints with `is`;
5. deformer/helper paths;
6. controller graph removal and attachment-workflow migration diagnostics.

For each batch:

- replace component indices with handles;
- replace direct packed storage access with registered storage-backed views
  and ordinary field reads/assignments;
- derive effects transitively;
- remove that node's kind-specific port metadata;
- run family tests plus graph/runtime regressions before continuing.

### 3. Project and graph migration

- Keep repository fixtures on the canonical typed find nodes introduced in
  plan 5.
- Reject old projects that contain removed definitions or `index`/`xform`
  ports with node/port diagnostics.
- Keep controller behavior in attachment/viewport configuration; there is no
  controller graph-node migration.
- The remaining HD-IK, spline-IK, and full-body-IK sources still depend on
  dynamic packed-index traversal. They are removed from graph registration in
  this cutover rather than exposed through an unsafe integer fallback; a
  future algorithm-specific period may reintroduce typed replacements.

### 4. Remove legacy infrastructure

The scene-input legacy infrastructure is removed in plan 5:

- no hidden legacy `find_*` definitions are registered;
- no public `index` and `xform` output construction/adapters remain;
- `orlrig.input.find_controller`, controller handle semantics, and controller
  graph-node registration are absent;
- version the solver/runtime context to remove packed controller snapshots and
  controller count/offset lanes after all controller graph consumers are gone;
- remove `scene.array_index` inference for component handles;
- remove parsing/import/evaluation of
  `partial_*_{joint,controller,locator}_ports`;
- remove obsolete kind-specific fields from `PartialEvaluationFootprint` and
  their serialized form after no compatibility reader needs them;
- remove obsolete xform packing/bindings used only by legacy find outputs;
- delete integer-to-component fallback paths and stale tests;
- bump language, graph schema, handle ABI, and cache versions as required.

### 5. Final safety audit

- Search authored ORL for component parameters still named/used as indices.
- Search graph registration/evaluation for kind-specific metadata authority.
- Confirm no implicit `int <-> handle` conversion or fallback remains.
- Confirm no generated generic kernel branches on handle `type_id`.
- Confirm serialized graphs contain stable identity, never dispatch slots.

## Tests

Run the full focused matrix after every family, then the complete suite at the
end:

```sh
cmake --build build --target \
  orl_analysis_tests orl_graph_ir_tests orl_graph_lowering_tests \
  orl_rig_graph_tests orl_codegen_tests orl_exec_tests \
  orlrig_tests orl_scene_graph_tests orl_syntax_tests orl_gpu_tests
ctest --test-dir build --output-on-failure
```

Required final cases:

- canonical joint/locator input nodes each have one handle output;
- no controller input node or controller handle type is registered;
- controller setup/input transforms and attachment behavior continue working
  through viewport authoring tests;
- all migrated algorithms reject wrong component kinds at graph compile time;
- exact and specialized-union CPU results match baseline fixtures;
- conditional CUDA parity for supported algorithms;
- evaluation-plan reads/writes no longer depend on kind-specific port strings;
- old project fixtures migrate or produce intentional precise diagnostics;
- project migration is deterministic and idempotent;
- scene repacking does not change selected identity;
- no runtime kind branch exists in generic specialized kernels;
- no legacy authoring definition or implicit integer fallback remains.

## Exit criteria

- The migration matrix is complete and reviewed.
- All shipped rig nodes use typed handles where they identify components.
- Legacy `index/xform` scene-input sockets and kind-specific metadata are gone.
- Full CPU test suite passes; GPU tests pass or report only documented
  environment skips.
- Language/graph/project/ABI versions clearly reject unsupported old artifacts
  instead of misinterpreting them.

## Not in scope

- Runtime-varying nominal handle kinds.
- Any persistent handle storage; handles remain dispatch-local.
- Replacing genuine numeric indices unrelated to component identity.
- New rig algorithms unrelated to the migration.
