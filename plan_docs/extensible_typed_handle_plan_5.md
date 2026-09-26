# Extensible typed handles plan 5: scene inputs and one-socket nodes

## Goal

Introduce the authored scene-input contract:

```text
find_joint.handle      : joint_handle
find_locator.handle    : locator_handle
```

These nodes expose no public `index` or `xform`.
`orlrig.input.find_controller` is removed. Stable joint/locator identity is
resolved to a dispatch-local typed handle during graph bind/lowering.

## Depends on

- [Plan 2](extensible_typed_handle_plan_2.md): graph handle ports.
- [Plan 3](extensible_typed_handle_plan_3.md): handle binding.
- [Plan 4](extensible_typed_handle_plan_4.md): rig type registry/resolvers.

## Owned submodule

- `src/orlexec/orlrig/graph_resources.hpp`
- `src/orlexec/orlrig/graph_resources.cpp`
- `src/orlviewer/graph_scene_inputs.hpp`
- `src/orlviewer/graph_scene_inputs.cpp`
- `src/orlviewer/graph_scene_runtime.cpp`
- `src/orlviewer/component_manager.hpp`
- `src/orlviewer/qt/node_graph_editor.cpp`
- `src/orlviewer/qt/property_editor.cpp`
- project/graph migration code in `src/orlviewer`
- constant-output and graph-lowering APIs used by scene input nodes

## Compatibility boundary

This period is an intentional breaking change. There are no hidden
compatibility definitions for the old scene-input nodes.

- `orlrig.input.find_{joint,locator}` are the only joint/locator lookup
  definitions and expose one typed `handle` socket.
- Old `index` and `xform` sockets are removed rather than adapted.
- `orlrig.input.find_controller` is removed. Controller attachment and
  viewport transform state remain authoring utilities, not graph inputs.
- Old serialized graphs that reference removed definitions or ports fail with
  a precise unknown-definition/unknown-port diagnostic and must be rebuilt.
- No old integer edge is reinterpreted as a handle.

## Design fixed in this period

1. Every new scene input node has one output named `handle`.
2. The node definition supplies the exact nominal type.
3. The editable graph stores the selected stable `ComponentId`/name, never a
   packed slot.
4. Binding resolves stable identity against the current snapshot and creates
   `{type_id, slot}`.
5. Topology and packing remain fixed for the compiled rig's lifetime. A
   repack invalidates that rig; its replacement resolves the same serialized
   identity to a fresh slot.
6. Missing/deleted/ambiguous scene elements fail graph binding. Index `0` is
   never substituted.
7. Mesh handling stays unchanged unless a dedicated `mesh_handle` consumer is
   part of this period; do not broaden scope accidentally.
8. Controllers have no graph input node, nominal handle type, runtime token,
   or packed handle arena.

## Work items

### 1. Node definitions

- Change `make_find_definition` so joint/locator definitions create exactly
  one scalar handle output with the registered nominal type.
- Remove `orlrig.input.find_controller` from node registration and authoring
  menus.
- Remove xform output adapters and array-index semantics from the new
  definitions.
- Preserve the `name` parameter and stable node IDs.

### 2. Stable identity resolution

- Extend scene input catalog/runtime binding to resolve selected component
  identity to the current typed slot.
- Validate descriptor tag, component existence, kind, snapshot revision, and
  slot bounds.
- Invalidate and rebuild the compiled rig on topology/packing revision; never
  reuse or persist one of its dispatch-local tokens.

### 3. Lowering integration

- Replace stringified scalar constant handling with a typed handle value path.
- Ensure graph specialization sees the exact output type even though component
  identity is not known until bind time.
- Do not materialize xform buffers merely because a handle is connected.

### 4. Project loading

- Reject old `handle/index/xform` connections with a precise diagnostic.
- Reject old `find_controller` nodes and direct users toward controller
  attachment/viewport configuration.
- Do not synthesize index buffers, matrix adapters, or controller graph
  inputs while loading a project.

### 5. Editor/reflection update

- Display one `handle` socket with type-specific color/style or label.
- Confirm port cardinality treats handles as scalar without treating them as
  editable numeric constants.

## Tests

Extend:

- `tests/rig_graph_tests/rig_graph_tests.cpp`
- `tests/scene_graph_tests/scene_graph_tests.cpp`
- `tests/graph_lowering_tests/graph_lowering_tests.cpp`
- graph/project serialization tests

Required cases:

- each new find node has exactly one output named `handle`;
- joint/locator outputs have distinct exact types;
- no `find_controller` definition or `controller_handle` is registered;
- selected stable identity binds to the correct current slot;
- repacking invalidates the old compiled rig, and the rebuilt rig preserves
  serialized identity and selected component even if the new slot differs;
- missing/deleted/wrong-kind components fail before execution;
- no xform buffer is packed for an unused typed handle;
- old project fixtures produce a precise breaking-change diagnostic;
- no legacy joint/locator definitions are registered;
- controller attachment remains available through the viewport workflow only.

Suggested verification:

```sh
cmake --build build --target \
  orl_rig_graph_tests orl_graph_lowering_tests orl_scene_graph_tests
./build/tests/rig_graph_tests/orl_rig_graph_tests
./build/tests/graph_lowering_tests/orl_graph_lowering_tests
./build/tests/scene_graph_tests/orl_scene_graph_tests
```

## Exit criteria

- Newly authored joint/locator inputs expose one typed socket.
- Controllers remain available to the viewport attachment workflow but not to
  graph authoring.
- Stable identity survives save/load and recompilation after storage
  repacking.
- Existing project fixtures are intentionally invalidated and report the
  required graph rebuild.
- No solver metadata or stdlib index signature is removed yet.

## Not in scope

- Two-bone IK conversion.
- Evaluation-plan effect derivation.
- Named unions and `is`.
- Final deletion of legacy definitions and metadata.
