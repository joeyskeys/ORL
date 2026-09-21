# ORL project file format

This document describes the JSON project file written by
`save_project_json` / read by `load_project_json`
(`src/orlviewer/project_serialization.cpp`). Use it when generating a
project by hand or by script.

The file is UTF-8 JSON. The viewer writes it with 2-space indents. Compact
JSON also loads. Comments are not allowed.

Current constants:

- Project magic: `ORL_PROJECT`
- Project `format_version`: `1`
- Nested graph-stages magic: `ORL_GRAPH_STAGES`
- Graph-stages `format_version`: `1`
- `language_version`: `orl-0`
- `logical_abi_version`: `orlgraph-0`

Node definitions are **not** stored in the project. Graph nodes refer to
registry IDs such as `orlrig.solver.ik_two_bone`. The viewer registers those
at startup.

Meshes and other `vkkk::Scene` objects are **not** stored. A skeleton-only
project is valid.

---

## 1. Two different integer namespaces

This is the usual source of bad generated files.

| Integer | Meaning | Where it appears | Example |
|---|---|---|---|
| **Component ID** | File-local stable handle for one component | `joints[].id`, `joints[].value.parent`, `locators[].id`, `controllers[].id`, `constraints[].id`, constraint `root` / `mid` / `end` / `target` / `pole`, attachment `controller` / `target` | `"id": 7` |
| **Packed index** | Position of a **live** joint (or locator) in the packed array | weight `values[].joint`; runtime `Joint.parent` after load | `"joint": 0` |

They are not interchangeable. The file stores parent as a component ID. The loader rewrites it to a packed index for the runtime joint buffer.

### Component ID

- Unsigned integer, **must be `>= 1`**. `0` is invalid.
- Unique across joints, locators, controllers, and constraints in the file.
- Names are also unique across those kinds.
- On load, every file ID is **remapped** to a new runtime `ComponentId`.
  Runtime IDs are a process-wide monotonic counter shared with weights,
  deformers, and anything else created before load. Do not assume a file ID
  survives into the running viewer.
- Constraint and attachment fields that name other components use **file
  component IDs**, and those IDs are remapped with the same table.

### Packed index

- Signed integer used at **runtime** after load (`Joint.parent`) and in
  weight `values[].joint`.
- `-1` means “no parent”.
- For joints, it is an index into the live packed joint array, which is
  the `components.joints` array order after load.
- Do **not** write a packed index into `joints[].value.parent` in the
  file. Write the parent joint's component ID instead.

Correct pair:

```json
"joints": [
  { "id": 10, "name": "root",  "value": { "parent": -1, "...": "..." } },
  { "id": 20, "name": "spine", "value": { "parent": 10, "...": "..." } }
]
```

`spine.parent == 10` because `root` has `"id": 10`. Load maps that ID to
packed slot 0.

Wrong:

```json
{ "id": 20, "name": "spine", "value": { "parent": 0 } }
```

`0` is not a valid component ID. The second joint would appear to parent
to a missing id rather than to the joint with `"id": 1`.

---

## 2. Top-level document

```json
{
  "header": {
    "magic": "ORL_PROJECT",
    "format_version": 1
  },
  "evaluation": {
    "evaluate_orl": true
  },
  "graph": { },
  "components": { },
  "editor": { }
}
```

Required:

- `header.magic`
- `header.format_version` (unsigned integer `1`)
- `graph` (object; staged-graph document, see §4)
- `components` (object; see §3)

Optional:

- `evaluation.evaluate_orl` (bool). Default if omitted: `true`.
  When true, the viewer runs solver evaluation after load.
- `editor` (object). Viewer-only UI state. Not part of GraphModule /
  orlgraph. Omitted files load with the default node-graph auto-layout.
  See §2.1.

`header.format_version` and component IDs must parse as JSON integers, not
floats.

### 2.1 `editor` (optional)

Persists the node-graph canvas so switching solver/deformer and reopening
the file keep each stage's node positions, pan, and zoom.

```json
"editor": {
  "node_graph": {
    "solver": {
      "pan": [48.0, 48.0],
      "zoom": 1.0,
      "nodes": [
        { "id": "ik_two_bone_1", "x": 64.0, "y": 80.0 },
        { "id": "__graph_input__:joints", "x": -280.0, "y": 80.0 }
      ]
    },
    "deformer": {
      "pan": [48.0, 48.0],
      "zoom": 1.0,
      "nodes": []
    }
  }
}
```

- `editor.node_graph.solver` / `editor.node_graph.deformer` are independent.
  Missing stages keep the default pan `[48, 48]`, zoom `1`, and auto-placed
  nodes.
- `pan`: length-2 array `[x, y]` in canvas pixels.
- `zoom`: number, typically `0.35`–`2.5`.
- `nodes[]`: `{ "id", "x", "y" }`. `id` is the graph node id, or
  `__graph_input__:<port>` / `__graph_output__:<port>` for interface
  nodes. Unknown ids are ignored. Nodes missing from this list are
  auto-placed.
- `frames[]` (optional): editor-only groups around nodes. Missing means
  no frames. Each entry is:

```json
{
  "id": "frame",
  "title": "Frame",
  "x": 10.0,
  "y": 20.0,
  "width": 400.0,
  "height": 280.0,
  "collapsed": false,
  "members": ["ik_two_bone_1", "__graph_input__:joints"]
}
```

  `x`/`y`/`width`/`height` are the expanded canvas rect. `collapsed`
  hides member nodes and exposes one pseudo input and one pseudo output
  on the frame. Member ids that do not exist after load are dropped.

---

## 3. `components`

Required arrays (may be empty):

- `joints`
- `locators`
- `controllers`
- `constraints`
- `attachments`

Optional objects:

- `weights`
- `deformer`

The viewer already owns a persistent weight component (default name
`weights`) and deformer (default name `deformer`). Load will not destroy
those. Generated names must not collide with them.

Load order:

1. Validate graphs against the registry.
2. Validate component IDs, names, parent IDs, constraint refs,
   attachment refs.
3. Destroy existing joints, locators, controllers, constraints, and curves.
4. Recreate joints **in array order**, then locators, then controllers.
5. Restore attachments, then overwrite controller xforms, then constraints.
6. Optionally overwrite the persistent weight / deformer buffers.
7. Install solver + deformer graphs.

Packed joint order after load **is the `joints` array order**. File
`parent` values are component IDs of joints in that array; load converts
them to packed indices.

### 3.1 Joint

```json
{
  "id": 3,
  "name": "root",
  "value": {
    "parent": -1,
    "selected": 0,
    "translation": [0.0, 1.0, 0.0],
    "rotation": [0.0, 0.0, 0.0, 1.0],
    "scale": [1.0, 1.0, 1.0]
  }
}
```

| Field | Type | Notes |
|---|---|---|
| `id` | uint64 `>= 1` | File component ID. Remapped on load. |
| `name` | non-empty string | Unique. Graph `find_joint` matches this name. |
| `value.parent` | int64 | Parent joint **component ID**, or `-1`. |
| `value.selected` | int64 | Optional; default 0. Viewer selection flag. |
| `value.translation` | 3 floats | **Parent-local** translation. Root is world. |
| `value.rotation` | 4 floats | Quaternion `x, y, z, w`. Identity is `[0,0,0,1]`. |
| `value.scale` | 3 floats | Local scale. Identity is `[1,1,1]`. |

World transform is the product of ancestor local matrices
(`joint_world_matrix`). Y-up, right-handed; viewer world is +X right, +Y up,
+Z toward the camera.

The 128-byte runtime `Joint` also has padding; the file only stores the
fields above.

### 3.2 Locator

```json
{
  "id": 40,
  "name": "ik_target_arm_l_locator",
  "xform": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0.72,0.98,0,1]
}
```

`xform` is a 4×4 matrix, 16 numbers, **column-major GLM layout**:
index `column * 4 + row`. Translation is `[12], [13], [14]` (`x, y, z`).
Identity:

```
[1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1]
```

Locators are world-space. They are not parented. Packed locator order is
the `locators` array order.

`find_locator` matches `name`.

### 3.3 Controller

```json
{
  "id": 50,
  "name": "ctrl_root",
  "value": {
    "xform": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,1,0,1],
    "input_xform": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
  },
  "shape": 1
}
```

| Field | Meaning |
|---|---|
| `value.xform` | Setup / bind transform. World placement of the handle at author time. |
| `value.input_xform` | Animation delta relative to setup. Identity at bind. |
| World used at runtime | `xform * input_xform` |

`shape` is `orlviewer::ControllerShape` as int64:

| Value | Name | Typical use |
|---:|---|---|
| 0 | Circle (alias Curve) | Default FK / IK target |
| 1 | Square | Root / COG |
| 2 | Triangle | |
| 3 | Diamond | |
| 4 | Star | |
| 5 | Cross | |
| 6 | Arrow | |
| 7 | Polygon | IK pole (viewer default) |

`find_controller` matches `name`.

Controllers are not a packed solver buffer in new graphs. They are
authoring handles. Driving a joint or locator is done with an attachment,
or with solver-graph nodes such as `orlrig.constraint.copy_xform`.

### 3.4 Attachment

```json
{
  "controller": 50,
  "target": 3,
  "target_kind": 1,
  "xform": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
  "input_drives_target": true
}
```

| Field | Type | Notes |
|---|---|---|
| `controller` | component ID | Must be a controller. |
| `target` | component ID | Joint or locator. |
| `target_kind` | int64 | `0` None (invalid here), `1` Joint, `2` Locator. Must match `target`. |
| `xform` | mat4 | Target-local offset: `inverse(target_world) * controller_world` at bind. Identity if the handle sits on the target. |
| `input_drives_target` | bool | If true, each eval writes `controller_world * inverse(xform)` onto the target. |

Rules from the viewer:

- One controller per target.
- IK targets/poles should attach to **locators**, not to the IK chain
  joints (that would cycle with two-bone IK).
- FK handles attach to **joints**.
- If the solver-stage graph has any nodes/inputs/outputs, that graph is
  authoritative and `apply_controller_inputs` is skipped. Empty graphs keep
  the attachment + constraint fallback path.

### 3.5 Constraint (two-bone IK component)

```json
{
  "id": 90,
  "name": "ik_arm_l",
  "type": "ik_two_bone",
  "root": 8,
  "mid": 9,
  "end": 10,
  "target": 40,
  "pole": 41,
  "bound": true
}
```

`root`, `mid`, `end`, `target`, and `pole` are **component IDs**, not packed
indices. `root` / `mid` / `end` must exist (normally joints). `target` /
`pole` must exist (normally locators; the runtime can also read a
controller world xform).

`bound: true` is required for the fallback solver to evaluate the chain.

`root` / `mid` / `end` must form a two-bone chain in the file:
the end joint's `parent` ID is `mid`, and the mid joint's `parent` ID is
`root`. The same component IDs go in the constraint.

This component path runs only when the solver graph is empty. A populated
solver graph must encode the same IK with `orlrig.solver.ik_two_bone` and
`find_*` nodes; otherwise attachments/constraints are ignored.

### 3.6 Weights (optional)

Omitted in a skeleton-only file.

```json
"weights": {
  "name": "weights",
  "weight_count": 5,
  "values": [
    { "weight": 1.0, "joint": 0 }
  ]
}
```

`values[].joint` is a **packed joint index**, not a component ID.
Layout is vertex-major:
`values[vertex * weight_count + slot]`. Default `weight_count` is 5.
These integers are **not** remapped on load.

### 3.7 Deformer (optional)

Omitted in a skeleton-only file.

```json
"deformer": {
  "type": "lbs",
  "mesh_name": "",
  "bind_model": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
  "bound": false,
  "bind_positions": [],
  "inverse_binds": []
}
```

`bind_positions` entries are 4 doubles (`point`). `inverse_binds` entries
are 16 doubles (matrices). `mesh_name` refers to a scene mesh that is not
part of this file.

---

## 4. `graph` (staged graphs)

This object is an `ORL_GRAPH_STAGES` document embedded as `project.graph`.

```json
"graph": {
  "header": {
    "magic": "ORL_GRAPH_STAGES",
    "format_version": 1,
    "language_version": "orl-0",
    "logical_abi_version": "orlgraph-0",
    "module_id": "human.solver",
    "content_hash": ""
  },
  "stages": {
    "solver": { },
    "deformer": { }
  }
}
```

Both `stages.solver` and `stages.deformer` are required `GraphModule`
objects.

`content_hash` is an FNV-1a hex digest of the **canonical compact** staged
JSON with an empty hash field. If `content_hash` is `""` or omitted, the
loader skips the hash check. If it is non-empty, it must match the
canonical serializer. Hand-authored files should leave it empty unless they
go through `serialize_graph_stages_json`.

Load always `validate()`s both graphs against the registry. Empty modules
are valid.

### 4.1 Graph module

```json
{
  "module_id": "human.solver",
  "version": { "major": 1, "minor": 0, "patch": 0 },
  "language_version": "orl-0",
  "logical_abi_version": "orlgraph-0",
  "inputs": [],
  "outputs": [],
  "resources": [],
  "nodes": [],
  "connections": []
}
```

All six arrays are required.

Graph-level `inputs` / `outputs` / `resources` / `nodes` use **string
stable IDs**, not component IDs. Scene elements are selected by **component
name** on `find_*` nodes (`parameters.name`), never by numeric file IDs.

### 4.2 Node instance

```json
{
  "id": "find_root",
  "definition": "orlrig.input.find_joint",
  "name": "find_root",
  "parameters": {
    "name": {
      "type": {
        "kind": 4,
        "name": "string",
        "lanes": 0,
        "extent": 0,
        "element": null
      },
      "value": { "kind": "string", "value": "root" }
    }
  },
  "mappings": [],
  "provenance": {
    "locations": [],
    "source_nodes": [],
    "description": ""
  },
  "inline_policy": 0
}
```

`inline_policy`: `0` Default, `1` Never, `2` Always.

Useful registry IDs (not stored; referenced by `definition`):

Solver stage:

- `orlrig.input.find_joint` — outputs `handle`, `index`, `xform`; param `name`
- `orlrig.input.find_controller` — same ports; param `name`
- `orlrig.input.find_locator` — same ports; param `name`
- `orlrig.input.find_mesh` — output `handle`; param `name`
- `orlrig.solver.ik_two_bone` — inputs `root`, `mid`, `end`, `target_index`, `pole_index` (all packed indices / array-index semantics); output `status`
- `orlrig.solver.fk`
- `orlrig.constraint.copy_xform` (and copy_translation / rotation / scale, aim, aim_locator)

Deformer stage:

- `orlrig.stage.computed_joints`
- `orlrig.deformer.lbs.capture_bind` / `orlrig.deformer.lbs.evaluate`
- `orlrig.auto_weight.*`

`find_*` `index` is a packed index at eval time, not a file component ID.

### 4.3 Connection endpoint

```json
{
  "source": { "kind": 0, "owner": "find_root", "port": "index" },
  "destination": { "kind": 0, "owner": "solver", "port": "root" },
  "conversion": "",
  "shape": [],
  "provenance": { "locations": [], "source_nodes": [], "description": "" },
  "feedback": false
}
```

`kind`: `0` NodePort, `1` GraphInput, `2` GraphOutput.

- NodePort: `owner` = node instance id, `port` = port id.
- GraphInput / GraphOutput: `owner` = interface port id; `port` is unused
  but still required (empty string is fine).

`shape: []` is a scalar.

### 4.4 Logical type / domain / shape (when you add graph ports)

`type.kind` (`LogicalTypeKind`):

| 0 Void | 1 Bool | 2 Int64 | 3 Float64 | 4 String |
| 5 Vector | 6 Point | 7 Normal | 8 Vec4 | 9 Quaternion |
| 10 Matrix | 11 Struct | 12 Array | 13 Buffer | 14 Unknown |

String example: `"kind": 4, "name": "string", "lanes": 0, "extent": 0, "element": null`.

`domain.kind` (`DomainKind`): `0` Constant, `1` Rig, `2` Joint, `3` Vertex,
`4` Edge, `5` Face, `6` Instance, `7` Buffer, `8` Custom.

Scalar shape: `[]`. Symbolic length-one: `[{"kind": 1, "constant": 0, "symbol": "joint_count"}]`.

Constant values wrap a type plus a tagged payload:

```json
{ "kind": "string", "value": "root" }
{ "kind": "int64", "value": 0 }
{ "kind": "float64", "value": 1.0 }
{ "kind": "bool", "value": true }
{ "kind": "null" }
```

`direction`: `0` Input, `1` Output, `2` InOut.

---

## 5. Runtime behavior that affects generated files

1. **Empty solver graph**  
   Viewer applies controller attachments, then evaluates bound
   `ik_two_bone` constraints. This is the compatibility path and is enough
   for a skeleton + FK handles + IK handles.

2. **Non-empty solver graph**  
   Only the graph runs. Put `find_*` + `ik_two_bone` / `copy_xform` in the
   graph or FK/IK from components will not run.

3. **Graph scene lookup is by name**  
   `find_joint` parameter `name` must equal `joints[].name`.

4. **IDs in a viewer-saved file often do not start at 1**  
   The app creates `weights` and `deformer` first (`id` 1 and 2). The first
   joint is often `3`. Generated files may still start at 1; load remaps.
   Do not reuse the names `weights` or `deformer`.

5. **Packed buffers are rebuilt from live objects**  
   Deleting a joint frees it and removes it from `joint_order`. File IDs
   are never reused in a running session. Save writes `parent` as the
   parent joint's runtime component ID; load maps that ID back to a packed
   index.

---

## 6. Minimal skeleton recipe

1. List joints in the packed order you want after load.
2. Assign each a unique `id >= 1` and a unique `name`.
3. Set `value.parent` to the parent joint's **component ID**, or `-1`.
4. Store local translation/rotation/scale, not world, except for the root.
5. Give locators their own IDs. Put IK targets at the end-joint **world**
   position; poles via the two-bone pole vector of root/mid/end world
   positions.
6. Give controllers their own IDs. Identity `input_xform`. `xform` at the
   same world as the handle.
7. Attach FK controllers to joints (`target_kind: 1`). Attach IK
   controllers to locators (`target_kind: 2`).
8. Constraints: `root`/`mid`/`end`/`target`/`pole` = those **component
   IDs**. `type: "ik_two_bone"`, `bound: true`.
9. Use empty solver and deformer modules unless you also author graph
   nodes. Leave `content_hash` empty.
10. Omit `weights` and `deformer` for skeleton-only.

Worked IDs (illustrative, not required values):

```
joints packed:  [0]=root id=1, [1]=mid id=2, [2]=end id=3
file parents:   root -1, mid 1, end 2
locator ids:    target=10, pole=11
controller ids: fk_root=20, ik_target=21, ik_pole=22
constraint:     root=1, mid=2, end=3, target=10, pole=11
```

---

## 7. Source map

| Topic | Code |
|---|---|
| Save / load | `src/orlviewer/project_serialization.cpp` |
| Node-graph layout (editor-only) | `src/orlviewer/qt/node_graph_editor.cpp` |
| Node-graph frame create | `src/orlviewer/node_graph/node_ops.cpp` |
| Component IDs, packed order | `src/orlexec/orlrig/component_store.hpp/.cpp` |
| Joint local/world, parent index | `src/orlexec/orlrig/joint.hpp` |
| Attachments | `src/orlviewer/component_manager.hpp/.cpp` |
| Two-bone chain / pole | `src/orlexec/orlrig/ik.hpp` |
| Constraint fallback vs graph | `src/orlviewer/vp/solver_feature.cpp` |
| Staged graph JSON | `src/orlgraph/graph_serialization.cpp` |
| Registry node defs | `src/orlexec/orlrig/graph_resources.cpp` |
| Controller shapes | `src/orlviewer/comps/controller_curves.hpp` |
