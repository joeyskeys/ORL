# ORL Node Reference

This document describes the current node model, the registered graph nodes,
and the ORL standard-library procedures that are suitable as node
implementations.

The catalog intentionally distinguishes between:

- **Registered graph nodes**: definitions currently registered in
  `orlrig::NodeRegistry` and usable by the hand-built LBS graph.
- **Stdlib node candidates**: ORL procedures that can be compiled as
  standalone entry functions and can be imported into the graph registry.
- **Helpers**: procedures used by another node implementation; they are not
  intended to be exposed as user-facing graph nodes.

`src/orlexec/orlrig/graph_resources.cpp` explicitly registers the two LBS
runtime definitions plus the public auto-weight, solver, and constraint
stdlib definitions. The standard LBS graph still instantiates only the two
LBS runtime nodes; registration makes the other definitions available to the
node browser and graph authoring code. Source implementation and runner
support are still separate concerns and are noted below.

## 1. Graph node model

### 1.1 Node definition

A `NodeDefinition` describes a reusable operation. It contains:

- `id`: stable identity used by serialized graphs and connections;
- `qualified_name`: display and lookup name;
- `version`: definition version;
- `implementation`: ORL function, subgraph, builtin, or runtime adapter;
- `inputs` and `outputs`: typed ports;
- `parameters`: named specialization/runtime parameters;
- `effects`: resources read, written, or read/write;
- `capabilities`: supported execution backends;
- `inline_policy`: whether graph optimization may inline the node;
- `pure`: whether the node has no observable side effects;
- `stateful`: whether the result depends on persistent evaluation state;
- `provenance`: source locations and origin information.

The registry supports lookup by either stable ID or qualified name. Duplicate
definition IDs are rejected.

### 1.2 Node instance

A `NodeInstance` is one use of a definition inside a `GraphModule`. It adds:

- a stable instance ID;
- a definition reference;
- a user-facing instance name;
- constant parameter values;
- parameter mappings;
- provenance and an optional inline-policy override.

Instance IDs identify nodes in connections. Definition IDs identify the
operation implemented by those nodes.

### 1.3 Ports

Each port has:

- a stable port ID and display name;
- a direction: input, output, or in/out;
- a cardinality: scalar, array, or buffer;
- a logical type;
- a domain such as constant, joint, vertex, or buffer;
- a shape expression;
- required/optional status;
- an optional default value;
- optional semantic and coordinate-space tags.

The current ORL importer maps:

- a scalar formal parameter to a scalar input port;
- a buffer formal parameter to a buffer input port with a
  `<parameter_name>_count` shape symbol;
- a non-void return value to an output port named `result`.

Hand-authored runtime definitions can expose more accurate output and effect
contracts than the generic importer. This is why the LBS graph uses explicit
runtime definitions.

### 1.4 Connections and effects

Connections are directed:

```text
graph input or node output -> node input or graph output
```

Connections may also carry a conversion, shape rule, provenance record, and a
feedback flag. Ordinary graph cycles are invalid; feedback is reserved for
explicit stateful or iterative nodes.

Effects are independent of dataflow. A node that writes an observable
resource must remain live even if its scalar return value is unused.

## 2. Shared data and ABI conventions

The following conventions apply to the current rigging nodes.

### 2.1 Logical types

- `point`: a position represented by four doubles in an ORL buffer.
- `vector`: a direction/offset; it does not imply translation.
- `matrix`: a row-major 4x4 transform.
- `Joint`: named ORL struct corresponding to the 128-byte host/GPU ABI.
- `Weight`: named ORL struct corresponding to the 16-byte host/GPU ABI.
- `int`: signed integer scalar. Buffer index/count arguments are passed as
  scalar `int` parameters even when their host storage uses the ORL integer
  slot size.
- `float`: scalar single-precision value. The host runner packs it using the
  ORL floating-point slot size.

### 2.2 Matrix layout

The current matrix helpers use row-major indexing:

```text
[ 0  1  2  3 ]
[ 4  5  6  7 ]
[ 8  9 10 11 ]
[12 13 14 15 ]
```

Translation is stored at indices `[3]`, `[7]`, and `[11]`. Transforming the
origin with `matrix * point(0, 0, 0)` extracts the world-space translation.

### 2.3 Joint layout and hierarchy

`Joint` contains:

- `parent`: parent joint index, or `-1` for a root;
- `selected`, `pad0`, and `pad1`: storage/viewer fields;
- `translation`: local translation in a `vec4`;
- `rotation`: local quaternion;
- `scale`: local scale in a `vec4`.

`joint_world_matrix` composes the local matrix with each ancestor. Joint
indices are zero-based. Invalid negative indices generally mean “no joint”;
out-of-range positive indices are rejected by public solver procedures.

### 2.4 Weight layout

`Weight` contains:

- `weight`: influence value;
- `joint`: influenced joint index, or `-1` for an unused slot.

Weight buffers are vertex-major:

```text
weights[vertex * weight_cnt + slot]
```

The normal form is a fixed number of slots per vertex. Unused slots have
`weight == 0` and `joint == -1`. Auto-weight algorithms normalize the
selected slots so their weights sum to one whenever a valid influence exists.

### 2.5 Return-value convention

The current procedures use their integer return value as a status/count:

- map-style operations return the number of processed vertices;
- FK returns the number of processed joints;
- constraints and solvers return `1` for success and `0` for rejected input or
  an invalid configuration;
- output buffers are mutated in place through buffer parameters.

## 3. Registered graph nodes

These are the rig node definitions registered by
`register_rig_node_definitions`. The standard LBS graph instantiates the two
LBS definitions in sections 3.1 and 3.2; the stdlib definitions below are
available for authored graphs.

### 3.1 `orlrig.deformer.lbs.capture_bind`

**Stable ID and implementation**

- ID: `orlrig.deformer.lbs.capture_bind`
- Qualified name: `orlrig.deformer.lbs.capture_bind`
- Implementation kind: `Runtime`
- Runtime name: `orlrig.deformer.lbs.capture_bind`
- Pure: `false`
- Inline policy: `Never`

**Inputs**

- `joints`
  - type: `buffer<Joint>`
  - shape: `joint_count`
  - required: yes
  - effect: read from the joints resource

**Outputs**

- `inverse_binds`
  - type: `buffer<matrix>`
  - shape: `joint_count`
  - required: no because the node creates/writes the output resource
  - effect: writes the inverse-bind resource

**Behavior**

For every joint, the node computes its current world matrix and stores its
inverse in `inverse_binds[joint]`. The resulting matrix is later multiplied
by the posed world matrix during LBS evaluation.

This node captures the bind pose; it must run after the bind-pose joints are
available and before the corresponding deformation evaluation.

**Graph usage**

The standard LBS graph creates an instance named `capture_bind` and connects:

```text
graph input joints -> capture_bind.joints
capture_bind.inverse_binds -> deform.inverse_binds
```

### 3.2 `orlrig.deformer.lbs.evaluate`

**Stable ID and implementation**

- ID: `orlrig.deformer.lbs.evaluate`
- Qualified name: `orlrig.deformer.lbs.evaluate`
- Implementation kind: `Runtime`
- Runtime name: `orlrig.deformer.lbs.evaluate`
- Pure: `false`
- Inline policy: `Never`

**Inputs**

- `bind_positions`
  - type: `buffer<point>`
  - shape: `vertex_count`
  - required: yes
  - effect: read
- `joints`
  - type: `buffer<Joint>`
  - shape: `joint_count`
  - required: yes
  - effect: read
- `inverse_binds`
  - type: `buffer<matrix>`
  - shape: `joint_count`
  - required: yes
  - effect: read
- `weights`
  - type: `buffer<Weight>`
  - shape: `weight_count`
  - required: yes
  - effect: read

**Outputs**

- `posed_positions`
  - type: `buffer<point>`
  - shape: `vertex_count`
  - required: no because it is an observable output resource
  - effect: write

**Behavior**

For each vertex, the node reads the bind position and all
`weight_cnt` influence cells. For every valid influence:

```text
world  = joint_world_matrix(joints, weight.joint)
skin   = world * inverse_binds[weight.joint]
posed += weight.weight * (skin * bind_position)
```

The result is written to `posed_positions[vertex]`. Invalid influence cells
with a negative joint index are skipped.

**Graph usage**

The standard graph connects:

```text
graph input bind_positions -> deform.bind_positions
graph input joints         -> deform.joints
graph input weights        -> deform.weights
capture_bind.inverse_binds -> deform.inverse_binds
deform.posed_positions     -> graph output posed_positions
```

### 3.3 Scene lookup input nodes

The runtime registry also exposes two scene lookup nodes:

- `orlrig.input.find_joint`
- `orlrig.input.find_controller`

Both are `Runtime` nodes with a compile-time string parameter named `name` and
one scalar `int` output named `handle`. In the viewer, the node editor renders
`name` as a scene-backed combobox:

- `find_joint` lists existing joint component names and resolves the selected
  name to its current packed-joint index.
- `find_controller` lists existing controller component names and resolves the
  selected name to its component handle.

The output is intentionally represented as `int` because the current graph IR
has no opaque-handle type. A runtime dispatcher interprets the value according
to the node definition's semantic (`scene.joint_handle` or
`scene.controller_handle`). The scene list is refreshed as the viewer scene
changes; a saved graph stores the selected name, not a transient component
pointer.

## 4. Deformer node candidates

### 4.1 `deformer/lbs`

The source module is `resource/stdlib/deformer/lbs.orl`.

#### `deformer_lbs_capture_bind`

Signature:

```orl
int deformer_lbs_capture_bind(
    Joint joints[],
    matrix inverse_binds[],
    int joint_count
)
```

Inputs and effects:

- `joints[]`: bind-pose joint buffer, read;
- `joint_count`: number of joints;
- `inverse_binds[]`: matrix buffer written in place.

The function is the ORL implementation behind the registered runtime
`capture_bind` definition. Its return value is `joint_count`.

#### `deformer_lbs`

Signature:

```orl
int deformer_lbs(
    point bind_positions[],
    point output_positions[],
    Joint joints[],
    matrix inverse_binds[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt
)
```

Inputs and effects:

- `bind_positions[]`: rest-pose vertex positions, read;
- `joints[]`: current local joint pose, read;
- `inverse_binds[]`: captured inverse bind matrices, read;
- `weights[]`: vertex-major influence cells, read;
- `vertex_count`, `joint_count`, `weight_cnt`: extents;
- `output_positions[]`: posed positions, written.

The vertex loop is marked `parallel for`, so it is suitable for the CUDA
backend. The graph-level runtime definition exposes the output as
`posed_positions` and declares the output write observable.

## 5. Auto-weight node definitions and source procedures

Each public procedure in this section has a registered graph definition with
the corresponding `orlrig.auto_weight.*` ID. The definition uses the
procedure's buffer parameters as input ports, exposes its integer return as
`result`, declares buffer writes as observable effects, and advertises CPU
and CUDA capability when the implementation contains a parallel vertex pass.

Auto-weight functions operate on a mesh position buffer, a joint buffer, and
an output `Weight` buffer. Most algorithms use one slot per vertex as a hard
fallback and then write up to `weight_cnt` influences.

The common helper module is `auto_weight/common`; it is not itself a public
node.

### 5.1 `auto_weight_closest_joint`

Source: `resource/stdlib/auto_weight/closest_joint.orl`

Signature:

```orl
int auto_weight_closest_joint(
    point positions[],
    Joint joints[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt,
    float dropoff
)
```

Behavior:

- computes distance from each vertex to each joint world-space origin;
- keeps the two nearest joints;
- assigns inverse-distance weights `1 / (distance + epsilon)^dropoff`;
- normalizes the pair;
- fills unused slots with empty weights.

This algorithm uses joint origins, not bone segments. A zero/invalid result
falls back to a hard weight on joint `0`.

Current runner status: the source exists, but `AutoWeightRunner` does not
currently list `closest_joint` in its accepted algorithm names.

### 5.2 `auto_weight_closest_distance`

Source: `resource/stdlib/auto_weight/closest_distance.orl`

Signature:

```orl
int auto_weight_closest_distance(
    point positions[],
    Joint joints[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt,
    float dropoff
)
```

Behavior:

This is an alias of `auto_weight_closest_joint`. It uses the two nearest joint
origins and inverse-distance falloff, without hierarchy filtering.

Current runner status: supported by `AutoWeightRunner` under the algorithm
name `closest_distance`.

### 5.3 `auto_weight_closest_hierarchy`

Source: `resource/stdlib/auto_weight/closest_hierarchy.orl`

Signature:

```orl
int auto_weight_closest_hierarchy(
    point positions[],
    Joint joints[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt,
    float dropoff
)
```

Behavior:

1. Finds the closest joint origin.
2. Restricts candidates to the closest joint, its parent, or its direct
   children.
3. Chooses the two nearest candidates.
4. Applies the same inverse-distance falloff as
   `auto_weight_closest_distance`.

The hierarchy restriction prevents an unrelated nearby limb from receiving
the second influence.

Current runner status: supported by `AutoWeightRunner` as
`closest_hierarchy`.

### 5.4 `auto_weight_envelope`

Source: `resource/stdlib/auto_weight/envelope.orl`

Signature:

```orl
int auto_weight_envelope(
    point positions[],
    Joint joints[],
    float radii[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt
)
```

Inputs:

- `positions[]`: mesh vertices;
- `joints[]`: joint hierarchy;
- `radii[]`: one radius per joint;
- `vertex_count`, `joint_count`, `weight_cnt`: extents;
- `weights[]`: output.

Behavior:

- treats each joint-to-parent segment as a capsule-like influence region;
- computes squared distance to the segment;
- computes a quadratic radial falloff;
- chooses the strongest unique `weight_cnt` influences;
- normalizes the selected values.

The implementation ensures the radius is at least the bone length and uses a
default radius when a radius is nearly zero. If no positive influence is
found, it assigns a hard weight to joint `0`.

This is an envelope-style approximation, not a volumetric collision or
distance-field solver.

Current runner status: the source exists, but `AutoWeightRunner` does not
currently list `envelope`.

### 5.5 `auto_weight_heat`

Source: `resource/stdlib/auto_weight/heat.orl`

Signature:

```orl
int auto_weight_heat(
    point positions[],
    int offsets[],
    int neighbors[],
    Joint joints[],
    float radii[],
    float scratch[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt
)
```

Additional inputs:

- `offsets[]`: CSR row offsets, with at least `vertex_count + 1` entries;
- `neighbors[]`: CSR neighbor vertex indices;
- `radii[]`: one radius per joint;
- `scratch[]`: `vertex_count * joint_count` temporary heat values.

Behavior:

- identifies the closest bone segment for every vertex;
- pins vertices near a bone or with a clearly dominant closest bone;
- initializes pinned heat to one for the closest joint;
- performs 16 neighbor-average relaxation passes;
- extracts the strongest `weight_cnt` joint fields and normalizes them.

The implementation is a fixed-iteration Laplacian relaxation. It is in the
Heat Map family, but it is not a full sparse linear-system solve.

Current runner status: supported by `AutoWeightRunner` as `heat`. A mesh CSR
must be supplied.

### 5.6 `auto_weight_geodesic`

Source: `resource/stdlib/auto_weight/geodesic.orl`

Signature:

```orl
int auto_weight_geodesic(
    point positions[],
    int offsets[],
    int neighbors[],
    Joint joints[],
    float scratch[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt
)
```

Additional inputs:

- `offsets[]` and `neighbors[]`: mesh CSR adjacency;
- `scratch[]`: `vertex_count * joint_count` distance/score storage.

Behavior:

- initializes each vertex/joint distance with distance to the joint's
  parent-child segment;
- relaxes distances across the mesh graph for 32 passes;
- converts distance to inverse-square influence;
- keeps and normalizes the strongest `weight_cnt` influences.

This is a surface-graph approximation of geodesic voxel weighting. It does
not build a filled voxel volume, so it cannot reproduce Maya's complete
Geodesic Voxel behavior around cavities or disconnected surface regions.

Current runner status: supported by `AutoWeightRunner` as `geodesic`. A mesh
CSR must be supplied.

### 5.7 `auto_weight_harmonic`

Source: `resource/stdlib/auto_weight/harmonic.orl`

Signature:

```orl
int auto_weight_harmonic(
    point positions[],
    int offsets[],
    int neighbors[],
    Joint joints[],
    float radii[],
    float scratch[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt
)
```

Behavior:

- pins a wider neighborhood around the closest bone than the heat variant;
- performs 24 neighbor-average relaxation passes;
- extracts the top `weight_cnt` fields and normalizes them.

The implementation is a fixed-iteration combinatorial Laplacian relaxation.
It is not yet a full harmonic-coordinate linear solve.

Current runner status: source implementation exists, but the current
`AutoWeightRunner` algorithm whitelist does not expose `harmonic`.

### 5.8 `auto_weight_bounded_biharmonic`

Source: `resource/stdlib/auto_weight/bounded_biharmonic.orl`

Signature:

```orl
int auto_weight_bounded_biharmonic(
    point positions[],
    int offsets[],
    int neighbors[],
    Joint joints[],
    float radii[],
    float scratch[],
    Weight weights[],
    int vertex_count,
    int joint_count,
    int weight_cnt
)
```

Behavior:

- uses the same pinning and neighbor relaxation structure as the harmonic
  implementation;
- clamps each relaxed field to `[0, 1]`;
- renormalizes the fields to partition unity after each update;
- extracts the strongest `weight_cnt` influences.

This is explicitly an approximation. It is not the full bounded-biharmonic
quadratic-program solve described in the research literature.

Current runner status: source implementation exists, but the current
`AutoWeightRunner` algorithm whitelist does not expose it.

### 5.9 Auto-weight helpers

`resource/stdlib/auto_weight/common.orl` provides shared procedures:

- `auto_weight_segment_distance2`;
- `auto_weight_segment_distance`;
- `auto_weight_dropoff`;
- `auto_weight_joint_origin`;
- `auto_weight_joint_bone_distance`;
- `auto_weight_bone_taken`;
- `auto_weight_is_relative`;
- `auto_weight_write_hard`;
- `auto_weight_write_two_dropoff`;
- `auto_weight_write_topk`.

These helpers are implementation details. They should not normally appear as
standalone nodes because they do not define a complete mesh-to-weight
operation.

## 6. Solver node definitions and source procedures

Each public procedure in this section has a registered
`orlrig.solver.*` definition. Solver definitions are effectful; joint and
history buffers that are updated in place are declared read/write. The
history-dependent solver is also marked stateful so feedback edges can be
validated for it.

Solver procedures mutate the supplied `Joint` buffer. They are generally
stateful or effectful from a graph perspective and should not be optimized as
pure arithmetic nodes.

### 6.1 `solver_fk`

Source: `resource/stdlib/solver/fk.orl`

Signature:

```orl
int solver_fk(
    Joint joints[],
    matrix world[],
    int joint_count
)
```

Behavior:

- reads local joint transforms and parent indices;
- computes one world matrix per joint;
- writes `world[index]`;
- returns `joint_count`.

The local joint buffer is not rewritten. `solver_fk_world_matrix` is the
per-joint helper and can be used when only one world matrix is needed.

The implementation walks parent links with a bounded step count. Parent-first
ordering is supported, but callers should still provide a valid acyclic
hierarchy.

Current runner status: no `SolverRunner` method currently exposes FK.

### 6.2 `solver_ik_two_bone`

Source: `resource/stdlib/solver/ik_two_bone.orl`

Signature:

```orl
int solver_ik_two_bone(
    Joint joints[],
    int root,
    int mid,
    int end,
    matrix target[],
    matrix pole[],
    int joint_count
)
```

Inputs:

- `joints[]`: mutable local pose;
- `root`, `mid`, `end`: indices of a strict three-joint chain;
- `target[]`: one-element target transform buffer;
- `pole[]`: one-element pole transform buffer;
- `joint_count`: joint extent.

Behavior:

- validates indices and the `root -> mid -> end` parent relationship;
- measures upper and lower bone lengths;
- clamps target distance to reachable range;
- uses the pole position to select the bend plane;
- rotates root and mid;
- leaves translations and the end-joint rotation unchanged.

Return value is `1` on success and `0` if the chain, lengths, or target are
invalid.

Current runner status: this is the only solver exposed by the current
`SolverRunner`, through `evaluate_two_bone`.

### 6.3 `solver_hd_id`

Source: `resource/stdlib/solver/hd_id.orl`

Signature:

```orl
int solver_hd_id(
    Joint joints[],
    Joint history[],
    int root,
    int end,
    matrix target[],
    int joint_count,
    int iterations
)
```

Behavior:

- validates that `root` is an ancestor of `end`;
- copies valid previous rotations from `history[]` into the current pose;
- runs CCD passes from the end parent toward the root;
- writes the solved pose back to `joints[]`;
- copies the resulting pose to `history[]`.

`history[]` is both input and output. The caller must preserve it between
evaluations to obtain the intended history-dependent continuity. Passing a
fresh history buffer every frame removes the warm-start behavior.

The target is a one-element matrix buffer. `iterations < 1` is promoted to
one iteration.

Current runner status: source implementation exists, but no runner method or
graph definition currently exposes it.

### 6.4 `solver_spline_ik`

Source: `resource/stdlib/solver/spline_ik.orl`

Signature:

```orl
int solver_spline_ik(
    Joint joints[],
    int chain[],
    point spline[],
    int chain_count,
    int point_count,
    int joint_count
)
```

Inputs:

- `chain[]`: joint indices in root-to-end order;
- `spline[]`: world-space Catmull-Rom control points;
- `chain_count`, `point_count`, `joint_count`: extents.

Behavior:

- validates every chain index and adjacent parent relationship;
- samples the spline at evenly spaced chain parameters;
- computes a tangent for each chain segment;
- rotates the corresponding pivot toward the tangent;
- preserves local translations.

The spline sampler clamps the parameter to `[0, 1]` and uses endpoint
duplication for the first and last Catmull-Rom segments.

Current runner status: source implementation exists, but no runner method or
graph definition currently exposes it.

### 6.5 `solver_full_body_ik`

Source: `resource/stdlib/solver/full_body_ik.orl`

Signature:

```orl
int solver_full_body_ik(
    Joint joints[],
    int effectors[],
    matrix targets[],
    int effector_count,
    int joint_count,
    int iterations
)
```

Inputs:

- `effectors[]`: end-joint indices;
- `targets[]`: one target matrix per effector;
- `iterations`: number of global CCD passes.

Behavior:

- validates all effector indices;
- processes effectors sequentially;
- for each effector, walks every ancestor;
- rotates each ancestor toward its target direction;
- repeats for the requested number of passes;
- returns `effector_count` on success.

This is a sequential CCD solver. When effectors share ancestors, the result
depends on effector ordering and the requested iteration count.

Current runner status: source implementation exists, but no runner method or
graph definition currently exposes it.

### 6.6 Solver helpers

The solver modules contain helpers such as:

- quaternion-from-two-directions routines;
- safe perpendicular-axis selection;
- joint-origin and world-to-local conversions;
- scalar absolute value and square-root approximations;
- Catmull-Rom spline sampling;
- chain validation.

These are not intended as independent graph nodes. The public node boundary
is the `solver_*` operation that mutates or produces a complete result.

## 7. Constraint node definitions and source procedures

Each public procedure in this section has a registered
`orlrig.constraint.*` definition. Constraint definitions expose source and
destination matrix buffers plus the index/count scalar inputs, and mark the
destination as an observable read/write effect.

Constraints are effectful transform operations. They modify a destination
matrix buffer and should normally be scheduled in graph order, rather than
treated as freely reorderable pure nodes.

All current constraint procedures use row-major matrices and preserve
components not owned by the individual constraint.

### 7.1 `constraint_aim`

Source: `resource/stdlib/constraint/aim.orl`

Signature:

```orl
int constraint_aim(
    matrix targets[],
    matrix subjects[],
    vector axes[],
    int target_index,
    int subject_index,
    int target_count,
    int subject_count
)
```

Behavior:

- reads the target translation at `targets[target_index]`;
- uses `axes[0]` as the subject's local aim axis;
- rotates `subjects[subject_index]` so that axis points toward the target;
- preserves subject translation and axis scales.

The axis buffer is currently a one-element input. The target and subject
indices are independently validated. The return value is `1` on success.

### 7.2 `constraint_copy_xform`

Source: `resource/stdlib/constraint/copy_xform.orl`

Signature:

```orl
int constraint_copy_xform(
    matrix source[],
    matrix destination[],
    int source_index,
    int destination_index,
    int source_count,
    int destination_count
)
```

Copies the complete source matrix to the destination matrix. Translation,
rotation, scale, and any remaining matrix fields are replaced.

### 7.3 `constraint_copy_translation`

Source: `resource/stdlib/constraint/copy_translation.orl`

Signature:

```orl
int constraint_copy_translation(
    matrix source[],
    matrix destination[],
    int source_index,
    int destination_index,
    int source_count,
    int destination_count
)
```

Copies only matrix entries `[3]`, `[7]`, and `[11]`. Destination orientation
and scale are preserved.

### 7.4 `constraint_copy_rotation`

Source: `resource/stdlib/constraint/copy_rotation.orl`

Signature:

```orl
int constraint_copy_rotation(
    matrix source[],
    matrix destination[],
    int source_index,
    int destination_index,
    int source_count,
    int destination_count
)
```

Copies the source orientation while preserving destination translation and
axis magnitudes. The source basis axes are normalized before being scaled by
the destination axis lengths.

### 7.5 `constraint_copy_scale`

Source: `resource/stdlib/constraint/copy_scale.orl`

Signature:

```orl
int constraint_copy_scale(
    matrix source[],
    matrix destination[],
    int source_index,
    int destination_index,
    int source_count,
    int destination_count
)
```

Copies the magnitudes of the source basis axes while preserving destination
orientation and translation.

### 7.6 Constraint helpers

`resource/stdlib/constraint/common.orl` provides:

- basis-column extraction;
- safe vector normalization;
- quaternion-from-vector rotation;
- translation copying;
- scale copying;
- rotation copying;
- aim rotation.

These helper procedures are implementation details and should not be
registered separately.

### 7.7 Current registration status

There is currently no `ConstraintRunner` and no constraint registration call
in `orlrig::register_rig_node_definitions`. The five constraint procedures
are therefore source-level node candidates only.

## 8. Core data modules

### 8.1 `joint`

Source: `resource/stdlib/joint.orl`

Import with:

```orl
use joint;
```

Public helpers:

- `joint_identity()`: creates an identity joint with parent `-1`;
- `joint_from_trs(parent, translation, rotation, scale)`;
- `joint_local_matrix(joint)`;
- `joint_world_matrix(joints, index)`;
- `joint_skin_matrix(world, inverse_bind)`.

These functions define the shared rig transform semantics. They are normally
inlined into deformer and solver implementations rather than exposed as
standalone graph nodes.

### 8.2 `weight`

Source: `resource/stdlib/weight.orl`

Import with:

```orl
use weight;
```

Public helpers:

- `weight_empty()`: returns `{ weight: 0, joint: -1 }`;
- `weight_bind(joint)`: returns a full influence on one joint;
- `weight_pair(weight, joint)`: constructs one influence cell.

These helpers define the canonical empty, hard-bind, and weighted-cell
representations.

## 9. Authoring and import examples

### 9.1 Compiling an external ORL file

The CMake target `orlc` provides a command-line compiler for external ORL
files. It performs parsing, ORL code generation, and CPU JIT loading by
default:

```text
orlc my_solver.orl --entry solver_main
orlc my_solver.orl --entry solver_main --emit-ir my_solver.ll
orlc my_solver.orl -I path/to/modules --backend cuda
```

Use `--print-ir` to print the generated LLVM/NVVM IR. Use `--emit-oro` to
analyze the source and export its functions as an `.oro` node-library
document:

```text
orlc my_nodes.orl --emit-oro my_nodes.oro --module my_nodes --export my_solver
```

The `.oro` option serializes graph definitions; it is not a native object
file and does not replace backend code generation.

### 9.2 Compiling a stdlib function directly

The current execution layer can compile a module and select an entry function:

```cpp
auto program = ORL::exec::OrlProgram::Compile(
    "use solver/ik_two_bone;\n",
    {
        .entry_function = "solver_ik_two_bone",
        .source_name = "orlrig_solver_ik_two_bone",
    });
```

The caller then creates an `OrlExecution`, binds every reflected parameter,
and evaluates the function.

### 9.3 Importing functions as definitions

`orlcomp::import_node_definitions` converts analyzed ORL functions into
generic `NodeDefinition` values:

```text
ORL function parameter -> input port
ORL buffer parameter   -> buffer input with a count shape
ORL non-void return    -> scalar output named result
parameter access       -> resource effect
parallel for           -> CUDA capability
pure function          -> default inline policy
```

For example, importing `solver_fk` yields a definition whose inputs
correspond to `joints`, `world`, and `joint_count`, plus a scalar `result`
output. A hand-authored definition may rename that result or expose an
in/out resource more accurately.

Applications can import and register external source without rebuilding:

```cpp
orlcomp::NodeImportOptions options;
options.module_name = "my_nodes";
options.source_name = "my_nodes.orl";
options.include_paths.push_back("path/to/modules");
options.exported_functions = {"my_solver"};
const auto result = orlcomp::register_orl_node_definitions_file(
    registry, "my_nodes.orl", options);
```

The source and file overloads parse, analyze, import, and preflight duplicate
definition IDs before adding them to the application-owned `NodeRegistry`.
`exported_functions` can restrict registration to public entry functions and
keep helper procedures out of the node browser.

### 9.4 Standard LBS graph

The current `orlrig::make_lbs_graph()` contains:

```text
bind_positions ───────────────> deform.bind_positions
joints ───────> capture_bind.joints
      └───────────────────────> deform.joints
weights ──────────────────────> deform.weights
capture_bind.inverse_binds ───> deform.inverse_binds
deform.posed_positions ───────> posed_positions
```

Its node instances are:

- `capture_bind` using `orlrig.deformer.lbs.capture_bind`;
- `deform` using `orlrig.deformer.lbs.evaluate`.

## 10. Runtime support matrix

This section records implementation status, not just source-file presence.

### Registered graph definitions

- `orlrig.deformer.lbs.capture_bind`: registered runtime node.
- `orlrig.deformer.lbs.evaluate`: registered runtime node.
- `orlrig.auto_weight.closest_joint`: registered ORL-function node.
- `orlrig.auto_weight.closest_distance`: registered ORL-function node.
- `orlrig.auto_weight.closest_hierarchy`: registered ORL-function node.
- `orlrig.auto_weight.envelope`: registered ORL-function node.
- `orlrig.auto_weight.heat`: registered ORL-function node.
- `orlrig.auto_weight.geodesic`: registered ORL-function node.
- `orlrig.auto_weight.harmonic`: registered ORL-function node.
- `orlrig.auto_weight.bounded_biharmonic`: registered ORL-function node.
- `orlrig.solver.fk`: registered ORL-function node.
- `orlrig.solver.ik_two_bone`: registered ORL-function node.
- `orlrig.solver.hd_id`: registered stateful ORL-function node.
- `orlrig.solver.spline_ik`: registered ORL-function node.
- `orlrig.solver.full_body_ik`: registered ORL-function node.
- `orlrig.constraint.aim`: registered ORL-function node.
- `orlrig.constraint.copy_xform`: registered ORL-function node.
- `orlrig.constraint.copy_translation`: registered ORL-function node.
- `orlrig.constraint.copy_rotation`: registered ORL-function node.
- `orlrig.constraint.copy_scale`: registered ORL-function node.

### Auto-weight runner names

Currently accepted by `AutoWeightRunner`:

- `closest_distance`;
- `closest_hierarchy`;
- `heat`;
- `geodesic`.

Source implementations not currently accepted by the runner:

- `closest_joint`;
- `envelope`;
- `harmonic`;
- `bounded_biharmonic`.

Algorithms requiring mesh adjacency (`heat`, `geodesic`, `harmonic`, and
`bounded_biharmonic`) require CSR `offsets[]` and `neighbors[]` data.

### Solver runner names

Currently exposed by `SolverRunner`:

- `evaluate_two_bone`, backed by `solver/ik_two_bone`.

Source-only solver candidates:

- `solver_fk`;
- `solver_hd_id`;
- `solver_spline_ik`;
- `solver_full_body_ik`.

### Constraint runner

No constraint runner or graph execution adapter exists yet. The five
constraint procedures are registered graph definitions and can be authored,
but they are not yet wired into a dedicated constraint runner.

## 11. Adding a new node

For an ORL function intended as a node:

1. Give the public entry function a stable, category-prefixed name such as
   `solver_new_method` or `constraint_new_rule`.
2. Keep helper functions separately named and document the public entry point.
3. Use explicit buffer/count pairs for variable-size data.
4. Validate indices and counts before reading or writing buffers.
5. State which buffers are read, written, or read/write.
6. Document coordinate spaces and matrix conventions.
7. Return a count or explicit success status consistently with neighboring
   nodes.
8. Add the algorithm to the corresponding runner whitelist or runner class.
9. Register a `NodeDefinition` if it should appear in graph-authored LBS/rig
   graphs.
10. Add CPU tests and conditional CUDA tests where the body contains
    `parallel for`.

For a hand-authored runtime node, also provide:

- stable definition ID and qualified name;
- explicit ports and shape symbols;
- resource effects;
- backend/runtime name;
- purity, statefulness, and inline policy;
- graph construction and validation coverage.

## 12. Known limitations

- The registry contains 20 definitions, but only the two LBS runtime nodes are
  instantiated by `make_lbs_graph()`.
- Generic ORL import currently represents formal parameters as input ports;
  it does not automatically infer a richer in/out port contract.
- Several source algorithms are intentionally approximations of their
  research or DCC counterparts, especially heat, geodesic, harmonic, and
  bounded biharmonic weighting.
- The stdlib solver and constraint definitions are not yet wired into a
  dedicated graph execution scheduler or runner adapter.
- The graph editor currently renders the graph module's registered node
  instances; it is not a complete node browser or authoring system.
