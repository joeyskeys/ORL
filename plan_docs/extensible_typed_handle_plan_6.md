# Extensible typed handles plan 6: effects and two-bone IK pilot

## Goal

Prove the design end to end on one production solver. Convert
`solver_ik_two_bone` from packed integer indices to exact joint/locator
handles, and derive its evaluation footprint from typed data-access effects
rather than kind-specific port metadata.

All other algorithms may remain on the legacy path during this period.

## Depends on

- [Plan 4](extensible_typed_handle_plan_4.md): exact storage-backed struct
  views.
- [Plan 5](extensible_typed_handle_plan_5.md): typed scene inputs.

## Owned submodule

- `src/orlcomp/orl_analysis.h`
- `src/orlcomp/orl_analysis.cpp`
- `src/orlcomp/orl_graph_import.cpp`
- `src/orlgraph/graph_ir.hpp` effect/footprint records
- `src/orlexec/orlrig/evaluation.hpp`
- `src/orlexec/orlrig/evaluation.cpp`
- `src/orlexec/orlrig/graph_resources.cpp`
- `resource/stdlib/solver/ik_two_bone.orl`
- solver execution/binding integration and related tests

## Access-effect decision

Read/write effects are inferred from field operations on registered
handle-backed views and propagated transitively through helper calls. ORL does
not add `in`, `out`, or `inout` parameter syntax.

Copying, forwarding, comparing, or type-testing a handle has no component-data
effect. Binding `Joint(handle)` has no effect by itself. Reading
`joint.field` contributes a read; assigning `joint.field = value` contributes
a write; both together produce read-write. A write with no transitive read is
naturally write-only without a separate declaration.

Do not keep string metadata as a second authority once typed effects exist for
this solver.

## Design fixed in this period

1. Effects identify a handle parameter, nominal component type, and read/write
   mode.
2. Helper-call effects are transitive.
3. Handle-backed view field reads and writes are the primitive effects.
4. `partial_propagation = "descendants"` remains algorithm-specific metadata.
5. Component identity is resolved from the incoming typed handle edge.
6. A runtime-selected joint identity remains conservative even though its
   nominal type is exact.

## Work items

### 1. Effect summaries

- Extend function summaries with parameter-relative handle effects.
- Seed field capability from registered exact views.
- Track parameter provenance through local handle aliases and
  call-argument remapping.
- Propagate effects through the call graph to a fixed point.
- Diagnose recursion/unknown external calls conservatively.
- Conservatively merge branch effects, then remove effects from dead
  compile-time-specialized `is` branches.

### 2. Graph import

- Emit typed resource effects for exported ORL nodes.
- Keep effects attached to stable parameter/port IDs rather than names in
  comma-separated metadata.
- Preserve source provenance for diagnostics.

### 3. Evaluation-plan resolution

- Require a successfully validated graph before footprint compilation; do not
  let evaluation planning accept a connection that graph validation rejects.
- Follow each affected handle input edge to its stable scene input identity.
- Populate joint/locator read/write sets from handle type and the inferred
  parameter effect summary.
- Generalize the current joint-only packed-index bookkeeping where locator
  dirty-region tracking needs an equivalent identity map.
- Preserve existing hierarchy expansion and descendant propagation.
- If identity cannot be proven, mark the region global or use the existing
  runtime dirty-set mechanism; never guess a slot.

### 4. Two-bone IK source conversion

- Replace all five integer parameters with:
  - three `joint_handle` values;
  - two `locator_handle` values.
- Replace direct `solver_context.*[index]` access and index bounds logic with
  exact storage-backed view conversions and ordinary field access.
- Keep hierarchy relationship and numerical validity checks that are intrinsic
  to the IK algorithm.
- Assign root and mid fields directly; do not call a writeback function.
- Remove this solver's `partial_*_{joint,locator}_ports` metadata only after
  effect tests pass.

### 5. End-to-end execution

- Connect typed `find_*().handle` outputs directly to the solver.
- Verify CPU and CUDA results against the legacy implementation/fixtures.
- Confirm graph compilation fails before JIT on kind mismatch or unresolved
  required identity.

## Tests

Extend:

- `tests/analysis_tests/analysis_tests.cpp`
- `tests/rig_graph_tests/rig_graph_tests.cpp`
- `tests/orlrig_tests/orlrig_tests.cpp`
- `tests/scene_graph_tests/scene_graph_tests.cpp`
- `tests/gpu_tests/gpu_tests.cpp`
- `tests/syntax_tests/language_syntax_tests.cpp`

Required cases:

- direct, aliased, and transitively called field effects;
- copying/comparing a handle produces no component-data effect;
- view binding without field access produces no component-data effect;
- field assignment without a field read is inferred as write-only;
- whole-view copy, return, argument passing, and persistence are rejected;
- root/mid write and root/mid/end read sets;
- target/pole locator read sets;
- descendant propagation remains correct;
- locator-to-joint connection is rejected;
- evaluation-plan compilation cannot bypass graph connection validation;
- unresolved identity becomes conservative, not an unsafe sparse plan;
- typed CPU IK matches the legacy numerical result;
- conditional CUDA parity;
- generated/imported solver definition contains no kind-specific port metadata;
- malformed hierarchy and numerical edge cases still report correctly.

Suggested verification:

```sh
cmake --build build --target \
  orl_analysis_tests orl_rig_graph_tests orlrig_tests \
  orl_scene_graph_tests orl_gpu_tests orl_syntax_tests
./build/tests/analysis_tests/orl_analysis_tests
./build/tests/rig_graph_tests/orl_rig_graph_tests
./build/tests/orlrig_tests/orlrig_tests
./build/tests/scene_graph_tests/orl_scene_graph_tests
./build/tests/gpu_tests/orl_gpu_tests
./build/tests/syntax_tests/orl_syntax_tests
```

## Exit criteria

- Two-bone IK has no authored packed-index parameters or checks.
- Its kind-specific partial-port metadata is gone.
- Evaluation reads/writes are derived from typed effects.
- CPU/GPU and partial-evaluation behavior match the legacy baseline.
- Other legacy algorithms remain supported and unchanged.

## Not in scope

- Named unions or `is`.
- Constraint/deformer bulk migration.
- Removal of shared legacy infrastructure.
