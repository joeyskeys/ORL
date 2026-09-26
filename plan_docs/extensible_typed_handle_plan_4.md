# Extensible typed handles plan 4: `orlrig` registry and typed data access

## Goal

Make exact rig handles useful by registering their nominal types, resolving
dispatch-local slots into component storage, and binding storage-backed typed
views through exact handle-to-struct conversions.

This period is tested with manually bound handles. Scene `find_*` nodes remain
unchanged until plan 5.

## Depends on

- [Plan 3](extensible_typed_handle_plan_3.md) transports exact handle tokens.

## Owned submodule

- `src/orlexec/orlrig/graph_resources.hpp`
- `src/orlexec/orlrig/graph_resources.cpp`
- `src/orlexec/orlrig/component_store.hpp`
- `src/orlexec/orlrig/component_store.cpp`
- `src/orlexec/orlrig/abi.hpp`
- `src/orlexec/orlrig/joint.hpp`
- `src/orlexec/orlrig/locator.hpp`
- a new `orlrig` handle registry/context if separation is clearer
- `resource/stdlib/rig/handles.orl` and exact-type helper modules
- compiler extension hooks required for registered views and field lowering

## Identity mapping decision

The package-prefixed canonical name is the nominal identity and the value
stored by serialization. The 64-bit runtime `type_id` is a versioned
deterministic hash of that canonical name. Registry construction must detect
collisions and fail. The first design has no separate stable declaration ID,
so renaming or moving a handle type requires an explicit migration.

Do not use registration order, C++ `typeid().hash_code()`, addresses, or
compiler-specific RTTI names.

## Design fixed in this period

1. `orlrig` declares `joint_handle` and `locator_handle`; core ORL does not
   know these names.
2. Registration binds a nominal type to:
   - stable runtime tag;
   - supported CPU/GPU backends;
   - storage arena/resolver;
   - supported destination-struct views;
   - read and write lowering for every exposed field.
3. Conversion syntax is exact-type only, such as `Joint(joint_handle)`,
   `Locator(locator_handle)`, and `WorldTransform(joint_handle)`.
4. Conversion binds a local, non-escaping lvalue view. Reading a field loads
   component storage and assigning a field stores it immediately.
5. Direct handle member access remains illegal.
6. There are no named handle accessors. Data access after conversion uses
   ordinary struct fields such as `WorldTransform(handle).xform`.
7. Every exact source-handle/destination-struct pair has at most one
   conversion. Different semantics require different destination structs, not
   ambiguous conversions to the same physical type.
8. Every registered field is read/write capable. If both directions cannot be
   implemented consistently, the view conversion is not registered.
9. Whole handle-backed views cannot be copied, returned, passed, buffered, or
   persisted. Their individual fields remain ordinary values.
10. Generated exact-type code calls one selected resolver. It does not switch
   on `type_id`.
11. Controllers have no handle or typed view. They remain attached
    joint/locator authoring utilities outside compiled rig dataflow.

## Work items

### 1. Registry and descriptor API

- Define immutable descriptors keyed by canonical nominal identity.
- Validate duplicate names/tags, unsupported targets, missing resolvers,
  duplicate source/destination views, and incomplete field read/write
  lowering.
- Expose descriptors to semantic analysis, graph import, and codegen through
  a target-independent interface.
- Keep C++ RTTI, if used internally by a CPU adapter, behind the descriptor.
- Implement descriptors as a C++ `orlrig` registration API. Do not add ORL
  annotations or a separate manifest; graph authors never write descriptors.

### 2. Runtime handle context

- Add a versioned context rather than changing the existing six-lane
  `SolverContext` silently.
- Represent storage with device-portable bases/offsets, count, stride, and
  access policy; never embed host-only pointers in a GPU-visible record.
- Check null/wrong-type tags, slot bounds, backend support, and write
  permission.
- Record the compiled rig's topology/packing revision at the dispatch
  boundary and reject execution if it has changed.
- Keep handles dispatch-local and prohibit topology repacking for the complete
  lifetime of that compiled rig.

### 3. Exact component access

- Register `joint_handle -> Joint` and `locator_handle -> Locator`.
- Define semantic view structs such as `WorldTransform` for computed
  data that is not a field of `Joint`.
- Register exact conversions such as `joint_handle -> WorldTransform`.
- Keep joint world-transform hierarchy traversal and inverse write mapping
  inside the corresponding field lowering, separate from stored `Joint`
  fields.

### 4. Compiler extension hooks

- Resolve a registered constructor conversion only when the handle flow type
  is exact and that source/destination pair has one registered conversion.
- Track the conversion result as a distinct handle-backed lvalue category even
  though source syntax names the destination struct.
- Lower field reads and assignments through target-independent descriptor
  hooks.
- Reject whole-view copies, returns, arguments, buffers, persistent storage,
  and other escaping aliases.
- Reject unavailable backends and incomplete or contradictory field
  lowerings before LLVM emission.
- Include registry/descriptor ABI versions in compilation cache keys.

## Tests

Extend:

- `tests/analysis_tests/analysis_tests.cpp`
- `tests/codegen_tests/codegen_tests.cpp`
- `tests/exec_tests/exec_tests.cpp`
- `tests/orlrig_tests/orlrig_tests.cpp`
- `tests/gpu_tests/gpu_tests.cpp`

Required cases:

- deterministic registry IDs and collision/duplicate rejection;
- canonical-name serialization and an explicit migration diagnostic for a
  renamed type;
- manually bound joint and locator field read/write on CPU;
- world-transform conversion followed by ordinary `.xform` read and
  assignment;
- joint world-transform fields use hierarchy-aware read and inverse-write
  behavior;
- inferred effects distinguish field read, field write, and read-write;
- whole-view copy/return/persistence fails semantic analysis;
- wrong tag, invalid token, changed compiled-rig revision, and out-of-range
  slot fail safely;
- locator passed to a joint conversion fails during semantic analysis;
- direct field access on a handle still fails;
- CPU/CUDA parity for exact field read/write paths;
- a custom test-only handle type proves the API is not hard-coded to rig
  component names.

Suggested verification:

```sh
cmake --build build --target \
  orl_analysis_tests orl_codegen_tests \
  orl_exec_tests orlrig_tests orl_gpu_tests
./build/tests/analysis_tests/orl_analysis_tests
./build/tests/codegen_tests/orl_codegen_tests
./build/tests/exec_tests/orl_exec_tests
./build/tests/orlrig_tests/orlrig_tests
./build/tests/gpu_tests/orl_gpu_tests
```

## Exit criteria

- Exact rig handles can read/write supported component data without indices.
- Runtime failures are bounds/tag/lifetime diagnostics, never crashes.
- CPU and GPU access the same logical values.
- No input-node or legacy graph schema behavior changes.

## Not in scope

- `find_*` node output replacement.
- Evaluation-plan metadata removal.
- Named unions, universal narrowing, or `is`.
- Bulk solver/constraint/deformer migration.
