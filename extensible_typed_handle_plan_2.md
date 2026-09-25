# Extensible typed handles plan 2: graph type system and persistence

## Goal

Carry exact nominal handle types through graph IR, validation, reflection, and
serialization without changing execution or scene input nodes.

At the end of this period hand-built graphs and imported ORL definitions can
contain exact handle ports. Type-invalid connections fail during graph
validation, and a serialized graph preserves nominal identity deterministically.

## Depends on

- [Plan 1](extensible_typed_handle_plan_1.md) exposes resolved handle
  declarations and function signatures.

## Owned submodule

Target-independent graph types and ORL graph import:

- `src/orlgraph/graph_types.hpp`
- `src/orlgraph/graph_types.cpp`
- `src/orlgraph/graph_ir.hpp`
- `src/orlgraph/graph_ir.cpp`
- `src/orlgraph/graph_validation.cpp`
- `src/orlgraph/graph_serialization.cpp`
- `src/orlgraph/graph_reflection.hpp`
- `src/orlgraph/graph_reflection.cpp`
- `src/orlcomp/orl_graph_import.cpp`
- `graph_json_format.md`

This layer must remain independent of `orlexec`, `orlrig`, the viewer, LLVM,
and CUDA.

## Design fixed in this period

1. Add `LogicalTypeKind::Handle`.
2. An exact handle logical type contains its canonical package-prefixed name.
3. Equality and hashing include nominal identity.
4. Exact `T -> T` is valid; `T -> U`, `int -> T`, and `T -> int` are invalid.
5. Universal and union assignability are reserved for plan 7.
6. Handle ports have scalar cardinality but are not numeric scalar constants.
7. Serialized handles contain no slot, pointer, packed index, or process-local
   registry ordinal.

## Work items

### 1. Logical type representation

- Add an exact-handle factory and canonical-name representation.
- Update `is_scalar`, equality, canonical naming, ordering/hashing users, and
  exhaustive `LogicalTypeKind` switches.
- Keep the representation extensible so plan 7 can add accepted leaf sets
  without changing the serialized exact-type spelling.

### 2. Validation and diagnostics

- Add a dedicated handle assignability helper instead of relying on raw
  `LogicalType::operator==`.
- Use it in graph connection/interface validation and
  `Port::compatible_value`; audit every exact-type comparison on a connection
  path.
- Report source and destination nominal names in mismatch diagnostics.
- Reject handle constants until a stable graph identity value is defined in
  plan 5.

### 3. Deterministic serialization

- Bump the graph logical ABI/schema version.
- Serialize exact handle identity by canonical package-prefixed name. The
  first schema has no separate stable declaration ID.
- Reject unknown/malformed handle records without coercing them to `Unknown`
  or `Int64`.
- Add deterministic round-trip and old-schema compatibility fixtures.

### 4. Reflection and ORL import

- Reflect handle port types and nominal IDs to editor/backend clients.
- Map plan-1 exact handle parameters and returns directly to handle logical
  types in `orl_graph_import.cpp`.
- Remove the current heuristic only for typed handle ports; legacy integer
  scene-index inference remains until the final migration period.

## Tests

Extend:

- `tests/graph_ir_tests/graph_ir_tests.cpp`
- `tests/analysis_tests/analysis_tests.cpp`
- `tests/rig_graph_tests/rig_graph_tests.cpp` only for imported definitions

Required cases:

- exact-handle logical type equality/canonical naming;
- exact matching connection succeeds;
- joint-to-locator and handle-to-int connections fail;
- serialization round-trip preserves nominal identity;
- malformed and unknown handle records fail with stable diagnostics;
- reflected/imported ports retain handle type;
- graphs containing only legacy types serialize byte-for-byte as before,
  except for an intentional schema-version change if required.

Suggested verification:

```sh
cmake --build build --target \
  orl_graph_ir_tests orl_analysis_tests orl_rig_graph_tests
./build/tests/graph_ir_tests/orl_graph_ir_tests
./build/tests/analysis_tests/orl_analysis_tests
./build/tests/rig_graph_tests/orl_rig_graph_tests
```

## Exit criteria

- Graph IR can represent and persist exact handles without rig dependencies.
- Incompatible exact handle connections are rejected before lowering.
- ORL import no longer erases exact handle identity.
- Graph execution still rejects handle ports as unsupported; plan 3 owns that
  boundary.

## Not in scope

- Runtime values or binding.
- Scene input node changes.
- Data resolvers/conversions.
- Universal handles, named unions, `is`, or specialization.
