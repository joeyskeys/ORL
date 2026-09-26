# Extensible typed handles plan 3: compiler and execution ABI

## Goal

Define and implement target-independent transport for an exact handle through
LLVM, CPU JIT execution, graph execution, and CUDA/PTX.

This period treats a handle as opaque identity data. It does not resolve rig
storage. A minimal echo/equality kernel proves transport without coupling the
core runtime to joints or locators.

## Depends on

- [Plan 1](extensible_typed_handle_plan_1.md): exact handle semantic types.
- [Plan 2](extensible_typed_handle_plan_2.md): exact graph handle ports.

## Owned submodule

- `src/orlcomp/orl_runtime_signature.h`
- `src/orlcomp/orl_codegen.cpp`
- `src/orlcomp/orl_codegen.h`
- `src/orlcomp/orl_jit.cpp`
- `src/orlcomp/orl_jit.h`
- `src/orlcomp/orl_gpu.cpp`
- `src/orlcomp/orl_gpu.h`
- `src/orlcomp/orl_graph_lowering.cpp`
- `src/orlexec/orl_exec.hpp`
- `src/orlexec/orl_exec.cpp`
- `src/orlexec/orl_graph_exec.hpp`
- `src/orlexec/orl_graph_exec.cpp`
- compiler cache/version keys affected by the ABI

## Design fixed in this period

1. Introduce one shared-width value contract:

   ```cpp
   struct HandleValue {
       std::uint64_t type_id;
       std::int64_t slot;
   };
   ```

2. The C++ struct documents the value but does not define a platform aggregate
   calling convention.
3. Host and kernel wrappers flatten a handle deterministically into two
   64-bit lanes, or use an explicitly tested LLVM struct layout.
4. `type_id` is checked against the parameter's nominal type during binding.
5. A reserved token represents invalid/null. No implicit integer binding is
   accepted.
6. Equality/inequality compare opaque tokens; arithmetic and field extraction
   remain unavailable. Invalid tokens are rejected before data access.
7. CPU and GPU use the same lane widths, order, signedness, and alignment.
8. The first ABI has no generation lane. Tokens are created for one dispatch,
   cannot enter persistent ORL state, and cannot be reused by a later
   dispatch.
9. Topology/packing are immutable for a compiled rig. Any such change
   invalidates and rebuilds the rig before another dispatch.

## Work items

### 1. Runtime reflection and public binding API

- Add `Handle` to `OrlRuntimeParameterKind` and
  `ORL::exec::ParameterKind`.
- Carry canonical nominal type identity in parameter descriptions.
- Add an explicit `bind_handle(name, HandleValue)` API and graph-input binding
  storage.
- Validate missing, wrong-kind, wrong-tag, and invalid bindings with parameter
  names in diagnostics.

### 2. LLVM and wrapper ABI

- Map a handle to the documented two-lane LLVM representation.
- Generate argument unpacking for ordinary functions, exported host wrappers,
  and graph-lowered entry functions.
- Implement equality, inequality, copy, return, and validity.
- Include the handle ABI version in JIT/cache keys.

### 3. CPU invocation

- Extend ordered runtime arguments without reinterpreting handle memory as an
  integer array.
- Cover mixed signatures so buffer/int/float/handle ordering cannot diverge
  between reflection and invocation.
- Verify return and forwarding behavior through helper calls.

### 4. CUDA/PTX invocation

- Extend GPU kernel parameter metadata and packing.
- Assert generated parameter size/alignment and lane order.
- Add conditional CUDA execution parity tests; also inspect PTX/LLVM IR in
  environments without a device.

### 5. Graph execution

- Map `LogicalTypeKind::Handle` to `ParameterKind::Handle`.
- Lower graph handle inputs and exact handle edges without converting them to
  `int`.
- Continue rejecting component data access; plan 4 adds resolvers.

## Tests

Extend:

- `tests/codegen_tests/codegen_tests.cpp`
- `tests/exec_tests/exec_tests.cpp`
- `tests/graph_lowering_tests/graph_lowering_tests.cpp`
- `tests/gpu_tests/gpu_tests.cpp`

Required cases:

- LLVM type and exported-wrapper lane layout;
- CPU echo, copy, equality, inequality, invalidity, and helper forwarding;
- rejection of any attempt to persist or reuse a handle across dispatches;
- mixed runtime parameter ordering;
- missing/wrong nominal tag binding fails;
- graph input to graph output preserves both lanes;
- CUDA/PTX layout matches CPU and conditional device execution has parity;
- ordinary integer programs retain their existing ABI.

Suggested verification:

```sh
cmake --build build --target \
  orl_codegen_tests orl_exec_tests \
  orl_graph_lowering_tests orl_gpu_tests
./build/tests/codegen_tests/orl_codegen_tests
./build/tests/exec_tests/orl_exec_tests
./build/tests/graph_lowering_tests/orl_graph_lowering_tests
./build/tests/gpu_tests/orl_gpu_tests
```

GPU tests may report an intentional skip when no CUDA device/toolchain is
available; IR/PTX layout tests must still run where supported.

## Exit criteria

- An exact handle crosses every supported execution boundary without loss.
- CPU/GPU ABI details are documented and locked by tests.
- Wrong nominal tags fail at bind time.
- Compiled-rig invalidation prevents dispatch after a topology/packing
  revision.
- No rig component knowledge enters core ORL or generic graph execution.

## Not in scope

- Stable scene identity to slot resolution.
- Joint/locator storage.
- Input-node cutover.
- Named unions, `is`, effects, or solver migration.
