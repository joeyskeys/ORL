# Extensible typed handles plan 7: unions, `is`, and static specialization

## Goal

Add controlled generic handle parameters without runtime handle-kind
dispatch. Named unions and universal `handle` describe accepted graph inputs.
The handle-only `is` expression flow-narrows source code, while graph
specialization resolves and folds every test before LLVM emission.

## Depends on

- [Plan 1](extensible_typed_handle_plan_1.md): exact frontend types.
- [Plan 2](extensible_typed_handle_plan_2.md): exact graph types.
- [Plan 3](extensible_typed_handle_plan_3.md): exact executable ABI.
- [Plan 4](extensible_typed_handle_plan_4.md): exact storage-backed struct
  views.
- [Plan 6](extensible_typed_handle_plan_6.md): typed effects that can be
  specialized with each branch.

## Owned submodule

- lexer/parser/AST files in `src/orlcomp`
- `src/orlcomp/orl_analysis.h`
- `src/orlcomp/orl_analysis.cpp`
- `src/orlcomp/orl_graph_import.cpp`
- `src/orlcomp/orl_graph_lowering.cpp`
- `src/orlcomp/orl_codegen.cpp`
- graph type/validation/reflection/serialization files in `src/orlgraph`
- compiler cache/specialization keys

## Design fixed in this period

1. The only new narrowing operator is `is`; there is no `as`.
2. A declaration such as
   `handle xform_handle = joint_handle | locator_handle;` defines a closed
   accepted leaf set.
3. Universal `handle` is an open accepted set.
4. Union/universal parameters are compile-time generic graph sockets, not
   runtime-varying sum values.
5. Every executable graph edge provides one provable exact nominal leaf type.
6. Inside `if (value is joint_handle)`, the original expression is
   flow-narrowed, so ordinary `Joint(value)` conversion syntax is valid.
7. Graph specialization folds `is` to a Boolean constant and removes dead
   branches before LLVM code generation.
8. `type_id` remains available for binding validation and diagnostics, but
   generated kernels do not branch on it.

## Work items

### 1. Union declarations and normalization

- Parse named union declarations and add `KwIs`.
- Resolve declarations to sorted, duplicate-free exact leaf sets.
- Reject unknown leaves, union cycles, duplicate declarations, and empty
  unions.
- Keep canonical serialization independent of declaration order.

### 2. Assignability

- Extend graph and semantic assignability with the subset rule:
  exact-to-union and exact-to-universal are valid; broad-to-narrow is not.
- Treat a union output whose nominal kind can vary at runtime as unsupported.
- Include accepted leaf sets in reflection and cache keys.

### 3. Flow-sensitive `is` analysis

- Add a dedicated AST expression; do not encode `is` as a normal function
  call or equality.
- Permit only handle expression on the left and exact nominal handle type on
  the right.
- Narrow the tested expression in the dominated true branch and subtract the
  leaf in the false/`else if` path where useful.
- Type-check each reachable branch under its narrowed environment.
- Diagnose conversion from a still-broad flow type.

### 4. Graph specialization

- Propagate the exact source leaf through graph inputs, forwarding nodes, and
  generic helper calls.
- Key compiled functions/kernels by generic function plus exact handle-type
  substitutions.
- Fold each `is` and prune dead branches in typed IR before LLVM lowering.
- Specialize effects together with code so dead-branch reads/writes do not
  pollute the evaluation plan.
- Fail graph compilation when exact provenance cannot be proven.

### 5. Backend proof

- Make codegen reject unresolved `is`; it must never lower it to a runtime
  `type_id` comparison.
- Inspect LLVM IR/PTX to ensure there is no kind comparison or switch.
- Permit runtime variation of component identity within one exact type.

## Tests

Extend:

- `tests/lexer_tests/lexer_tests.cpp`
- `tests/parser_tests/parser_tests.cpp`
- `tests/analysis_tests/analysis_tests.cpp`
- `tests/graph_ir_tests/graph_ir_tests.cpp`
- `tests/graph_lowering_tests/graph_lowering_tests.cpp`
- `tests/codegen_tests/codegen_tests.cpp`
- `tests/exec_tests/exec_tests.cpp`
- `tests/gpu_tests/gpu_tests.cpp`

Required cases:

- deterministic union normalization and serialization;
- subset assignability matrix for exact, union, and universal types;
- positive `is` flow narrowing with `Joint(value)`/`Locator(value)`;
- invalid `is` operands and insufficient narrowing diagnostics;
- same generic function specializes differently for joint and locator inputs;
- dead branch effects are absent from each specialized graph;
- generated LLVM/PTX contains no runtime kind switch/comparison;
- runtime-varying component identity within `joint_handle` remains valid;
- runtime-varying nominal kind or unresolved provenance is rejected;
- specialization/cache keys distinguish exact substitutions.

Suggested verification:

```sh
cmake --build build --target \
  orl_lexer_tests orl_parser_tests orl_analysis_tests \
  orl_graph_ir_tests orl_graph_lowering_tests \
  orl_codegen_tests orl_exec_tests orl_gpu_tests
./build/tests/lexer_tests/orl_lexer_tests
./build/tests/parser_tests/orl_parser_tests
./build/tests/analysis_tests/orl_analysis_tests
./build/tests/graph_ir_tests/orl_graph_ir_tests
./build/tests/graph_lowering_tests/orl_graph_lowering_tests
./build/tests/codegen_tests/orl_codegen_tests
./build/tests/exec_tests/orl_exec_tests
./build/tests/gpu_tests/orl_gpu_tests
```

## Exit criteria

- Generic handle source uses only `is` plus typed-view conversions and
  ordinary field syntax.
- Every executable `is` has been compile-time folded.
- No CPU/GPU kernel contains runtime handle-kind dispatch.
- Specialization preserves correct effects and cache isolation.

## Not in scope

- Runtime sum types or dynamically changing nominal handle kinds.
- Persistent collections of heterogeneous handles.
- Remaining rig-node migration and compatibility deletion.
