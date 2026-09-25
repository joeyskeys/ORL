# Extensible typed handles plan 1: core frontend and exact handle types

## Goal

Teach the ORL frontend that `handle` is an opaque value category and that
declarations such as `handle joint_handle;` create nominal types. This period
stops before runtime transport, rig storage access, unions, and `is`.

At the end of this period the compiler can parse, retain, and semantically
check exact handle types. Existing ORL programs behave unchanged. Programs
that try to execute handle parameters receive an explicit
"runtime handle ABI not enabled" diagnostic until plan 3.

## Depends on

- [`input_node_problem.md`](input_node_problem.md) is the semantic authority.
- No earlier typed-handle implementation plan.

## Owned submodule

ORL lexer, parser, AST, and semantic type analysis:

- `src/orlcomp/orl_lexer.h`
- `src/orlcomp/orl_lexer.cpp`
- `src/orlcomp/orl_ast.h`
- `src/orlcomp/orl_parser.h`
- `src/orlcomp/orl_parser.cpp`
- `src/orlcomp/orl_preprocessor.h`
- `src/orlcomp/orl_preprocessor.cpp`
- `src/orlcomp/orl_analysis.h`
- `src/orlcomp/orl_analysis.cpp`
- `src/orlcomp/orl_codegen.cpp` only for fail-closed diagnostics

The active frontend is handwritten. Do not implement this work in the legacy
`orllex.l`/`orlgram.y` skeleton unless the build is deliberately switched back
to that frontend.

## Design fixed in this period

1. `handle` is a core keyword and universal opaque type.
2. `handle name;` is a top-level nominal handle declaration.
3. Nominal identity is the declaration's canonical package-prefixed name,
   such as `orlrig::joint_handle`; equal source spelling or storage
   representation does not imply assignability.
4. Handle declarations follow declaration-before-use, like current structs.
5. Exact handle values support assignment, parameter passing, return, and
   equality in the type model. A source-level validity utility can be added
   later without introducing another keyword.
6. Handle values do not support arithmetic, indexing, direct member access,
   buffer element syntax, or implicit conversion to/from `int`.
7. Do not add `as`, pointer syntax, unions, `is`, data access, or
   `in/out/inout` keywords. Parameter effects will be inferred in plan 6.

## Work items

### 1. Lexical and AST representation

- Add `KwHandle`.
- Add `HandleDefinitionStatement` with a canonical name.
- Preserve declaration module/package provenance through textual `use`
  expansion, or obtain it from the package registry, so the canonical name is
  deterministic while source can use a short imported spelling.
- Preserve source locations for duplicate, unknown, and misplaced declaration
  diagnostics.
- Represent handle identity explicitly; do not infer it from a raw
  `type_name` string in later phases.

### 2. Parser symbol table

- Extend the current struct-only type tracking to distinguish built-ins,
  structs, and nominal handles.
- Parse top-level `handle joint_handle;`.
- Allow declared handle types and universal `handle` in scalar declarations,
  function parameters, and returns.
- Reject handle buffers and arrays initially. Persistent collections need a
  separate lifetime/ABI design.

### 3. Typed semantic model

- Introduce a reusable semantic type representation rather than spreading
  handle-name string checks through analysis and codegen.
- Resolve each expression's type.
- Enforce exact nominal assignment and call compatibility.
- Diagnose direct member access, indexing, arithmetic, and integer conversion
  on handles before LLVM code generation.
- Preserve handle declarations and function handle signatures in
  `AnalysisResult` for the graph importer in plan 2.
- Because `FunctionParameterSummary::logical_type` is currently an
  `orlgraph::LogicalType`, keep the new compiler semantic handle identity in a
  separate resolved-type field during this period. Leave graph logical mapping
  unsupported rather than reporting a handle as `Int64` or `Struct`; plan 2
  adds the graph representation and importer mapping.

### 4. Fail-closed backend boundary

- If handle-typed executable code reaches codegen before plan 3, emit one
  structured unsupported-ABI diagnostic.
- Never lower a handle silently as ordinary `i64`.

## Tests

Add focused cases to:

- `tests/lexer_tests/lexer_tests.cpp`
- `tests/parser_tests/parser_tests.cpp`
- `tests/analysis_tests/analysis_tests.cpp`
- `tests/syntax_tests/language_syntax_tests.cpp`
- `tests/codegen_tests/codegen_tests.cpp` for the fail-closed boundary

Required positive cases:

- universal and nominal declarations parse;
- declaration-before-use succeeds;
- exact handle assignment, parameter passing, equality, and return type-check;
- old source produces unchanged AST and analysis summaries.

Required negative cases:

- duplicate/unknown handle declarations;
- handle/struct name collision;
- `joint_handle` assigned to `locator_handle`;
- implicit `int <-> handle`;
- arithmetic, indexing, array/buffer declaration, and direct member access;
- executable handle signature before the runtime ABI is enabled.

Suggested verification:

```sh
cmake --build build --target \
  orl_lexer_tests orl_parser_tests orl_analysis_tests \
  orl_syntax_tests orl_codegen_tests
./build/tests/lexer_tests/orl_lexer_tests
./build/tests/parser_tests/orl_parser_tests
./build/tests/analysis_tests/orl_analysis_tests
./build/tests/syntax_tests/orl_syntax_tests
./build/tests/codegen_tests/orl_codegen_tests
```

Use the actual configured build directory if it is not `build`.

## Exit criteria

- All existing frontend tests remain green.
- Exact handle misuse fails in semantic analysis, not in LLVM assertions.
- No runtime or graph behavior changes.
- No handle is represented as an unbranded integer in public compiler data.

## Not in scope

- Graph logical types and serialization.
- Runtime/JIT/GPU ABI.
- Rig handle registration and component storage.
- Named unions, `is`, or graph specialization.
