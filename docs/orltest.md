# `src/orltest`: legacy/orphaned test harness

## Status

`src/orltest` is not an active ORL module or an active test target in the
current repository. It is a retained legacy GoogleTest harness for an old
Wavefront OBJ loader.

The current root build:

- does not add `src/orltest`;
- does not include the old GoogleTest submodule;
- does not define the old `loader` target;
- does not contain the old hand-written mesh/OBJ parser sources;
- uses Catch2-based executables under `tests/` instead.

The only root reference is the stale commented line:

```cmake
#add_subdirectory(orltest)
```

That path is also wrong for the retained directory: the files are under
`src/orltest`, not root-level `orltest`.

The accurate architectural classification is:

```text
src/orltest
  historical test harness
  not compiled
  not part of ORL compiler/runtime/viewer execution
```

## File inventory

| File | Role |
| --- | --- |
| `CMakeLists.txt` | Inactive legacy GoogleTest target definition |
| `vertex_parse.cpp.in` | Configure-time test source template |

There are only two tracked files in the current directory. The generated
`vertex_parse.cpp` is not checked in.

## Intended historical pipeline

The old harness was designed to look like:

```text
src/orltest/CMakeLists.txt
  -> configure vertex_parse.cpp.in
  -> generated vertex_parse.cpp
  -> old mesh/OBJ loader
  -> GoogleTest executable
  -> CTest registration
```

It has no relationship to the active ORL source pipeline:

```text
ORL source
  -> orlcomp
  -> orlexec
  -> orlrig
  -> viewer
```

It also does not exercise the current `orlgraph` graph IR, typed handles,
LLVM codegen, CUDA backend, rig runners, or scene graph runtime.

## CMake implementation

The retained `CMakeLists.txt` attempts to:

1. add `${CMAKE_SOURCE_DIR}/thirdparty/googletest` into the build;
2. add old viewer include paths;
3. configure `vertex_parse.cpp.in` into the binary directory;
4. define an `add_gtest` macro;
5. create an `obj_parse_tests` executable;
6. link:

```text
gtest
gmock
gtest_main
loader
```

7. register `obj_parse_tests` with `add_test`.

The file contains commented fragments for an earlier `mesh` target and
direct `obj_parse_tests` construction. The active macro call is:

```cmake
add_gtest(obj_parse_tests ${CMAKE_CURRENT_BINARY_DIR}/vertex_parse.cpp)
```

This CMake file is not included by the root project, so none of those
commands execute today.

## Test template

`vertex_parse.cpp.in` includes:

```cpp
#include "gtest/gtest.h"
#include "mesh.h"
#include "objgram.hpp"
```

It configures a test source path containing:

```text
@CMAKE_CURRENT_SOURCE_DIR@/sphere.obj
```

### Vertex test

`ObjParseTests.VertexParse`:

1. creates an old `Mesh`;
2. opens `sphere.obj`;
3. reads the file into a string if the stream is good;
4. calls `obj_parse_buffer(obj_buf, &m)`;
5. asserts `m.getVertexAt(0).x == 0.f`.

### Index test

`ObjParseTests.IndexParse` repeats the file read and parser call, then asserts:

```text
m.getIndexAt(0).x == 8
```

### Test entry point

The template defines its own `main`:

```cpp
::testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
```

At the same time, the CMake macro links `gtest_main`. If this target were
restored without cleanup, the duplicate-main behavior would need to be
resolved.

## Missing dependencies

The current repository does not contain the prerequisites this harness names:

| Expected dependency | Current state |
| --- | --- |
| `thirdparty/googletest` | Not present; `.gitmodules` declares only `thirdparty/vkkk` |
| `loader` target | Not defined by current CMake |
| `mesh.h` / `mesh.cpp` | Removed from the current viewer implementation |
| `loaders/objgram.y` | Removed |
| `loaders/objlex.l` | Removed |
| `sphere.obj` | Not present |
| old viewer include layout | Replaced by vkkk scene/mesh integration |

`src/cmake/flexbison.cmake` remains in the repository, but the root
`include(flexbison)` is commented out and current viewer targets do not invoke
the old OBJ grammar.

`.gitignore` ignores `*.obj`, so even restoring a fixture under the old
directory would require a deliberate fixture policy.

## Failure behavior if resurrected unchanged

Even if the root path and missing targets were temporarily restored, the
template has weak failure behavior:

- failure to open `sphere.obj` leaves `obj_buf` empty;
- the parser call is skipped;
- the test still indexes `m.getVertexAt(0)` or `m.getIndexAt(0)`;
- an absent/empty fixture can therefore cause invalid access or an
  uninformative failure instead of a fixture diagnostic.

The generated source also depends on an old `Mesh` API and old parser symbols,
so it cannot be treated as a smoke test for the current viewer loader.

## Historical role

The directory predates the current ORL compiler/runtime architecture. It
originated as a GoogleTest test around a hand-written OBJ parser and later
moved under `src/`. The old integration was commented out as the viewer moved
to vkkk-based scene and mesh loading.

Current model loading is handled by the viewer operation path, especially
`src/orlviewer/ops/load_model_op.cpp`, which delegates to vkkk scene/mesh
facilities. The current tests therefore validate scene/mesh integration
through vkkk and rig/runtime data rather than this old parser.

## Difference from current tests

The active test architecture is under `tests/`:

- Catch2, not GoogleTest;
- one CMake subdirectory per subsystem;
- lexer/parser/syntax tests;
- semantic analysis and graph import;
- LLVM/codegen/JIT;
- CUDA/PTX;
- graph IR/lowering;
- executor and rig runtime;
- skinning;
- scene graph and vkkk integration.

`src/orltest` is not included in `tests/CMakeLists.txt` and does not provide
coverage for any of those systems.

## What “module analysis” means here

For active modules, a module document can describe APIs, ownership, runtime
flows, and implementation invariants. For `orltest`, the important result is
the negative one:

- it has no current public API;
- it has no current production call sites;
- it has no active build target;
- it has no current runtime or data ownership;
- it is not a second ORL test suite;
- it cannot be restored by merely uncommenting one CMake line.

## Reuse or retirement options

### Retire and preserve as historical context

This is the lowest-risk interpretation:

- keep the source directory for history;
- leave it excluded from the build;
- do not list it as an active test;
- direct new contributors to `tests/` and current vkkk loading.

### Migrate the intent

If OBJ-loader coverage is still needed, write a new Catch2 test against the
current production loader:

1. identify the vkkk scene/mesh loading API used by
   `load_model_op.cpp`;
2. add a managed fixture under the current resource/test-fixture policy;
3. assert explicit file-open and parse errors;
4. test the resulting vkkk mesh/scene data;
5. add the test under `tests/` with current target dependencies.

This would test current behavior rather than reviving the removed `Mesh` and
Flex/Bison loader.

### Reconstruct the old harness

This would require a deliberate legacy restoration:

- restore/vendor GoogleTest;
- restore the `loader` target;
- restore old mesh and OBJ grammar sources;
- restore a non-ignored fixture;
- fix the root path to `src/orltest`;
- remove either the explicit `main` or `gtest_main`;
- add robust fixture-open/parser assertions;
- decide whether it belongs in current CI.

That work would create a second, unrelated loader stack and should not be
presented as an ORL module without an explicit project decision.

## Recommended source reading order

1. `src/orltest/CMakeLists.txt` to see the intended target graph.
2. `src/orltest/vertex_parse.cpp.in` to see the two historical assertions.
3. Root `CMakeLists.txt` to verify the target is not included.
4. `.gitmodules` and `.gitignore` to verify missing dependencies/fixture policy.
5. `src/orlviewer/ops/load_model_op.cpp` for the current replacement path.
6. `tests/` CMake files and test sources for the active test architecture.
