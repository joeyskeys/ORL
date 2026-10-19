# ORL statically selected thin graphics adapter for vkkk and mt

## Status and scope

Status: implementation in progress. The static contract, configure-time
selection, and first adapter targets are now being introduced. The legacy
viewer feature graph is still vkkk/Vulkan-specific, so this document
continues to distinguish the completed foundation from the remaining viewer
migration work.

The current implementation slice provides:

- `orl_rhi_contracts`, a header-only static adapter concept with value
  contracts, opaque mesh tokens, frame state, capabilities, and structured
  status results;
- `ORL_RHI_BACKEND=AUTO|VKKK|MT`, resolved by CMake into one generated
  configuration header and one `SelectedAdapter` alias;
- an `orl_rhi_vkkk` adapter that owns a vkkk context, fixed-color pipeline,
  indexed mesh resources, frame lifecycle, clear pass, resize, and draw
  submission;
- an `orl_rhi_mt` adapter that owns an offscreen Metal surface, command
  buffers, clear pass, and host-visible indexed mesh buffers;
- a backend-independent compile/conformance probe.

The mt slice deliberately reports indexed draw submission as unsupported until
its logical MSL pipeline registration is added. It still validates the
device/surface/resource ownership boundary; it does not claim mesh rendering
parity yet. The existing interactive viewer remains enabled only for the VKKK
configuration while its native feature graph is migrated to the selected
adapter in later phases.

The goal is to let `orl_viewer` use one small set of ORL-owned graphics calls
while keeping graphics-resource ownership inside the existing libraries:

```text
orl_viewer<SelectedAdapter>
    |
    v
ORL thin RHI/adapter contract
    |------------------------|
    v                        v
vkkk adapter                mt adapter
    |                        |
    v                        v
vkkk resources/scene       mt resources/scene
    |                        |
  Vulkan                   Metal
```

`SelectedAdapter` is resolved while configuring and compiling ORL. The final
viewer contains one concrete adapter, not a runtime choice between vkkk and mt.

The first supported configurations should be:

- non-macOS: vkkk/Vulkan;
- macOS: mt/Metal;
- an explicit backend override for development and CI.

The RHI is an ORL-facing integration boundary, not another graphics library.
It standardizes the calls that the viewer needs and translates them to vkkk or
mt. It does not recreate their resource managers, scene graphs, shader
systems, pipeline caches, command encoders, or viewport implementations.

The word "backend" is relative to ORL: vkkk and mt are backend integrations
for the viewer even though they are themselves wrappers over Vulkan and Metal.
The adapter must preserve useful native capabilities instead of reducing both
libraries to an accidental lowest common denominator.

### Compile-time backend policy

Backend selection is a build property, not a runtime setting:

1. CMake resolves `ORL_RHI_BACKEND=AUTO` to `VKKK` or `MT` from the target
   platform.
2. CMake generates one selected-backend configuration header or target alias.
3. `orl_viewer` is compiled against the concrete selected adapter through a
   type alias or a class/template parameter.
4. Only the selected adapter sources, native include paths, libraries, and
   shader assets enter the build graph.

The common RHI contract must use static dispatch. It should use identical
concrete method surfaces, concepts, traits, or compile-time aliases rather
than a virtual `IRhiBackend` base. It must not use a runtime backend enum,
backend registry, plugin loader, `std::variant<VkkkAdapter, MtAdapter>`, or a
runtime factory that links both implementations.

Static selection removes the unselected backend and runtime dispatcher from
the final binary. It does not remove the code required by the selected
adapter or by the selected vkkk/mt library. Runtime capability checks are
still allowed for features that depend on the actual GPU or OS; those checks
choose a path inside the already-selected backend and never switch to the
other graphics library.

## 1. Current boundary and constraints

The viewer is currently coupled to vkkk at almost every graphics boundary:

- `src/orlviewer/main.cpp` constructs `vkkk::Context`, `vkkk::Scene`,
  `vkkk::Camera`, `vkkk::WindowBackend`, and the templated
  `vkkk::vp::Viewport`.
- Viewer features inherit from `vkkk::vp::ViewportFeature` and record directly
  into `vk::raii::CommandBuffer` objects.
- Scene, mesh, line, point, pipeline, shader, and buffer types are vkkk
  types. Several feature headers also expose Vulkan extent and pipeline types.
- `SceneGraphContext`, `SceneInputCatalog`, `ComponentManager`, selection, and
  operations expose vkkk scene or input types even when their main work is
  renderer-independent.
- `src/orlviewer/CMakeLists.txt` links vkkk and adds its source directory to
  the viewer include path. The root build always adds the vkkk submodule when
  the viewer is enabled.
- Existing viewer shaders are GLSL/SPIR-V-oriented. They cannot be assumed to
  be directly usable by Metal.
- The current scene/runtime path can pass CUDA device allocations into Vulkan
  rendering. Metal support must not promise CUDA-to-Metal external memory
  interop without a separately verified producer, format, synchronization, and
  lifetime contract.

The `mt` project is not a source-compatible Vulkan port. Its public C++ API is
generated from a Swift Metal core and follows the native ownership graph:

```text
Device -> CommandQueue -> CommandBuffer -> Render/Compute/BlitEncoder
```

It provides optional renderer, scene, viewport, picking, AppKit/CAMetalLayer,
and MTKView layers. It intentionally does not require a Vulkan-like global
`Context`, and it uses MSL/metallib rather than SPIR-V as its native shader
path.

## 2. Goals and non-goals

### Goals

1. Make viewer source use ORL-owned calls instead of vkkk or Vulkan types.
2. Keep vkkk responsible for its graphics resources, scene, shader/pipeline
   objects, caches, and native command recording.
3. Keep mt responsible for its Metal resources, scene, shader/pipeline
   objects, caches, and native command recording.
4. Add a Metal adapter using the installed `mt` C++ package on macOS.
5. Make backend capabilities and fallback decisions explicit and observable.
6. Separate renderer-independent ORL data and graph execution from backend
   resource ownership.
7. Avoid duplicating scene management, model loading, resource lifetime, and
   graphics concepts merely to make the two libraries look identical.
8. Enable adapter-level and offscreen tests without requiring a window for
   every viewer behavior.
9. Make backend dispatch compile-time and ensure the unselected backend is not
   compiled or linked into the viewer.

### Non-goals for the first implementation

- Do not make `mt` source-compatible with vkkk or reproduce either library's
  class hierarchy.
- Do not create an ORL-owned `Buffer`, `Texture`, `Pipeline`, `Scene`, or
  `Viewport` implementation that duplicates vkkk/mt ownership.
- Do not expose a second generic command-encoder or resource-lifetime system
  in the thin adapter.
- Do not add runtime backend selection, virtual backend dispatch, or a
  dual-backend binary. Changing backend requires a separate configure/build.
- Do not rewrite the ORL compiler, graph IR, component store, or rigging
  algorithms as part of the graphics migration.
- Do not port CUDA execution to Metal under the name of RHI work.
- Do not assume one shader source language can replace both GLSL/SPIR-V and
  MSL without validating layout, precision, coordinate, and pipeline behavior.
- Do not make Qt, GLFW, AppKit, MTKView, ImGui, or a file-dialog toolkit a
  required dependency of the RHI core.
- Do not hide all Metal ownership behind a synthetic global context merely to
  match the current vkkk startup code.

## 3. Proposed thin adapter structure

The important design rule is ownership:

- vkkk owns its Vulkan context, queues, command buffers, resources, scene,
  shader modules, pipelines, caches, and native viewport services;
- mt owns its Metal device/queue/command buffers, resources, scene, shader
  libraries, pipelines, caches, and native viewport services;
- ORL owns component identity, graph execution, authoring state, selection,
  camera/navigation state, and viewer workflow;
- the RHI only translates the viewer's required calls between those owners.

The dispatch boundary is compile-time. `orl_viewer_core` should be written
against `SelectedAdapter` or be instantiated with the selected adapter type.
The adapter contract is a common static method surface; it is not a base class
whose methods are called through a vtable.

Use separate targets/namespaces with one-way dependencies:

```text
orl_viewer_core
    |
    +-- ORL::Rhi thin contracts
    +-- ORL component/graph/authoring code
    |
    +-- selected host and adapter

orl_rhi_vkkk  -> vkkk -> Vulkan
orl_rhi_mt    -> mt   -> Metal/Swift runtime
```

The exact target names can change, but these rules must remain:

- RHI headers contain only small value types, opaque tokens, call interfaces,
  capability records, and structured results. They do not contain a second
  graphics-resource class hierarchy.
- RHI calls are statically dispatched through the selected concrete adapter.
  A common header may select one implementation with a generated configuration
  header, but it must not include both implementations in the same target.
- Adapter implementations may include vkkk, Vulkan, mt, Metal, Swift
  generated headers, and host libraries internally.
- Viewer domain code may include RHI headers and ORL domain headers, but not
  native backend types.
- `orlexec`/`orlrig` remain the owners of language execution and rig data.
- Native scene/resource managers stay behind the selected adapter.

### 3.1 Thin call contract

Define only the operations that the current viewer actually needs. The first
contract should cover:

- initialize/shutdown and capability discovery;
- attach a host surface or create an offscreen target;
- begin/end a frame, resize, present, wait idle, and report errors;
- submit the viewer's scene, picking, compute, and overlay work;
- update logical scene/display data through adapter-owned resources;
- request and poll picking/readback results;
- expose backend capability flags and explicit unsupported-operation errors.

These are calls and opaque tokens, not ORL-owned replacements for vkkk or mt
objects. A token may identify an adapter-owned scene object, mesh, material,
pipeline, or readback request, but the adapter decides how it is allocated,
stored, updated, synchronized, and destroyed.

The call surface should be usable by a class template or concrete selected
adapter without virtual dispatch. Backend-specific operations may be exposed
through statically selected extension traits or `if constexpr` paths. Runtime
capability values may still reject an operation or select a fallback inside
the selected adapter.

Do not initially define generic ORL `Buffer`, `Texture`, `Pipeline`,
`CommandEncoder`, `Scene`, or `Viewport` classes. If a viewer operation needs
one of those concepts, pass a small description or opaque token to the
adapter and let the selected library perform the native work.

The frame contract should cover the behavior currently hidden in
`Context::begin_frame()`/`end_frame()` and the corresponding mt flow:

1. acquire a drawable or offscreen target;
2. let ORL update graph and authoring state;
3. ask the adapter to synchronize changed logical display data;
4. submit scene, picking, and overlay work;
5. present/commit when a drawable exists;
6. report resize, drawable loss, completion, and device errors.

The adapter may use a vkkk context or an mt frame internally. The common
contract must not imply that Metal has a Vulkan swapchain or that either
library's native frame object is owned by ORL.

### 3.2 Viewer-owned logical scene and assets

There is no common RHI `Scene` in the first design. `ComponentManager`,
`SceneGraphContext`, selection, and ORL graph state remain the logical source
of truth. The selected adapter maintains whatever native scene/resource
representation it needs as a graphics cache.

Model loading is also not an RHI responsibility. Keep a viewer/application
asset boundary that can:

1. obtain a path or asset value from a host service;
2. convert it to ORL-neutral geometry, normals, indices, materials, and frame
   metadata where necessary;
3. ask the selected adapter to create or update its native scene resources;
4. register stable ORL scene-object identities for selection and picking.

The vkkk `DrawableMgr` path and mt asset/scene facilities may be used inside
their adapters. They must not define the common RHI API or force ORL to
duplicate their loaders.

### 3.3 Viewer feature calls

ORL should own the semantic viewer feature layer because the features encode
ORL selection, components, graph updates, display modes, and workflow. Replace
the vkkk-specific `ViewportFeature` inheritance with a small ORL feature
lifecycle only where needed:

```text
Update       ORL graph/component work; not owned by RHI
Scene        adapter scene submission
Picking      adapter picking request/readback
Overlay      adapter overlay submission
```

The feature layer may use an opaque frame/pass token or a short-lived adapter
call context. It must not record into `vk::raii::CommandBuffer` or manipulate
mt encoders directly.

The existing grid, mesh, joint, controller, locator, transform-guide, HUD,
CSR, and picking features should migrate one at a time. Their semantic
behavior remains in ORL; native resource creation and recording remain in
the selected library.

### 3.4 Host and input boundary

Input and window integration are application/host concerns, not graphics
resource concerns. Define backend-neutral ORL input values for keys, mouse
buttons, actions, coordinates, modifiers, scroll, resize, close, and focus.
`ControlMap` and operations should consume these values instead of
`vkkk::Key`, `vkkk::InputAction`, `vkkk::MouseButton`, and
`vkkk::WindowBackend`.

Host adapters translate:

- vkkk's current GLFW/Qt callbacks for the Vulkan path;
- mt's AppKit/CAMetalLayer or MTKView events for the macOS path;
- a future Qt-on-Metal surface, if required, without changing RHI core.

The file-dialog service is likewise a host/application service and is not
selected by the graphics adapter.

### 3.5 Shader and capability policy

The common layer should use stable logical shader/feature IDs and small
semantic parameter descriptions, not common shader-module or pipeline
classes:

- vkkk resolves an ID through its existing GLSL/SPIR-V or shader-pack system;
- mt resolves the same logical ID through MSL/metallib and Metal binding
  metadata;
- each library owns compilation, reflection, caching, and pipeline lifetime.

Vulkan- or Metal-specific features remain adapter extensions. Argument
buffers, mesh/object shaders, tile passes, memoryless attachments, indirect
command buffers, ray tracing, GPU counters, and CUDA/Vulkan interop must be
capability-gated and must not enlarge the common contract unless the viewer
has a demonstrated need.

## 4. Backend adapter responsibilities

`VkkkAdapter` and `MtAdapter` are separate concrete implementations of the
same static contract. They are not subclasses of a runtime-polymorphic RHI
interface. The selected target provides the only adapter type visible to the
viewer build.

### 4.1 vkkk/Vulkan backend

The vkkk adapter is an integration layer over the existing vkkk API, not a
second Vulkan renderer. It owns or delegates all vkkk objects:

- `vkkk::Context`, queues, command buffers, and frame ownership;
- vkkk scene/drawable/mesh/line/point resources;
- vkkk shader modules, shader packs, pipelines, binding calls, and caches;
- vkkk window backends and input callbacks;
- vkkk model-loading and viewport services when their behavior matches the
  ORL operation.

The adapter translates ORL logical scene updates, feature calls, shader IDs,
and picking requests into those existing services. Any native escape hatch
must stay inside the vkkk adapter target; it must not appear in ORL viewer
headers or in the common RHI contract.

Preserve current behavior while migrating:

- Vulkan 1.3 startup and shader/pipeline caches;
- GLFW and optional Qt host behavior;
- camera handedness and front-face selection;
- CUDA/Vulkan device-buffer paths where the current backend supports them;
- existing model loading, scene rendering, picking, and overlay behavior.

### 4.2 mt/Metal backend

The mt adapter should use the public installed package and its generated C++
interoperability headers. It owns or delegates all mt objects:

- `mt::Device`, `mt::CommandQueue`, command buffers, encoders, and surfaces;
- mt scene, renderer, viewport, picking, shader, pipeline, cache, and resource
  services where they match the ORL operation;
- AppKit/CAMetalLayer or MTKView host integration;
- MSL/metallib loading, reflection, binding, and pipeline lifetime;
- capability records and Metal-specific fallback decisions.

It translates the same ORL logical calls used by the vkkk adapter, but it does
not need to imitate vkkk's context or class hierarchy. The adapter must
preserve Metal-native ownership and must not create a fake `mt::Context` solely
to mirror vkkk.

Required initial macOS capabilities:

- windowed and offscreen frame creation;
- resize and drawable-unavailable recovery;
- indexed mesh rendering;
- depth testing and blending;
- storage/uniform updates;
- scene-object and rig display;
- a picking/readback path;
- clear diagnostics when an optional native path is unavailable.

Metal-specific extensions such as argument buffers, mesh/object shaders,
tile passes, memoryless attachments, indirect command buffers, ray tracing,
and GPU counters should be exposed through capability-aware extension
interfaces. They should not block the baseline viewer path.

### 4.3 Compute and external-resource boundary

The thin RHI must not turn `exec::DeviceBufferView` into a common graphics
resource type. Instead, define an adapter capability and handoff boundary for:

- host-backed upload;
- a backend-owned GPU view;
- a borrowed external resource;
- synchronization/completion requirements;
- copy versus zero-copy behavior.

The vkkk adapter may implement the existing CUDA/Vulkan path using vkkk's
resource import facilities. The mt adapter should initially use host/shared
Metal buffers or an explicit copy path. IOSurface/shared-event integration
may be added later only when a concrete producer and synchronization contract
exists.

For macOS, the first correctness milestone should allow CPU evaluation and
host-to-Metal upload even if CUDA evaluation is unavailable. A device-only
CUDA-to-Metal route is not an acceptance criterion for the first RHI slice.

## 5. Viewer migration inventory

The migration should be organized by boundary rather than by file count.

### Startup and build

- Move device/surface/frame selection out of `main.cpp` into one selected
  adapter factory. The factory returns an ORL call interface; it does not
  expose a second device/resource object model.
- Replace direct backend selection and vkkk compile definitions with one
  `ORL_RHI_BACKEND` configuration.
- Keep `main.cpp` responsible for viewer composition, graph setup, operations,
  and the frame loop; it should only see ORL adapter/RHI types.

### Camera and coordinate policy

- Move camera state and navigation math to ORL-owned data.
- Define one explicit world-space convention, clip-space conversion policy,
  depth range, front-face rule, and viewport resize rule.
- Let each adapter convert the ORL camera state to its native scene/resource
  representation.
- Add cross-backend tests for handedness, winding, camera orbit, picking
  depth, and screen-to-world rays.

### Scene and components

- Do not replace vkkk or mt `Scene`, `SceneObject`, `Mesh`, `Lines`, `Points`,
  or curve systems with ORL-owned equivalents.
- Replace native scene references that leak into viewer/domain headers with
  stable ORL display tokens or a narrow adapter display service.
- Keep component IDs, stable names, packed joint IDs, and selection identity
  independent of native resource names.
- Keep `SceneInputCatalog` and ORL packed data renderer-independent. Ask the
  selected adapter to upload or bind the data through an explicit copy/import
  capability rather than exposing a generic RHI buffer.

### Operations and input

- Replace vkkk input enums and window pointers in operations with ORL input and
  host-service interfaces.
- Keep model loading, file dialogs, selection, transforms, and graph editing
  independent of the graphics API.
- Make `LoadModelOp` consume a neutral asset operation. The selected adapter
  may delegate loading/upload to `vkkk::DrawableMgr` or mt's asset facilities;
  neither loader becomes part of the RHI contract.

### Viewport features

Keep semantic feature behavior in ORL and migrate the native calls in this
order:

1. camera/clear/frame lifecycle and a minimal scene mesh;
2. Phong, wireframe, and x-ray mesh display;
3. joint, controller, locator, line, point, and frame-axis display;
4. grid, transform guides, and runtime HUD;
5. mesh/joint/rig picking and asynchronous readback;
6. mesh CSR compute passes and auto-weight GPU data preparation;
7. backend-specific optimized paths.

Each step should add one ORL adapter call, preserve the old vkkk behavior, and
then implement the equivalent mt translation. Do not introduce a second
ORL-owned scene or pipeline implementation to make a feature fit.

### Shaders

Port shader families by semantic feature, not by filename:

- scene Phong and wireframe;
- x-ray;
- joint/controller/locator/curve/point;
- grid, axes, transform guide, and HUD;
- mesh and rig picking;
- CSR compute.

For each family, define logical shader IDs and the minimum parameter metadata
needed by the viewer. Keep separate backend artifacts and let each adapter
load them through the native vkkk or mt shader/pipeline system. The viewer
must not construct a common pipeline object or load a backend-specific module.

## 6. Build and dependency plan

Add a CMake backend selection with behavior equivalent to:

```text
ORL_RHI_BACKEND = AUTO | VKKK | MT
```

Recommended selection rules:

- `AUTO` chooses `MT` on Apple platforms and `VKKK` elsewhere.
- CMake resolves `AUTO` to one canonical value during configuration; there is
  no `AUTO` branch in the final executable.
- An explicit unavailable backend fails during configuration with an
  actionable diagnostic.
- `MT` is accepted only on supported macOS/toolchain configurations.
- `VKKK` is not added to the build when `MT` is selected.
- `MT` is not required by non-macOS builds.
- Switching between vkkk and mt requires reconfiguring the build directory.
  The running viewer cannot switch backends.

Use an installed mt package as the normal dependency:

```text
find_package(mt CONFIG REQUIRED)
```

For local development, provide a cache variable such as
`ORL_MT_SOURCE_DIR` or a documented `CMAKE_PREFIX_PATH` setup. Do not commit
the developer-specific `/Users/joey/Desktop/workspace/self/mt` path into ORL.
If source-tree integration is needed later, use a reproducible dependency
policy rather than an absolute path.

The build should eventually produce:

- `orl_rhi_contracts` as a small header-only/value-contract target;
- one selected adapter target, either `orl_rhi_vkkk` over vkkk or
  `orl_rhi_mt` over mt;
- `orl_viewer_core` for viewer/domain code;
- `orl_viewer` as the platform executable and host composition target.

`orl_rhi_contracts` must not allocate graphics resources, compile shaders,
implement a scene manager, or own a command encoder. Keep resource/shader
installation and compile definitions backend-specific. Only the selected
adapter should receive its native include paths and link libraries.

The CMake target graph must enforce static selection:

1. Resolve the backend and generate `orlviewer/rhi/rhi_config.hpp` with the selected
   backend tag and selected-adapter alias.
2. Add only the selected adapter's source files with `target_sources`.
3. Link only the selected native target and its required runtime libraries.
4. Exclude the other adapter's source files, include directories, generated
   headers, shader assets, and link dependencies.
5. Expose one `orl_rhi_selected` target alias so viewer CMake does not need
   runtime or platform branching.

The configuration header may contain a compile-time selection such as:

```cpp
namespace ORL::Rhi {
using SelectedAdapter = backend::MtAdapter; // generated for this build
}
```

The vkkk build generates the corresponding vkkk alias instead. The actual
adapter type must be included only by the selected target. Do not implement
the alias as a conditional owning both backend types.

`orl_rhi_contracts` should remain header-only and contain no runtime service
object. When a call can be expressed as a small inline/static operation, the
compiler may eliminate the abstraction completely. The selected adapter still
contributes the translation code required to call vkkk or mt; the goal is to
avoid an additional generic dispatcher and all unselected backend code, not to
pretend that the selected integration has zero implementation cost.

## 7. Implementation phases

### Phase 0: contract inventory and proof of dependency shape

1. Freeze the current vkkk behavior and enumerate every vkkk/Vulkan symbol
   crossing `src/orlviewer` headers.
2. Inventory actual viewer calls and classify each as:
   - common ORL adapter call;
   - vkkk/mt-native implementation detail;
   - ORL domain or host behavior that should not enter RHI.
3. Decide the small result/error/capability and coordinate contracts. Avoid
   designing generic resource classes before a call site requires one.
4. Define the static adapter contract and generated selected-adapter alias.
   Verify that viewer code can compile against either concrete type without a
   virtual backend base or a dual-backend variant.
5. Build a minimal C++ consumer against the installed mt package on macOS.
6. Confirm a vkkk adapter and an mt adapter can both provide a frame and
   display a test object using their native resource/scene APIs.
7. Record unresolved host, shader, and compute interop decisions in this
   document rather than hiding them in backend adapters.

**Exit condition:** the proposed thin contract is derived from viewer calls,
does not duplicate a graphics resource model, and both dependencies are
buildable through explicit configuration with exactly one selected adapter.

### Phase 1: thin contract and vkkk adapter

1. Add the small ORL call contract, opaque adapter tokens, errors,
   capabilities, frame state, and backend-neutral input values.
2. Add the generated selected-adapter alias and instantiate viewer code with
   the concrete vkkk adapter; do not add a virtual backend interface.
3. Wrap the current vkkk context/window/frame lifecycle without moving vkkk
   resource ownership into ORL.
4. Route a clear pass and one mesh feature through the adapter.
5. Add a mock/offscreen contract test only for call ordering/results where
   practical; keep resource behavior tested by vkkk.
6. Keep all existing vkkk behavior working before broad feature migration.

**Exit condition:** the viewer can start and render the initial scene mesh via
ORL calls while vkkk remains the owner of the actual scene and GPU resources.

### Phase 2: remove native types from viewer code

1. Refactor camera, component display tokens, model-loading calls, and input.
2. Replace direct `vk::` recording and vkkk shader/pipeline calls in each
   feature with thin adapter calls or opaque pass contexts.
3. Replace the templated vkkk viewport dependency in ORL feature headers with
   the ORL feature lifecycle.
4. Split viewer/domain sources into a reusable library target.
5. Restrict all vkkk/Vulkan includes to `orl_rhi_vkkk` and its private
   implementation files.

**Exit condition:** a source audit finds no vkkk/Vulkan dependency in
viewer-facing RHI or domain headers, the vkkk viewer has visual/interaction
parity for the migrated features, and no duplicate ORL resource manager was
introduced.

### Phase 3: minimal mt backend on macOS

1. Link the mt package and generated C++ header through a dedicated backend
   adapter target.
2. Configure a separate build with `SelectedAdapter=MtAdapter`; do not link
   the vkkk adapter or its Vulkan dependencies.
3. Translate the thin lifecycle and scene-display calls to mt's
   `Device`/`CommandQueue`/surface/scene/resource APIs.
4. Render a clear, a test mesh, and a depth-tested scene object using mt-owned
   resources.
5. Add resize, drawable-loss, completion/error, and adapter-lifetime tests;
   rely on mt for native resource-lifetime behavior.
6. Establish the macOS host/input policy before porting the full editor UI.

**Exit condition:** a macOS build opens a Metal-backed viewport and renders a
scene through ORL calls and mt-owned resources without any vkkk/Vulkan
dependency.

### Phase 4: feature and shader parity

1. Add mt-native shader artifacts and register them under the logical shader
   IDs used by the adapter.
2. Migrate joint/controller/locator/line/point/grid/axis/overlay feature calls.
3. Implement mesh and rig picking with stable ORL IDs and mt-owned readback.
4. Port or replace CSR/auto-weight GPU calls according to mt capabilities.
5. Add display-mode, selection, transform, and camera behavior checks on both
   adapters.

**Exit condition:** the same viewer workflows work on Vulkan and Metal, with
documented capability-dependent fallbacks.

### Phase 5: execution/resource interop and optional paths

1. Define the neutral host-to-adapter handoff for graphics data.
2. Preserve the vkkk CUDA/Vulkan fast path behind the vkkk adapter.
3. Make CPU or host-upload paths correct on macOS.
4. Evaluate Metal compute or external-resource integration separately from
   the initial graphics migration.
5. Add optional argument-buffer, mesh-shader, indirect, tile, and profiling
   paths only where a viewer use case benefits from them.

**Exit condition:** no backend silently consumes an incompatible external
   allocation, and every fallback is reported with an actionable reason.

### Phase 6: verification and cleanup

1. Add thin-contract tests for lifecycle, call ordering, errors, capabilities,
   stable identity, and readback results.
2. Run vkkk tests and viewer regression tests on existing platforms.
3. Run mt Swift/C++ package and Metal-device tests on macOS.
4. Add a capability matrix covering device, shader, resource, picking, and
   host integration paths.
5. Build each backend in a separate configuration and verify that the
   unselected adapter target and native dependency are absent.
6. Remove compatibility shims after all viewer call sites use the thin
   adapter contract.
7. Update `docs/orlviewer.md` and build documentation to describe the final
   ownership and backend model.

## 8. Verification requirements

### RHI/adapter contract tests

Test every adapter, and a mock adapter where useful, for:

- initialization and capability discovery;
- frame begin/end, offscreen operation, resize, and drawable loss;
- logical display-token creation, update, invalidation, and destruction calls;
- shader-ID and feature-call translation;
- picking request/readback completion and stable ORL identity;
- structured error propagation and unsupported-operation reporting;
- coordinate, depth, blending, culling, and call-order behavior.

Test actual buffer, texture, pipeline, scene, cache, synchronization, and
resource-lifetime behavior through the vkkk and mt native test suites and
adapter integration tests. Do not recreate those resource tests in an ORL
resource implementation that does not own the resources.

### Static dispatch and binary tests

Verify:

- `SelectedAdapter` resolves to exactly one concrete type for each configure;
- viewer code compiles against both concrete adapters in separate builds;
- no virtual backend base, runtime backend registry, dual-backend variant, or
  runtime backend factory is required;
- the vkkk build does not link mt, Metal, or the Swift runtime;
- the mt build does not link vkkk or Vulkan;
- the unselected adapter sources and native dependencies are absent from the
  generated build graph;
- runtime capability checks only select paths inside the selected backend and
  never change the selected library.

### Viewer behavior tests

Verify:

- camera orbit, pan, zoom, perspective/orthographic behavior, and handedness;
- mesh Phong, wireframe, and x-ray modes;
- joint/controller/locator/curve/axis/grid/overlay rendering;
- object, mesh, joint, and rig picking;
- selection invalidation after scene/component changes;
- model frame conversion and winding/normal handling;
- solver/deformer update ordering relative to rendering;
- CPU evaluation and host-upload fallback;
- Qt/AppKit/MTKView host behavior according to the selected configuration.

### Build matrix

The final build matrix should include:

- non-macOS + `ORL_RHI_BACKEND=VKKK`;
- macOS + `ORL_RHI_BACKEND=MT`;
- macOS offscreen/headless RHI tests;
- explicit unavailable-backend configuration failure;
- Debug and Release where the mt toolchain supports both;
- the existing ORL graph/rig/compiler test targets independent of a window.

Each backend is a separate configure/build job. The matrix is not a
requirement to ship one executable containing both graphics libraries.

## 9. Risks and decisions to resolve early

### mt feature coverage

The mt wrapper has a broad planned/current graphics surface, but its API is
not guaranteed to match every vkkk convenience operation. The RHI spike must
identify which viewer features map directly, which need an ORL-owned
implementation, and which need a documented fallback.

### Shader and layout divergence

Vulkan and Metal differ in shader language, resource binding, clip/depth
conventions, and buffer packing. A common semantic pipeline contract plus
backend-specific shader artifacts is safer than pretending the current GLSL
files are portable.

### Window and UI integration

The current viewer chooses GLFW or Qt through vkkk. The mt project provides
AppKit/CAMetalLayer and MTKView adapters, not an automatic Qt/GLFW replacement.
The macOS host choice must be made before claiming full editor parity.

### CUDA and Metal interop

The current Vulkan path can consume device memory in ways that Metal cannot
assume. CPU/host upload is the first macOS correctness path. Any zero-copy
interop must be a separately tested capability, not an implicit RHI guarantee.

### RHI size and ownership

If the adapter contract exposes only raw native calls, viewer code will still
leak backend policy. If it absorbs resources, scenes, loaders, and graph logic,
it becomes a second graphics library or a second viewer. Keep the contract
call-oriented and viewer-focused: ORL owns logical workflow, while vkkk and mt
own actual graphics objects and native scheduling.

### Static selection trade-off

Compile-time selection prevents runtime backend switching and means a backend
change requires a new configure/build. This is intentional. It keeps the
final binary small and predictable, while runtime GPU capability checks still
handle differences between Metal devices or Vulkan devices within the selected
backend.

## 10. First implementation milestone

The first code milestone should be deliberately small:

1. `RhiAdapter`, `RhiFrameState`, `RhiCapabilities`, `SelectedAdapter`, opaque
   display tokens, logical shader IDs, and backend-neutral input/event values;
2. a vkkk build of the static contract for clear + indexed mesh + resize using
   vkkk-owned resources;
3. a separate mt build of the static contract for offscreen clear, resize,
   and mt-owned indexed mesh buffer upload; add indexed draw/pipeline
   registration before declaring the Metal vertical slice complete;
4. one `SceneMeshFeature` implemented through thin adapter calls;
5. one cross-backend lifecycle/display/depth conformance test;
6. no ORL scene/resource manager, solver, CUDA, picking, or full UI rewrite
   in this milestone.

Once that vertical slice is stable, add features in the migration order above.
This keeps the architectural boundary testable before the large viewer
feature set is moved.

## Definition of done

The RHI migration is complete when:

- the ORL adapter contract owns no graphics resources, scene manager, shader
  compiler, pipeline cache, or native command encoder;
- backend selection resolves to one concrete adapter at configure time, with no
  runtime backend dispatcher or dual-backend binary;
- vkkk owns the Vulkan graphics objects and mt owns the Metal graphics objects,
  including their resource lifetime and native caches;
- viewer/domain headers contain no vkkk, Vulkan, Metal, Swift, Qt, or GLFW
  implementation types;
- backend selection is explicit and platform-correct;
- vkkk/Vulkan and mt/Metal both satisfy the thin adapter contract and their
  native resource/scene tests;
- the viewer uses the same logical scene, camera, selection, input, and
  feature calls on both adapters;
- no duplicated ORL model loader, scene manager, pipeline system, or graphics
  resource lifetime system was introduced;
- shader, coordinate, resource, picking, and interop fallbacks are explicit;
- macOS can run the viewer through mt without requiring Vulkan;
- the existing Vulkan path retains its tested fast paths;
- backend-specific optimizations remain available through capability-gated
  extensions rather than leaking into common viewer code.
