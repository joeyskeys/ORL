## problems

1. No reusable viewer library
orl_viewer is a single executable. Its reusable sources are compiled directly into that executable, while tests only link orlexec/orlcomp. Tests cannot conveniently link ComponentManager, asset loading, or viewer pipeline code.

2. Model loading is GUI-bound
LoadModelOp only gets a path from a file dialog. The actual load, frame conversion, GPU upload, and scene-object creation are all inside on_eval. The frame conversion helper is also private to that .cpp.

3. Bind and auto-weight are deferred viewport features
AutoWeightFeature and DeformerFeature expose only request() plus on_update(). Their actual work is private:

 - auto-weight: AutoWeightFeature::run
 - bind: DeformerFeature::setup
 - LBS evaluation: DeformerFeature::evaluate

They depend on Selection, Context, frame updates, global runtime_config, and write errors only to stderr. Tests have no synchronous data-facing API returning success/errors.

4. IK has the same issue
CreateIkOp and SolverFeature contain the data mutation, but there is no standalone solver/bind operation accepting component data directly.

5. The actual viewer GPU path requires a window
vkkk::Context::init() requires a WindowBackend, creates a Vulkan surface and swapchain. There is no headless/offscreen compute initialization path. Therefore, testing the exact CUDA-to-Vulkan mesh update path currently requires a window backend or a new headless Context path.

6. GPU output readback is incomplete
OrlExecution::evaluate() can download host-bound buffers, so LBS output can be tested through OrlBuffer. However, the viewer uses evaluate_device() and writes CUDA output directly into Vulkan mesh memory. There is no supported Context API to download/read a mesh vertex buffer after that update. Existing debug dumps only cover host-side buffers and are hard-coded to files such as orl_debug_weights.txt and orl_debug_posed.txt.

7. Test isolation and determinism need attention

DrawableMgr is a global singleton.
Scene points to that global manager.
ComponentManager uses unordered storage for general components.
backend selection is process-global through runtime_config.
model loading can produce multiple mesh names without returning a stable result list.

The practical test tiers would be:

1. CPU data/unit tests: joint hierarchy, component packing, model conversion.
2. CPU ORL integration tests: auto-weight, bind capture, LBS.
3. CUDA tests: same pipeline with host readback.
4.Vulkan/CUDA interop tests: actual viewer GPU buffer update and mesh readback.

Currently the data and rendering is not fully isolated yet:

1. Scene points to the global singleton DrawableMgr and LightMgr.
Multiple tests share the same mesh manager.
2. Scene::clear_objects() does not clear meshes.
3. DrawableMgr::load_file() returns no loaded-mesh result; callers must compare names before/after.
4. Scene::camera is not initialized by the constructor, so data-only tests should explicitly set it to nullptr.
5. Scene and Mesh still include Vulkan-related headers even when used only for CPU data.