Critical gaps for real deformers:

~~No arrays/buffers or indexed access (vertices[i], weights[i], bones[j])~~
~~No vector component/member access (p.x, p.y, p.z)~~
~~No dynamic function inputs for mesh, bone matrices, weights, output buffers~~
~~No matrix × matrix composition~~
~~No matrix transpose; no matrix inverse~~
~~No cross product, normalize, length, clamp, or quaternion utilities; no lerp utility~~
No GPU thread/buffer model exposed in ORL (threadId, global buffers, kernel parameters)
~~No structs to represent a vertex, influence list, skeleton, or pose~~
No memory/address-space semantics for GPU input/output arrays
No sparse or variable-number bone influences
The CPU/CUDA skinning test currently compiles a fixed LBS checksum; typed buffer-based deformers compile to LLVM IR but do not yet have ORL-level GPU kernel parameters or thread indexing.

The highest-value next additions are:

~~Arrays/buffers with indexing~~
~~Function parameters~~; kernel parameters remain
~~.x/.y/.z component access~~
~~Matrix × matrix plus basic transform utilities~~; add inverse support
GPU kernel built-ins and device buffers

8.10 update:

Gaps for a real GPU LBS deformer:

What needs adding for real GPU LBS:

~~1. Automatic kernel ABI generation~~
~~The compiler generates the CUDA entry kernel and nvvm.annotations itself. Tests do not need AddKernelWrapperForCompute.~~

~~2. General GPU buffer runtime: allocation, upload/download, release, and synchronization~~
~~OrlGpuEngine supports typed scalar/buffer argument binding and explicit block/thread launch configuration.~~
~~The generated CUDA entry reflects the selected ORL function's buffer/scalar parameter ABI and validates runtime bindings.~~
Automatic dispatch selection remains.

3. Data-parallel lowering
~~`global_id()` provides the CUDA 1D work-item index, so a bounds-checked ORL function can process one element per GPU invocation.~~
~~Portable `parallel for` lowers to a sequential CPU loop or a bounds-checked CUDA work-item body.~~

4. Dispatch setup
~~The runtime derives grid size from an element count and uses configurable default/suggested threads per block.~~
`LaunchCudaKernelForElements(vertex_count)` supplies the matching CUDA work-item count.

5. GPU-safe buffer/address-space lowering
~~ORL keeps buffer syntax target-independent while the CUDA backend lowers entry-function buffers and kernel ABI parameters to NVPTX global address space.~~

6. NVPTX-capable LLVM/CUDA environment
The installed LLVM must contain the NVPTX target, and the CUDA driver must be available. The current CMake setup now tolerates LLVM builds without NVPTX, but such builds cannot emit PTX.

You do not need to expose raw threadIdx, blockIdx, or CUDA setup to ORL users.

Recommended language model:


kernel deform(point input_positions[],
              point output_positions[],
              matrix bone_matrices[],
              float weights[],
              int bone_indices[],
              int vertex_count) {
    parallel for (int vertex = 0; vertex < vertex_count; vertex++) {
        int influence = vertex * 2;
        ...
        output_positions[vertex] = skinned_position;
    }
}

Backend behavior:

CPU: lower parallel for as a normal loop initially.
CUDA: lower each logical vertex iteration to one GPU work item.
Runtime: infer dispatch size from vertex_count, allocate/copy buffers, launch, synchronize, and copy back output.
ORL authors only work with vertex; they never manage GPU thread IDs.
This gives you a portable high-level deformer model while making CUDA thread/grid setup an implementation detail. The next concrete implementation step should be a parallel for AST/parser construct plus automatic CUDA kernel-wrapper generation.

Current conclusion: remaining work for a general rigging tool and animation runtime

The compiler now has a portable `parallel for`, typed buffer parameters, structs, matrix/vector math, `quat` utilities, CPU JIT, CUDA kernel generation, GPU buffers, automatic dispatch, and CUDA global-memory buffer lowering. The remaining work is primarily a runtime and rigging-system layer rather than basic LBS compilation.

1. Complete and validate execution backends
- Install/configure oneTBB and finish testing CPU `parallel for` through the JIT runtime.
- Test CUDA PTX generation and GPU LBS execution on an LLVM build containing NVPTX plus a CUDA driver.
- Add ROCm lowering, runtime loading, and backend parity if AMD GPUs are a target.
- Add reliable runtime diagnostics for failed JIT, device compilation, kernel launch, and asynchronous GPU work.

2. Generalize deformation data
- Support variable-length/sparse influences through offset/count buffers or compressed sparse row layouts.
- Add normalized weight handling, zero-weight behavior, influence limits, and validation utilities.
- Add production deformation operations: blend shapes, dual-quaternion skinning, pose-space deformation, corrective shapes, normals/tangents, and bounds updates.
- Define stable packed buffer layouts, alignment rules, and serialization for points, matrices, `quat`, vertices, influences, and poses.

3. Build a rigging model above ORL
- Define first-class skeleton assets: joints, parent hierarchy, rest pose, inverse bind matrices, constraints, and metadata.
- Add rig graph nodes for local/world transforms, parent constraints, IK, aim/orient constraints, blend constraints, retargeting, and custom ORL deformers.
- Add dependency analysis, dirty propagation, cycle detection, deterministic evaluation order, and explicit space conversions.
- Add authoring APIs and a serialization format for rigs, graphs, bindings, and deformer settings.

4. Build the animation runtime
- Represent clips, tracks, keyframes, interpolation modes, events, looping, layers, masks, blend trees, state machines, and timelines.
- Sample and blend translation/rotation/scale using quaternion-safe interpolation such as normalized linear interpolation and slerp.
- Evaluate animation poses, apply constraints, produce skinning matrices, dispatch deformers, and publish render-ready buffers each frame.
- Add time control, seeking, deterministic fixed-step evaluation, caching, and multithreaded frame scheduling.

5. Improve ORL language/runtime ergonomics
- Add first-class kernel/deformer declarations and runtime metadata instead of selecting entry functions only by string.
- Infer and validate dispatch count from the declared `parallel for` bound where possible.
- Support reductions and well-defined synchronization/atomic operations for parallel algorithms that require shared results.
- Extend diagnostics, source locations, reflection, debug IR/PTX dumps, profiling markers, and reproducible test fixtures.
- Define a stable host API for compiling, caching, binding, evaluating, and hot-reloading ORL programs.

6. Production tooling and interoperability
- Import/export common mesh, skeleton, and animation formats such as glTF; add adapters for host DCC tools as needed.
- Provide rig/deformer inspection, pose and weight debugging, GPU buffer inspection, and CPU/GPU result comparison tools.
- Add benchmark scenes and regression tests for correctness, determinism, memory use, compile latency, and frame time.
- Version assets and runtime ABI so rigs and compiled programs remain compatible across releases.

Suggested delivery order:
1. Finish oneTBB and NVPTX environment validation.
2. Implement sparse influences and production LBS/DQS buffers.
3. Add skeleton/pose assets and clip sampling.
4. Add rig graph constraints and frame evaluation.
5. Add authoring, serialization, interchange, debugging, and performance tooling.