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

Automatic kernel ABI generation
The compiler should generate the CUDA entry kernel and nvvm.annotations itself. Tests should not need AddKernelWrapperForCompute.

General GPU buffer runtime
OrlGpuEngine needs APIs to allocate device buffers, upload typed data, pass arbitrary buffer/scalar kernel arguments, launch, synchronize, and download output buffers.

Data-parallel lowering
A GPU launch needs one work item per vertex. Today there is no language-level representation of “this iteration belongs to this GPU invocation.”

Dispatch setup
The runtime needs to derive the launch size from vertex_count, choose a block size, calculate the grid, and pass required metadata.

GPU-safe buffer/address-space lowering
ORL should keep buffer syntax target-independent, while the CUDA backend assigns the appropriate LLVM/NVPTX address spaces and kernel argument ABI.

NVPTX-capable LLVM/CUDA environment
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