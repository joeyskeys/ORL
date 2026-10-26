# Fused CUDA Solver Kernel

## Objective

Compile every eligible solver node in one evaluation island into one ordered
ORL entry function and execute that entry with one CUDA launch. Intermediate
solver state stays in one packed device allocation; the final joint and
locator views are borrowed by downstream GPU consumers without a solver
readback.

This first implementation is deliberately conservative:

- Fusion is for scalar (`element_count == 1`), device-safe, non-stateful
  regions only.
- Feedback, parallel domains, host adapters, missing capability metadata, and
  unresolved dependencies use the existing segmented fallback.
- The fallback remains a host-visible correctness path and is never reported
  as zero-readback fusion.

## Compile and schedule

`FusionCapability` is part of each node definition and is serialized with the
graph. ORL imports derive the scalar/device flags from the function summary;
stateful and `parallel for` functions are rejected for fusion. The evaluation
plan remains the authority for implicit footprint, hierarchy, resource, and
explicit-sequence dependencies.

The graph lowerer accepts a validated `execution_order`. It verifies that the
order is a complete node permutation and that every graph data or sequence
edge points forward. The scene runtime builds a temporary segment graph with
ordering-only sequence edges between consecutive evaluation-plan regions.
Those edges affect scheduling but are not lowered as function arguments.
`Connection::sequence` is preserved by optimization, serialization, graph
fingerprints, and lowering.

## Runtime artifact

`GraphSceneRuntime` builds one `OrlGraphProgram` and one
`OrlGraphExecution` for an eligible CUDA solver stage. Scene-input/runtime
nodes are retained in the segment and lowered as expressions; their execution
does not become an additional CUDA launch. The fused execution binds:

- one packed `SolverContext` allocation containing context, joints, locators,
  and controllers;
- one typed handle context for joint/locator views;
- immutable hierarchy context/data;
- graph inputs and, when safe, a device dispatch-selection buffer.

The fused branch calls `evaluate_device()` only. It verifies telemetry has zero
download calls/bytes and publishes the final device joint and locator views.
CPU execution, debug/explicit readback, and fallback segments continue to use
`evaluate_result()` and scene writeback.

## Device-state ownership

The packed host allocation is a staging mirror; the CUDA allocation is
authoritative after a fused launch. Typed handle views alias the packed
SolverContext ranges whenever a program also accesses `solver_context`
directly. This prevents typed writes and direct context reads from observing
different arenas.

Packed input revisions carry absolute dirty ranges. A controller, joint, or
locator edit uploads only its changed range, so an input update cannot replace
device-only joint state with an old host mirror. Host snapshots distinguish an
author-authored edit from an unchanged GPU-produced pose. Published device
views are invalidated on unbind, graph/topology rebuild, backend-mode changes,
and packed-handle topology changes.

GPU-written locator state is an intermediate device publication. The current
locator feature intentionally renders host-authored locator components, while
GPU graph consumers resolve the published device view. This keeps the policy
explicit and avoids silently copying locator state back to the host.

## Dirty dispatch

Phase one uses the CPU-built dirty plan only to decide whether the complete
fused entry launches. A clean fused island is skipped.

Phase two optionally binds a hidden device selection buffer and guards each
eligible solver call. The selection is generated from dependency closure and
evaluation-plan order. If a selected region cannot safely support sparse
dispatch, the CUDA planner selects the full plan. Nodes with consumed outputs
are not guarded because skipping them could invalidate a downstream value; the
runtime falls back to phase-one full fused execution for that artifact.

## Fallback and invalidation

Unsupported fusion cases retain segmented execution. Each fallback segment
reads back and commits scene writes before the next segment, ensuring the next
execution sees the preceding segment's joint/locator state. Fusion artifacts
are rebuilt when graph revision, scene-input revision, topology/dependency
revision, graph fingerprint, or backend changes.

## Verification

The regression matrix covers:

- explicit sequence and validated lowering order;
- implicit evaluation-plan dependencies and fusion capability metadata;
- mixed typed/direct solver state in one CUDA arena;
- packed subrange and dirty-range uploads;
- zero-download `evaluate_device()` telemetry;
- CUDA LBS consumption of a published joint view;
- device dispatch lowering and conservative fallback diagnostics;
- hierarchy/locator/device-state behavior in rig and GPU tests.
