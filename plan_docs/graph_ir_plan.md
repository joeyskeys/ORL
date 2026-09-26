# ORL Graph IR implementation plan

This roadmap implements [`graph_ir_design.md`](graph_ir_design.md) without
editing that design document or coupling the graph layer to the viewer.

## Architecture

```text
ORL source -> AST analysis -> node registry
                                \
graph authoring -> typed graph -> validation/mapping
                                      -> graph optimization
                                      -> portable .oro
                                      -> ORL/backend lowering
                                      -> CPU or CUDA execution
```

## Delivery phases

1. **Target-independent graph core**
   - Add the `orlgraph` CMake library under `src/orlgraph`.
   - Define stable IDs, logical types/values, domains, shapes, ports,
     node definitions/instances, connections, interfaces, resources,
     effects, provenance, and deterministic graph ordering.
   - Keep the library independent of LLVM, CUDA, TBB, GLM, Eigen, vkkk,
     Vulkan, `orlexec`, `orlrig`, and the viewer.

2. **ORL semantic analysis and node registry**
   - Add a semantic-analysis target beside the syntax-only parser.
   - Resolve types, functions, structs, calls, constants, buffer access,
     structured loops, and conservative purity/effect summaries.
   - Import supported ORL functions and stdlib algorithms as versioned graph
     node definitions with logical ports and source provenance.

3. **Graph construction and validation**
   - Add graph builders for nodes, ports, edges, graph inputs/outputs,
     constants, resource attributes, and explicit conversions.
   - Validate type, domain, shape, coordinate-space, definition-version,
     required-port, resource, and cycle rules.
   - Add deterministic topological scheduling; keep solver/feedback regions
     explicit rather than treating them as ordinary DAG edges.

4. **Graph optimization**
   - Add a pass manager over graph IR, before LLVM lowering.
   - Implement normalization, conversion insertion, compile-time parameter
     specialization, constant folding, semantic simplification, subgraph
     flattening, purity-aware inlining, effect-rooted DCE, CSE, scheduling,
     and provenance tracking.
   - Preserve opaque boundaries for stateful, external, reduction, atomic,
     synchronization, and solver operations.

5. **`.oro` serialization and reflection**
   - Add deterministic, versioned `.oro` load/save APIs.
   - Serialize logical types, node definitions/references, graph interfaces,
     nodes, edges, resources, parameter maps, schedules, reflection, and
     provenance.
   - Include format/language/ABI versions and content hashes; exclude host
     pointers, LLVM values, device pointers, Vulkan objects, and window state.

6. **Backend lowering and reusable execution**
   - Lower an optimized graph to stable ORL entry functions/planes first,
     reusing the current parser, LLVM codegen, CPU JIT, and CUDA path.
   - Bind logical values/resources with shape and count validation.
   - Compile once per backend and evaluate repeatedly without reparsing for
     every runtime data update.

7. **Standalone rig integration**
   - Adapt `orlrig` mesh, joint, weight, controller, bind, solver, and
     deformer resources without making graph IR depend on viewer data.
   - Replace hard-coded runner sequencing with graph dependencies only after
     the independent graph/compiler layers are stable.

8. **Tests and acceptance**
   - Add independent Catch2 graph tests with no LLVM or viewer dependency.
   - Cover type/shape validation, deterministic scheduling, AST import,
     parameter mapping, flattening, simplification, inlining, DCE, CSE,
     provenance, serialization, and compatibility failures.
   - Add CPU execution tests and conditional CUDA parity/readback tests for
     simple maps, FK, LBS, constraints, and auto-weight.

## First implementation slice

The first code slice is intentionally limited to:

1. `src/orlgraph` and its CMake target.
2. `GraphModule`, `NodeDefinition`, `NodeInstance`, `Port`, `Connection`,
   `GraphValue`, `LogicalType`, and `ResourceEffect`.
3. Graph input/output roots, basic port/type validation, and deterministic
   topological ordering.
4. Hand-built graph tests independent of LLVM, CUDA, vkkk, and the viewer.

AST analysis, optimization, serialization, lowering, and rig integration
start only after this boundary has independent tests.
