# Evaluation plan: order from hierarchy and solver IO

The evaluation plan can grow a directed dependency graph from the joint tree plus each solver's declared reads and writes. No user priority is required. The order is a topological sort of that graph. Two pasted IK nodes on the same joints are a validation error, because that graph has no edge between them.

This stays inside `compile_evaluation_plan`. It does not add a second hierarchy.

## Footprints name writes, not only reads

`PartialEvaluationFootprint` already has read ports for joints, controllers, and locators. Add the matching write ports:

- `write_controller_ports`
- `write_locator_ports`

Add the resolved sets on `SolverRegion`:

- `write_controllers`
- `write_locators`

`resolve_port` follows one wire to `find_joint` or `find_locator` and reads the
`name` parameter. A write port uses that same resolver. The typed `handle`
socket decides the component kind; there is no controller find node or
ambiguous index port. A port that matches none of those nodes stays global, as
an unresolved read does today.

Declare those ports on the nodes that need them:

- `ik_two_bone` reads typed `target`/`pole` locator handles and writes typed
  `root`/`mid` joint handles. Its footprint comes from imported handle effects.
- `aim` and `aim_locator` use typed source/destination transform handles.
- Each `copy_*` constraint uses typed source and destination transform handles.

Typed view effects identify the exact joint/locator read and write. No
kind-specific port strings are consulted.

## Three edge rules

Apply these after the existing graph wires:

1. A wire between two solver regions stays an edge.
2. For a joint, controller, or locator, every reader of an element depends on every writer of that element. Joint reads already include ancestors, so a child reader depends on an ancestor writer.
3. If one region writes joint P and another writes joint C, and P is an ancestor of C, the child writer depends on the parent writer.

Parallel batches become the levels of that graph. Two regions share a level only when no edge connects them.

## Errors

These are compilation errors, not an invented order:

- The node-graph schedule has a cycle. `topological_schedule` already reports this, and `dispatch_graph` already returns before JIT when `validation.ok()` is false.
- The evaluation dependency graph has a cycle. That is a real loop, such as two constraints each writing the locator the other reads.
- Two regions write the same joint, controller, or locator, and no path already orders them. The pasted pair of IK nodes is this case: both write the same root and mid, and the paste adds no wire.
- A global region overlaps a declared region and no wire orders them. `fk`, `hd_id`, `spline_ik`, and `full_body_ik` have no declared footprint, so their reads and writes are unknown. A wire can still order them. Otherwise the order is unresolved.
- Hierarchy compilation fails. `ensure_evaluation_plan` already calls `ensure_hierarchy_plan` first.
- Kernel compilation fails inside `build_orl_segment`. This is the JIT failure itself.

`compile_evaluation_plan` writes one string per evaluation error into `EvaluationPlanCompileResult.errors` and does not produce an executable plan.

## Stop JIT and report

`ensure_evaluation_plan` fails closed. On any evaluation error it leaves `compiled_evaluation_plan_` unset and returns the joined message, which it already does for today's plan errors.

`dispatch_graph` must return before `ensure_execution_plan`. That is the JIT entry: `build_orl_segment` compiles each segment to a kernel. An evaluation-plan error, a graph-schedule cycle, or a hierarchy error never reaches it. A later JIT failure also returns immediately, so a failed segment does not continue into later segments.

The viewer currently prints these failures with `std::cerr` and sets `graph_active_` false. That stops evaluation, but the user never sees it. Report the same message in the UI.

Show it once per failed plan, not on every frame while evaluation stays on. `RuntimeHudFeature` rewrites the status bar every frame, so the status bar is not the report. Use `QMessageBox::warning`, the same surface as an open-project failure. Deduplicate on the error text together with the graph revision, and show again only when a new failure appears. A later successful plan clears that reported failure so the same text can be shown if it comes back.

Keep the existing `std::cerr` line. The dialog is the user report; the console line stays for the log.

## Execution follows the plan

`build_dynamic_dispatch_plan` already emits `levels` from `plan.batches`, and it already pulls a region in when one of its dependencies is dirty. Filling `region.dependencies` and rebuilding batches from the topological levels is enough for the plan itself.

The runtime still launches fused segments in graph schedule order and only uses the dispatch plan to skip a clean segment. Two solver regions that must run in a new order have to be split into separate segments, and the solver pass has to launch those segments in `levels` order. Otherwise the evaluation plan records an order the JIT never calls.

## Files

1. `graph_ir.hpp`, `graph_serialization.cpp`, and `orl_graph_import.cpp` for the write-port fields.
2. `graph_resources.cpp` for the constraint and IK footprints.
3. `evaluation.hpp` and `evaluation.cpp` for the writer sets, the three edges, the topological batches, and the compilation errors.
4. `scene_graph_context.cpp` so a failed plan is not stored, and `graph_scene_runtime.cpp` so `dispatch_graph` returns before `ensure_execution_plan` and the solver UI shows the warning once per failure.
5. The solver launch path in `graph_scene_runtime.cpp`, so a successful plan splits segments and launches them in `levels` order.
6. Extend the existing evaluation-plan test: a constraint on a locator that an IK reads lands in an earlier level, two IK nodes on the same joints fail the plan, and a dependency cycle fails the plan with no executable plan.
