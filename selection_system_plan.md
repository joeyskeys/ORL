# Viewer Selection System Plan

## 1. Goal and non-negotiable behavior

The viewer needs one selection model for every viewport-selectable element.
Selection behavior must not change because the hit is a joint, controller,
locator, or mesh instance.

The first implementation must provide these semantics:

| Gesture | Result |
| --- | --- |
| Direct click on an allowed element | Replace the complete ordered selection with that element. Every previous element is deselected. |
| Shift-click on an allowed element | Append the element to the ordered selection if it is not already present. |
| Shift-click on an already selected element | No-op. Do not duplicate or reorder it. |
| Direct click on empty space | Clear the selection. |
| Shift-click on empty space | No-op. |
| Click on an element rejected by the active mask | No-op; it is not treated as empty space. |

The selection list is ordered by insertion time. The order is observable API
state, not an implementation detail. The first item is the first selected
element and the last item is the focus. No packing order, hash-map order,
render order, or component creation order may replace that order.

For example, a future parent-constraint operation can require exactly two
items and interpret:

```text
ordered[0] = the object whose parent is changed
ordered[1] = the parent object
```

The operation must consume that order directly; it must not sort the items or
infer the order from `focus()` or packed component indices.

## 2. Current implementation and concrete gaps

The current implementation is a useful starting point, but it is not yet a
general selection system:

- `src/orlviewer/selection.hpp` stores `SelectionRef` values in a vector, but
  the reference combines identity and transform-resolution data. A vector
  selection stores a raw `glm::vec3*`, and scene objects are identified by a
  name.
- `Selection::replace()` replaces all items, but `Selection::set()` removes
  only items of the same kind. Most picker paths call `set()`, so clicking a
  joint can leave an unrelated mesh or controller selected.
- `Selection::add()` does not check identity. Repeated add operations can
  insert duplicates.
- `focus()` is currently the last vector element, which is the correct basis
  for focus, but callers do not have explicit first/second/order APIs.
- There is no selection mask or registered selection-mode abstraction.
- `SelectOp` has kind-specific priority: controllers and locators are tested
  on the CPU, then GPU joints are preferred over GPU meshes, and a separate
  CPU joint fallback is used. This is not a common candidate/arbitration
  path.
- Joint and mesh picking are asynchronous independently. `SelectOp` uses
  `awaiting_joint`, `awaiting_mesh`, and `joint_hit` flags rather than a
  request transaction. A later scene repack can also make a packed joint
  index stale before the callback resolves it.
- The control map has an unmodified left-click binding for `select`, but its
  modifier matching is exact. A Shift-click therefore needs its own binding.
- Joint selection writes the packed `Joint.selected` field and propagates it
  to descendants for display. That field is useful as derived display state,
  but it must not become the authoritative ordered selection list.
- `SceneMeshFeature`, `ControllerFeature`, `LocatorFeature`, operations, the
  property editor, and graph/deformer code inspect `selection.refs()` directly.
  Those consumers need stable query helpers and explicit operation contracts.

The plan below changes the selection boundary first and then migrates these
callers. It does not require a rewrite of the component store or the ORL
handle system.

## 3. Selection model

### 3.1 Selectable kinds and masks

Introduce one central kind enum for viewport selection. The initial kinds are:

```text
SceneObject   a mesh/object instance in vkkk::Scene
Joint         a rig joint
Controller    a controller shape
Locator       a locator
Point         an explicitly registered point/vector target
```

`SceneObject` is the selectable mesh instance. It must not be confused with
the `mesh_name` asset shared by multiple scene objects. Reserve mask bits for
future viewport elements such as vertex, edge, face, curve, and constraint,
but do not pretend those are selectable until they have a provider.

Use a bitmask for the initial API:

```cpp
using SelectionMask = std::uint64_t;

SelectionMask selection_bit(SelectableKind kind);
bool allows(SelectionMask mask, SelectableKind kind);
```

The mask is a candidate filter, not a second selection list. It applies to all
providers and all element kinds uniformly.

### 3.2 Stable selection identity

Separate identity from the current data/transform address. Add a
`SelectionKey` (or equivalent) with this contract:

```cpp
struct SelectionKey {
    SelectableKind kind;
    // ComponentId for Joint/Controller/Locator,
    // a scene-object identity for SceneObject,
    // a registered point identity for Point.
};
```

Identity rules:

1. Joint, controller, and locator keys use `ComponentId`. Packed indices are
   never stored in the selection list.
2. Scene objects use a canonical scene-object identity. Initially this can be
   the unique scene object name, wrapped in a typed key so it cannot collide
   with a component name. If scene objects can be renamed independently,
   introduce a viewer-owned `SceneObjectId` and keep the name only as display
   data.
3. Point/vector selections receive an owner-provided stable token. A raw
   pointer may remain in a resolver payload for the current owner, but it must
   not be used for equality, ordering, persistence, or deferred GPU callbacks.
4. Every key includes its kind. The same text or numeric value in two domains
   is not the same element.

Keep transform resolution lazy. A selection item should contain a stable key
and enough resolver information to obtain an `XformAttr` at use time.
`XformAttr` remains the transform-operation abstraction; it is not the
selection identity.

### 3.3 Ordered selection state

Refactor `Selection` around an ordered vector of unique `SelectionItem`
objects. A separate hash set may accelerate membership, but the vector is the
only source of ordering.

The public operations should be explicit rather than having an ambiguous
`set()` method:

```cpp
enum class SelectionAction {
    Replace,
    Add,
};

enum class SelectionResult {
    Replaced,
    Added,
    AlreadySelected,
    RejectedByMask,
    InvalidTarget,
    Cleared,
    NoOp,
};

SelectionResult apply(SelectionAction action, SelectionTarget target,
    SelectionChangeReason reason);
void clear(SelectionChangeReason reason);
bool contains(const SelectionKey& key) const;
const SelectionItem* first() const;
const SelectionItem* focus() const;       // ordered.back()
const SelectionItem* at(std::size_t index) const;
const std::vector<SelectionItem>& ordered() const;
```

Required invariants:

- `ordered()` contains no duplicate `SelectionKey` values.
- `Add` appends exactly once and never changes an existing item's position.
- `Replace` discards all old items, including items of other kinds.
- `clear()` leaves no stale selected flags.
- removing or invalidating an item preserves the relative order of all
  surviving items.
- all mutations synchronize derived display state once per logical change.

Keep compatibility adapters temporarily if migrating every caller at once is
too invasive, but make `set()` either disappear or become an explicitly named
`replace()`. No new code may depend on same-kind replacement.

### 3.4 Active mode and selection mask

Add a small upper-level registry, owned by the viewer/application rather than
hard-coded into individual picker features:

```cpp
struct SelectionMode {
    std::string id;
    std::string label;
    SelectionMask mask = 0;
};

class SelectionModeRegistry {
public:
    bool register_mode(SelectionMode mode);
    bool set_active(std::string_view id);
    const SelectionMode& active() const;
};
```

`Selection` receives the active mask, or a reference to the registry, and
exposes `selection.mask()`/`selection.mode()`. `SelectOp` asks the selection
context for the mask; it does not contain a joint-versus-mesh branch.

Register these initial modes in `main.cpp` (or a viewer setup module):

```text
all       SceneObject + Joint + Controller + Locator + Point
objects   SceneObject
joints    Joint
rig       Joint + Controller + Locator
bind      SceneObject + Joint
```

The registry must allow application code to add modes and masks without
editing `SelectOp`. The `bind` mode preserves the current mesh-plus-joints
workflow for auto-weight/deformer setup while still allowing ordinary
joint-only work to reject mesh hits.

When changing modes, use an explicit transition policy. The default should be
`PruneDisallowed`: remove currently selected items whose kinds are not in the
new mask, preserve the order of all remaining items, then synchronize visual
state. A mode may opt into `Preserve` if an upper-level tool intentionally
wants hidden selections to survive. Operations must still validate their
required kinds instead of assuming the mask made the selection homogeneous.

Mode state is viewer interaction state. Do not add it to project files in the
first implementation. If selection persistence is later required, serialize
stable keys only and resolve them after scene assets/components are restored.

## 4. Picking and click application

### 4.1 One picker contract

Add a common picker/coordinator between `SelectOp` and the current CPU/GPU
providers. A provider returns candidates, not a final selection:

```cpp
struct PickCandidate {
    SelectionTarget target;
    SelectableKind kind;
    float depth = 0.0f;
    float screen_distance = 0.0f;
    std::uint32_t provider_priority = 0;
    std::uint64_t scene_revision = 0;
    std::uint64_t pick_generation = 0;
};

struct PickResult {
    std::vector<PickCandidate> candidates;
    bool hit_any_element = false; // includes masked kinds
    bool overflow = false;
};
```

The initial providers are:

- controller CPU provider;
- locator CPU provider;
- joint GPU provider with CPU fallback;
- scene-object/mesh GPU provider.

Each provider reports the element kind and stable identity. It may return
multiple overlapping hits when its backend supports that; it must not mutate
`Selection`.

### 4.2 Candidate filtering and arbitration

The coordinator performs the same sequence for every kind:

1. Collect all provider results for the click.
2. Discard stale results whose request/generation or scene revision no longer
   matches the click.
3. Convert packed joint IDs and scene object IDs to stable keys using the
   identity snapshot associated with the rendered pick pass.
4. Apply the active `SelectionMask`.
5. Deduplicate equal keys.
6. Choose the front-most allowed candidate using one documented depth rule.
   Use provider priority and a stable kind/key tie-break only within a depth
   epsilon.
7. If there is no allowed candidate but `hit_any_element` is true, report a
   masked hit. If there was no hit at all, report empty space.

The current unconditional “joint beats mesh” rule must not remain hidden in
`SelectOp`. Prefer a depth-aware common rule. If the rendering backend cannot
read comparable depth for a provider in the first pass, document a temporary
provider priority and cover it with a test; do not let callback arrival order
choose the result.

The mask is applied after detecting hits so a masked mesh does not look like
empty space. This prevents a direct click in joint mode from unexpectedly
clearing an existing joint selection merely because the cursor landed on a
non-selectable mesh. An allowed joint behind a masked mesh may still win if the
active mode is configured to ignore masked occluders. If a future mode needs
strict occlusion, make that an explicit mode policy rather than an accidental
provider detail.

### 4.3 Asynchronous request transaction

Replace `SelectOp`'s independent boolean flags with a
`PickTransaction`:

```text
request_id
cursor position
SelectionAction (Replace or Add)
mask snapshot
scene/topology/pick generation snapshot
expected provider count
completed provider results
```

Capture the action and mask at mouse-press time. Do not re-read Shift or the
active mode when a GPU callback arrives.

The transaction commits once all requested providers complete, or immediately
when no asynchronous provider is available. A new click supersedes an older
transaction. Scene clear, model load, component deletion, project load,
topology repack, resize, and mode change invalidate outstanding transactions.
Callbacks with an old request ID are ignored.

The joint pick pass currently returns packed indices. The pass must return (or
be paired with) the exact `packed_joint_ids()` snapshot/generation used to
render it. Resolving an index against a newly repacked vector is unsafe.
Likewise, mesh pick IDs must resolve through the scene-object map associated
with that render generation, not an unbounded mutable name map.

### 4.4 Applying the result

`SelectOp` becomes a thin gesture adapter:

```text
ignore click while create-joint modal is active
derive Replace/Add from the event
start picker transaction with action + mask snapshot
on completed result:
  allowed candidate -> Selection::apply(action, target)
  empty-space:
    Replace -> Selection::clear()
    Add -> no-op
  masked-only hit -> no-op
```

All CPU fallback paths must feed this same result path. They must not call
`selection.set()` or `selection.add()` directly.

## 5. Input and mode integration

The current `ControlMap::matches()` requires modifiers to match exactly.
Update `resource/config/control_map.json` with two explicit bindings for the
same operation:

```json
{
  "op": "select",
  "scope": "panel",
  "panel": "viewport",
  "input": {
    "type": "mouse_button",
    "button": "left",
    "action": "press"
  }
},
{
  "op": "select",
  "scope": "panel",
  "panel": "viewport",
  "input": {
    "type": "mouse_button",
    "button": "left",
    "action": "press",
    "mods": ["shift"]
  }
}
```

`SelectOp` uses the Shift bit in the event to choose `Add`; the separate
binding is needed so the operation receives the event under the existing exact
modifier matching. Do not broaden modifier matching globally as part of this
feature, because that would change unrelated controls.

Add a mode-switch operation or upper-level API for changing the active
selection mode. The initial UI can be a keyboard or application-controlled
mode switch; the selection core must not depend on a particular UI toolkit.
The active mode and mask should be visible in the HUD/property UI so users can
understand why a click did not select a mesh.

Reserve, but do not require for the first milestone:

- Ctrl-click removal/toggle;
- click cycling through overlapping candidates;
- box/lasso selection;
- hierarchy expansion (select parent, subtree, or descendants);
- hover/preselection.

These should reuse the same `SelectionAction`, mask, candidate, and ordered
list APIs rather than introduce parallel selection state.

## 6. Integration with current viewer code

### 6.1 Core and picker files

Implement the core in `selection.hpp` plus a `.cpp` if notification,
identity-resolution, or mode code makes the header too large. Add focused
headers for the mode registry and picker instead of placing all logic in
`SelectOp`.

Expected responsibilities:

| Area | Planned change |
| --- | --- |
| `selection.hpp` | `SelectableKind`, `SelectionMask`, `SelectionKey`, ordered unique items, explicit replace/add/remove/prune APIs, mask enforcement, focus/order helpers, change notifications |
| mode registry | register application modes and masks; active-mode transition/prune policy |
| picker coordinator | provider registration, candidate arbitration, transaction IDs, stale-result rejection |
| `ops/select_op.hpp` | translate input to a picker request and apply one common result |
| `vp/joint_picking_feature.hpp` and shader | return generation-associated stable-ID mapping and comparable depth/hit data |
| `vp/mesh_picking_feature.hpp` and shader | return generation-associated scene-object identity and depth/hit data |
| CPU controller/locator fallback | become picker providers and honor the same candidate contract |
| `main.cpp` | register default modes, wire picker providers, expose mode switching |
| `resource/config/control_map.json` | add Shift-click selection binding |

### 6.2 Rendering and derived visual state

Keep the ordered selection list authoritative. Update rendering consumers to use
common queries such as `selection.contains(key)` or
`selection.is_selected(kind, id)` instead of reimplementing vector scans where
appropriate.

Preserve the existing joint-chain visual behavior as a derived display policy:

- direct membership is the item in the ordered selection;
- descendant highlighting is a separate computed visual state;
- `Joint.selected` may continue to be filled for the existing shared shader
  layout, but it must be synchronized from the selection model and never read
  back as the selection source;
- audit ORL/runtime users of the `selected` field so an inherited descendant
  highlight is not interpreted as a user-selected item.

The mesh wireframe, controller color, locator color, property editor, and HUD
should all observe the same direct membership and focus state. A future
preselection/hover color must be separate from selected state.

### 6.3 Operation migration

Audit every current selection mutation and make its intent explicit:

- `SelectOp`: common Replace/Add path only.
- `CreateJointOp`, `CreateControllerOp`, `CreateLocatorOp`, and generated IK
  selection: explicit `replace()` when the newly created item should become the
  only selection.
- `CreateLocatorOp` cancellation: restore the saved ordered list, not a list
  rebuilt from unordered component traversal.
- `MirrorOp`: rebuild with an explicit ordered result and deliberately choose
  which mirrored item is focus.
- `ToggleControllerAttachmentOp`: preserve its mixed controller/target
  validation, but use ordered query helpers if an order-sensitive attachment
  shortcut is added.
- `DeleteOp`, clear-scene, model-load, and project-load paths: clear or prune
  before/after destruction so no stale IDs or pointers remain.
- transform operations: retain current focus-kind behavior, but iterate the
  ordered list and never reorder it. Mixed selections such as mesh + joints
  remain valid for bind operations even if a transform acts only on the focus
  kind.
- graph/deformer/auto-weight code: replace ad-hoc scans with helpers such as
  `selected_scene_object()`, `selected_components(kind)`, and
  `exactly_one(kind)`.

Operations that need positional meaning must use `ordered()`/`at(index)`.
Operations that merely need a set of elements may use membership helpers, but
must not reconstruct or sort the authoritative list.

### 6.4 Invalidation and lifetime

Add an explicit invalidation path:

```text
component/scene removed or renamed
project replaced
topology or packed snapshot changed
point owner destroyed
```

The path may be a `Selection::prune_invalid(resolver)` call at known mutation
boundaries initially, followed by change notifications if needed. It must run
before property/operation consumers observe the new scene. A missing target is
removed while surviving items keep their relative order.

Do not hold raw pointers to scene objects, packed joint arrays, or temporary
vectors in deferred picker transactions. Resolve stable keys against the
current scene only after generation validation.

Selection state is not serialized in the first version. On project load, clear
the selection after replacing components and ignore legacy `Joint.selected`
values as an authority. This avoids restoring process-local component IDs that
have been remapped during load.

## 7. Implementation phases

### Phase 0: contract and test scaffolding

1. Add `selection_system_plan.md` (this document) and record the current
   behavior in focused tests.
2. Add unit coverage for direct replacement, cross-kind replacement, current
   focus behavior, and the duplicate behavior that the new system must remove.
3. Define stable key equality and a test resolver for components, scene
   objects, and temporary points.

### Phase 1: ordered core and masks

1. Add selectable kinds, masks, selection keys/items, and explicit mutation
   results.
2. Implement unique ordered storage, `first()`, `at()`, `focus()`,
   `contains()`, and `prune`.
3. Add the mode registry and default modes.
4. Add change reasons/callbacks so Qt/HUD/render consumers can refresh without
   guessing which mutation occurred.
5. Convert joint selected-state synchronization to a derived update from the
   new list.

At the end of this phase, programmatic calls can express all required
semantics without involving GPU picking.

### Phase 2: common picker and input gestures

1. Introduce `PickCandidate`, `PickResult`, provider interfaces, and
   generation-aware transactions.
2. Move current CPU controller/locator/joint logic behind providers.
3. Make GPU callbacks carry request IDs and identity snapshots.
4. Add depth/arbitration data, including the chosen temporary fallback policy
   where the backend cannot yet return common depth.
5. Change `SelectOp` to use only the common result path.
6. Add the unmodified and Shift-click control bindings.

At the end of this phase, direct and Shift-click behavior is identical across
all currently pickable kinds.

### Phase 3: viewer migration

1. Register default modes and wire mode changes in `main.cpp`.
2. Migrate all `selection.set()` call sites to explicit `replace()` or
   `apply()`.
3. Migrate consumers to stable query helpers and ordered iteration.
4. Add invalidation calls to deletion, clear, load, rename, repack, and
   project-load paths.
5. Update property/HUD presentation to show active mode, mask, ordered items,
   and focus.
6. Ensure scene rendering and joint/controller/locator highlighting still
   update after every selection mutation.

### Phase 4: verification and hardening

1. Run headless selection and scene tests.
2. Run the viewer with GPU picking enabled and with GPU preparation forced to
   fail so CPU fallback is exercised.
3. Test out-of-order asynchronous callbacks, resize, scene deletion during a
   pending pick, and joint repacking during a pending pick.
4. Verify mixed mesh-plus-joint binding in `bind`/`all` mode and rejection of
   mesh hits in `joints` mode.
5. Add manual acceptance coverage for overlapping joints, mesh instances,
   controllers, and locators.

## 8. Test matrix

### Core selection tests

- Direct joint click after mesh selection leaves exactly the joint.
- Direct clicks replace across every pair of kinds.
- Shift-click appends in click order across mixed kinds.
- Shift-clicking an existing item is idempotent and does not change order or
  focus.
- Empty direct click clears; empty Shift-click does not.
- A masked-only hit does not clear an existing selection.
- A mode change prunes disallowed kinds while preserving surviving order.
- `first()`, `at(1)`, and `focus()` expose the expected order.
- Component packed-index changes do not change selection identity or order.
- Removing one item retains the relative order of the rest.
- Invalid point owners remove their point selection without dereferencing a
  dangling pointer.

### Picker tests

- Every provider produces the same `SelectionKey` for the same target.
- Candidate filtering uses the captured mask, not the mask at callback time.
- Allowed candidates are deduplicated.
- Depth/tie arbitration is deterministic and independent of callback order.
- A masked mesh is distinguishable from empty space.
- A stale request ID, scene revision, or pick generation cannot mutate
  selection.
- Joint packed indices resolve using the render snapshot, or the result is
  safely discarded.
- CPU fallback and GPU paths produce equivalent selection actions.

### Consumer/operation tests

- A parent-constraint-style operation sees the first and second selected
  objects in click order.
- Transform operations keep the ordered list unchanged and only transform the
  intended focus kind.
- Delete removes selected scene/component elements and leaves no stale refs.
- Clear scene and project load leave an empty, synchronized selection.
- Controller attachment and auto-weight/deformer setup continue to validate
  their required mixed kinds.
- The property editor lists items in selection order and displays the last item
  as focus.

### Manual viewer acceptance

1. In `all` mode, click a mesh, then click a joint: only the joint is
   highlighted.
2. Shift-click a controller and locator: the list and focus show
   mesh/joint/controller/locator in click order.
3. In `joints` mode, clicking a mesh does not add or clear the joint
   selection; clicking a joint does.
4. In `bind` mode, select a mesh and then joints with Shift and run the
   existing binding/auto-weight actions.
5. Delete or reload an element while selected and confirm the UI, colors, and
   operation predicates have no stale selection.
6. Click overlapping elements repeatedly and confirm the documented depth/tie
   rule rather than whichever asynchronous callback completes first.

## 9. Follow-up features that should build on this system

These are useful DCC behaviors but should not be implemented as unrelated
special cases:

- Ctrl-click subtract/toggle, represented by another `SelectionAction`.
- Marquee and lasso selection, producing a batch of candidates while retaining
  the same mask and order policy.
- Cycling through overlapping candidates, using the ordered candidate list and
  a cursor/selection history.
- Parent/child or subtree selection policies for joints.
- Hover/preselection and selection outlines as separate visual state.
- Selection history, undo/redo, named selection sets, and optional project
  persistence.
- Component-level mesh selection once vertices/edges/faces have stable IDs and
  dedicated providers.

The core deliverable is complete when all three required semantics—uniform
direct replacement, customizable masks/modes, and ordered Shift-add—are
enforced by one selection state and one picker result path for every current
viewport element.
