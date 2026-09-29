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
| Direct click on the same pixel when a hit in that pixel's overlay list is already selected | Resolve the next allowed hit in the GPU list, wrapping around, then replace the complete selection with it. |
| Shift-click on the same pixel when a hit in that pixel's overlay list is already selected | Resolve the next allowed hit that is not already selected and append it. If every allowed hit is selected, do nothing. |
| Shift-click resolves to an already selected element and there is no other allowed hit in the overlay list | No-op. Do not duplicate or reorder it. |
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
- GPU picking is not uniform: joints use an A-buffer, meshes use a single
  `R32Uint` object result, and controllers/locators are CPU-projected. The
  existing joint A-buffer is the basis for a GPU-first picker, but it must be
  generalized to every selectable element and preserve all overlay hits.
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

GPU pick shaders should use a frame-local `PickToken`/identity table to encode
these keys into 32-bit shader values. A token may refer to a component ID,
scene-object identity, or point identity, but the host-side table must be tied
to the render/pick generation that produced it. The selection list stores the
resolved stable key, never a packed index or transient GPU token.

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

## 4. GPU-first picking and click application

GPU picking is the primary selection path for every selectable element. Mesh
instances, joints, controllers, locators, and future point/component
providers should all render pick geometry into the same logical GPU selection
surface. CPU projection is a degraded fallback for environments where GPU
selection preparation or readback is unavailable; it is not a second
selection design with different click semantics.

### 4.1 One GPU pick surface and identity table

Add a common picker/coordinator between `SelectOp` and the viewport features.
Each feature contributes GPU pick fragments, and the coordinator resolves the
readback into candidates without mutating `Selection`.

The pick pass should use a frame-local identity table:

```cpp
struct PickToken {
    std::uint32_t value = 0;
};

struct PickCandidate {
    SelectionTarget target;       // resolved stable SelectionKey
    SelectableKind kind;
    float depth = 0.0f;
    std::uint32_t provider_priority = 0;
    std::uint64_t pick_generation = 0;
};

struct PickResult {
    std::vector<PickCandidate> hits; // normalized front-to-back
    bool hit_any_element = false;    // includes masked kinds
    bool overflow = false;
    std::uint64_t pick_generation = 0;
};
```

The identity table maps GPU tokens to stable keys for exactly the scene,
component packing, and viewport generation used to render the pick pass.
Joints must not rely on resolving a packed index against a later
`packed_joint_ids()` call. Mesh IDs must likewise resolve through the scene
object table associated with that pass.

The existing `JointPickingFeature` already demonstrates the required
direction: it renders joint points to an A-buffer and calls
`read_abuffer_pixel()` to retrieve all nodes for the clicked pixel. Generalize
that mechanism instead of reducing it to a single winning ID:

- `JointPickingFeature` contributes joint tokens.
- `MeshPickingFeature` contributes scene-object tokens; its current
  single-value `R32Uint` target is replaced or supplemented by the common
  A-buffer.
- Controller and locator features receive GPU pick passes using the same
  transforms and visible geometry as their scene passes.
- Point and future vertex/edge/face providers contribute tokens through the
  same interface.

Separate feature passes may share and append to one cleared A-buffer, or use
compatible per-kind A-buffers that the coordinator merges. In either design,
the result must be one ordered list for the clicked pixel, not a hard-coded
kind priority and not whichever asynchronous callback completes first.

### 4.2 Preserve the per-pixel overlay list

The original A-buffer list is essential for overlapping components. Extend its
node payload so every entry contains at least:

```text
GPU pick token
element kind or identity-table domain
fragment depth
next-list index
```

The current joint node stores a vertex/element ID and a next pointer. Add the
kind/token and depth information needed to merge all element types and to
normalize the list deterministically. A fragment shader can write
`gl_FragCoord.z`; the coordinator then sorts front-to-back and uses a stable
token tie-break within a depth epsilon. It must also deduplicate fragments
from the same element, since a mesh triangle or controller curve can produce
multiple entries at one pixel.

The list capacity must be large enough for normal overlays and expose its
overflow state. If the A-buffer overflows, the preferred behavior is to repeat
the pixel pick with a larger capacity or a targeted second pass. Do not claim
that cycling is complete when the GPU reported that some hits were dropped.
If a bounded fallback is temporarily required, expose a visible diagnostic and
test the truncation rule.

The coordinator does not select the first list entry immediately. It retains
the normalized list long enough to apply the active mask and the repeat-click
cycle below. Masking happens after hit collection so a masked mesh remains
distinguishable from empty space and does not accidentally clear a joint
selection.

### 4.3 Repeated-click cycle for one pixel

Add a `PickCycleState` owned by the picker coordinator:

```text
pixel coordinate
pick/render generation
normalized overlay hit list
last cycle position
```

The cached list may be reused when the pixel and render generation are
unchanged. Invalidate it when the cursor pixel changes, the camera/viewport
changes, the scene or component topology changes, the active mask/mode
changes, or the pick pass reports a new generation. A selection mutation caused
by the click itself must not invalidate the list; it is needed to identify the
currently selected hit on the next click.

For each click:

1. Resolve the GPU list to stable keys, remove duplicate fragments, and filter
   out kinds rejected by the captured `SelectionMask`.
2. Find the current cycle anchor. Prefer the focused selected item if it is in
   this pixel's allowed list; otherwise use the last selected item in the
   ordered selection that appears in the list.
3. For a direct click, choose the next allowed hit after the anchor, wrapping
   to the front. If there is no anchor, choose the front-most allowed hit.
   Apply `SelectionAction::Replace`, so the chosen object becomes the only
   selection.
4. For a Shift-click, scan after the anchor, wrapping as needed, and choose
   the first allowed hit that is not already selected. Apply
   `SelectionAction::Add`; existing items retain their positions. If all
   allowed hits are already selected, do nothing.
5. A direct click on a different pixel starts that pixel's cycle at the
   front-most allowed hit. A Shift-click on an already selected hit therefore
   advances to the next unselected overlay entry rather than duplicating it.

This gives the required DCC behavior while preserving the ordered selection
contract:

```text
overlay list:        A, B, C
direct click:        A
direct click again:  B        (selection becomes [B])
Shift-click again:   C        (selection becomes [B, C])
Shift-click again:   A        (selection becomes [B, C, A])
```

The example assumes all three hits pass the active mask. If the list contains
only masked entries, report a masked hit and leave the current selection
unchanged. If the list is empty, direct click clears and Shift-click is a
no-op. Cycling never reorders existing selected items; only an appended
candidate is placed at the end and becomes focus.

### 4.4 Candidate filtering and arbitration

The coordinator performs the same sequence for every element kind:

1. Collect all GPU A-buffer results for the click.
2. Discard stale results whose request, identity-table generation, or scene
   revision no longer matches the click.
3. Resolve tokens to stable `SelectionKey` values.
4. Deduplicate equal keys and normalize front-to-back using depth.
5. Apply the captured `SelectionMask`.
6. Run the repeated-click cycle over the remaining list.
7. If there is no allowed candidate but `hit_any_element` is true, report a
   masked hit. If there was no hit at all, report empty space.

The current unconditional “joint beats mesh” rule must not remain hidden in
`SelectOp`. GPU depth and the normalized overlay list determine the cycle
order. Provider priority and a stable kind/key tie-break are permitted only
within a depth epsilon. CPU fallback providers must return the same ordered
candidate contract, including multiple hits where they can determine them.

The default mode ignores masked entries for cycling, allowing an allowed joint
to be selected through a masked mesh when both are present in the GPU list. A
future mode may request strict masked occlusion, but that must be an explicit
mode policy and must still retain enough hit information to distinguish it
from empty space.

### 4.5 Asynchronous request transaction

Replace `SelectOp`'s independent boolean flags with a `PickTransaction`:

```text
request_id
pixel/cursor position
SelectionAction (Replace or Add)
mask snapshot
scene/topology/camera/pick generation snapshot
identity-table snapshot
expected provider/pass count
completed GPU overlay results
```

Capture the action and mask at mouse-press time. Do not re-read Shift or the
active mode when a GPU callback arrives. The transaction commits only after
the required GPU passes have completed and the complete overlay list is
available. If no GPU provider is available, the CPU fallback can complete the
same transaction immediately.

A new click supersedes an older transaction. Scene clear, model load,
component deletion, project load, topology repack, camera/viewport resize,
camera movement, and mode change invalidate outstanding transactions and the
cached cycle list. Callbacks with an old request ID or pick generation are
ignored.

### 4.6 Applying the result

`SelectOp` becomes a thin gesture adapter:

```text
ignore click while create-joint modal is active
derive Replace/Add from the event
start GPU pick transaction with action + mask snapshot
on completed overlay result:
  cycle to an allowed target
    -> Selection::apply(action, target)
  empty-space:
    Replace -> Selection::clear()
    Add -> no-op
  masked-only hit or exhausted Shift cycle -> no-op
```

All CPU fallback paths must feed this same cycle/result path. They must not
call `selection.set()` or `selection.add()` directly.

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

1. Keep `plan_docs/selection_system_plan.md` as the design contract and record
   the current behavior in focused tests.
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

1. Introduce the GPU pick-token table, common A-buffer payload, normalized
   overlay-list result, provider interfaces, and generation-aware
   transactions.
2. Generalize the existing joint A-buffer to meshes, controllers, locators,
   and other current viewport elements; retain CPU providers only as fallback.
3. Make GPU callbacks carry request IDs, identity snapshots, depth, and all
   overlay hits for the clicked pixel.
4. Implement per-pixel repeat-click cycling for direct and Shift-click,
   including overflow handling and mask filtering.
5. Change `SelectOp` to use only the common GPU/fallback cycle result path.
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
2. Run the viewer with GPU picking enabled and verify that meshes, joints,
   controllers, locators, and points contribute to the same GPU hit-list path.
3. Force GPU preparation to fail and verify that CPU fallback preserves the
   same selection/cycle semantics.
4. Test out-of-order asynchronous callbacks, A-buffer overflow, resize,
   camera movement, scene deletion during a pending pick, and joint repacking
   during a pending pick.
5. Verify mixed mesh-plus-joint binding in `bind`/`all` mode and rejection of
   mesh hits in `joints` mode.
6. Add manual acceptance coverage for overlapping joints, mesh instances,
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
- Every current selectable kind contributes a GPU hit to the common
  per-pixel overlay list.
- A-buffer entries are normalized front-to-back, deduplicated by element, and
  preserve all non-overflowed overlay hits.
- Candidate filtering uses the captured mask, not the mask at callback time.
- Allowed candidates are deduplicated.
- Depth/tie ordering is deterministic and independent of fragment or callback
  order.
- Repeated direct clicks on one pixel cycle A -> B -> C -> A and replace the
  selection each time.
- Repeated Shift-clicks on one pixel append the next unselected hit in list
  order and never duplicate an existing selection.
- Direct cycling wraps even when the next hit is already selected; Shift-cycle
  stops when all allowed hits are selected.
- A masked mesh is distinguishable from empty space.
- A stale request ID, scene revision, or pick generation cannot mutate
  selection.
- Joint packed indices resolve using the render snapshot, or the result is
  safely discarded.
- A-buffer overflow is reported and either retried with sufficient capacity or
  follows the documented incomplete-list policy.
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
6. Click overlapping elements repeatedly and confirm GPU list cycling:
   direct clicks replace with the next hit, while Shift-clicks append the next
   unselected hit in overlay order.

## 9. Follow-up features that should build on this system

These are useful DCC behaviors but should not be implemented as unrelated
special cases:

- Ctrl-click subtract/toggle, represented by another `SelectionAction`.
- Marquee and lasso selection, producing a batch of candidates while retaining
  the same mask and order policy.
- Parent/child or subtree selection policies for joints.
- Hover/preselection and selection outlines as separate visual state.
- Selection history, undo/redo, named selection sets, and optional project
  persistence.
- Component-level mesh selection once vertices/edges/faces have stable IDs and
  dedicated providers.

The core deliverable is complete when uniform direct replacement,
customizable masks/modes, ordered Shift-add, and GPU overlay-list cycling are
enforced by one selection state and one picker result path for every current
viewport element.
