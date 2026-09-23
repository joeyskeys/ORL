## Input xform ambiguous

ORL meant to be a general language, but the source of xform input can be highly realted to the rig component type(joint, locator, controller). It affects the optimization of the eval plan compilation.

Now it uses metadata to setup the info that where the xform may come from, but this's not a good design. The data source is from scene input, not a general function parameter, highly rigging related.

We must solve it in the language level, not relying user to code it explicitly.

Currently the rig part is a execution runtime upon the core ORL language. So maybe we can add definition into that level instead of making the rig concepts into the core language.