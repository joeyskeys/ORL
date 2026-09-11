#pragma once

#include "graph_ir.hpp"

namespace orlgraph
{

// Effect vocabulary is kept in a small compatibility header so backend
// adapters can depend on effects without importing graph construction helpers.
using Effect = ResourceEffect;
using EffectAccess = AccessMode;

} // namespace orlgraph
