#pragma once

#include <orlviewer/rhi/rhi_config.hpp>

#if ORL_RHI_BACKEND_VKKK
#include "vkkk_adapter.hpp"
namespace ORL::Rhi {
using SelectedAdapter = VkkkAdapter;
}
#elif ORL_RHI_BACKEND_MT
#include "mt_adapter.hpp"
namespace ORL::Rhi {
using SelectedAdapter = MtAdapter;
}
#else
#error "No ORL RHI backend was selected by CMake"
#endif

namespace ORL::Rhi {
static_assert(StaticAdapter<SelectedAdapter>);
} // namespace ORL::Rhi
