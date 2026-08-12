#pragma once

#include <cstdint>

using OrlParallelBodyFn = void (*)(void *context, std::int64_t index);

extern "C" {

void __orl_parallel_for(std::int64_t begin,
                        std::int64_t end,
                        OrlParallelBodyFn body,
                        void *context);

} // extern "C"
