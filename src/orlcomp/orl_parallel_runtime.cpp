#include "orl_parallel_runtime.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

extern "C" void __orl_parallel_for(std::int64_t begin,
                                   std::int64_t end,
                                   OrlParallelBodyFn body,
                                   void *context) {
    if (body == nullptr || end <= begin) {
        return;
    }

    tbb::parallel_for(tbb::blocked_range<std::int64_t>(begin, end),
                      [body, context](const tbb::blocked_range<std::int64_t> &range) {
                          for (std::int64_t index = range.begin(); index != range.end(); ++index) {
                              body(context, index);
                          }
                      });
}
