#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orl_gpu.h"

#include <string>
#include <vector>

using namespace orlcomp;

namespace {

const char *kCudaAddKernelPtx = R"ptx(
.version 7.0
.target sm_52
.address_size 64

.visible .entry add_scalar_i32(
    .param .u64 data_ptr,
    .param .u32 n,
    .param .s32 addend
)
{
    .reg .pred  %p;
    .reg .b32   %r<8>;
    .reg .b64   %rd<6>;

    ld.param.u64 %rd1, [data_ptr];
    ld.param.u32 %r1, [n];
    ld.param.s32 %r2, [addend];

    mov.u32 %r3, %tid.x;
    mov.u32 %r4, %ctaid.x;
    mov.u32 %r5, %ntid.x;
    mad.lo.s32 %r0, %r4, %r5, %r3;

    setp.ge.u32 %p, %r0, %r1;
    @%p bra DONE;

    mul.wide.u32 %rd2, %r0, 4;
    add.s64 %rd3, %rd1, %rd2;
    ld.global.s32 %r6, [%rd3];
    add.s32 %r7, %r6, %r2;
    st.global.s32 [%rd3], %r7;

DONE:
    ret;
}
)ptx";

} // namespace

TEST_CASE("cuda backend loads PTX and runs kernel", "[orl][gpu][cuda]") {
    OrlGpuEngine gpu(OrlGpuBackend::Cuda);
    gpu.SetDeviceCode(kCudaAddKernelPtx);

    if (!gpu.LoadToDriver()) {
        const auto &errors = gpu.Errors();
        std::string reason = errors.empty() ? "CUDA driver/module load unavailable in this environment"
                                            : errors.back();
        SKIP(reason);
    }

    std::vector<std::int32_t> values{1, 2, 3, 4, 5, 6, 7, 8};
    REQUIRE(gpu.RunCudaInt32AddKernel("add_scalar_i32", &values, 5));
    REQUIRE(values == std::vector<std::int32_t>{6, 7, 8, 9, 10, 11, 12, 13});
}

#endif
