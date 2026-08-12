#pragma once

#include "orl_ast.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace llvm {
class LLVMContext;
class Module;
}

namespace orlcomp {

enum class OrlCodegenTarget : std::uint8_t {
    Host,
    Cuda
};

class LlvmIrCodegen {
public:
    explicit LlvmIrCodegen(std::string module_name = "orl_module",
                           OrlCodegenTarget target = OrlCodegenTarget::Host);
    ~LlvmIrCodegen();

    bool Generate(const Program &program);

    const std::vector<std::string> &Errors() const;
    std::string DumpIR() const;
    const llvm::Module *GetModule() const;

    std::unique_ptr<llvm::Module> ReleaseModule();
    std::unique_ptr<llvm::LLVMContext> ReleaseContext();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orlcomp
