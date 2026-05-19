#pragma once

#include "orl_ast.h"

#include <memory>
#include <string>
#include <vector>

namespace llvm {
class LLVMContext;
class Module;
}

namespace orlcomp {

class LlvmIrCodegen {
public:
    explicit LlvmIrCodegen(std::string module_name = "orl_module");
    ~LlvmIrCodegen();

    bool Generate(const Program &program);

    const std::vector<std::string> &Errors() const;
    std::string DumpIR() const;
    const llvm::Module *GetModule() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace orlcomp
