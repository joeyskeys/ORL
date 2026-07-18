#pragma once

#include <string>
#include <vector>

#include <llvm/IR/IRBuilder.h>

namespace llvm {
class Module;
class Value;
}

namespace orlcomp {

// Lowers ORL's target-independent math built-ins directly to LLVM IR.
class OrlIntrinsicCodegen {
public:
    // Returns true when name is a built-in, including when lowering failed.
    static bool TryGenerate(const std::string &name,
                            const std::vector<llvm::Value *> &arguments,
                            llvm::IRBuilder<> &builder,
                            llvm::Module &module,
                            llvm::Value **result,
                            std::string *error);
};

} // namespace orlcomp
