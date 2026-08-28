#include "orl_codegen.h"
#include "orl_intrinsics.h"

#if __has_include(<llvm/IR/BasicBlock.h>)

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Verifier.h>
#include <llvm/MC/TargetRegistry.h>
#include <llvm/Support/CodeGen.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Target/TargetMachine.h>
#if __has_include(<llvm/TargetParser/Host.h>)
#include <llvm/TargetParser/Host.h>
#else
#include <llvm/Support/Host.h>
#endif

#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace orlcomp {

namespace {

std::string UnescapeStringLexeme(const std::string &lexeme) {
    if (lexeme.size() < 2 || lexeme.front() != '"' || lexeme.back() != '"') {
        return lexeme;
    }

    std::string out;
    out.reserve(lexeme.size() - 2);
    for (std::size_t i = 1; i + 1 < lexeme.size(); ++i) {
        const char c = lexeme[i];
        if (c != '\\' || i + 2 >= lexeme.size()) {
            out.push_back(c);
            continue;
        }

        const char e = lexeme[++i];
        switch (e) {
        case 'n': out.push_back('\n'); break;
        case 't': out.push_back('\t'); break;
        case 'r': out.push_back('\r'); break;
        case '\\': out.push_back('\\'); break;
        case '"': out.push_back('"'); break;
        default: out.push_back(e); break;
        }
    }
    return out;
}

} // namespace

struct LlvmIrCodegen::Impl {
    struct VariableInfo {
        llvm::Value *slot = nullptr;
        llvm::Type *type = nullptr;
        bool is_buffer = false;
    };

    struct LoopContext {
        llvm::BasicBlock *break_target = nullptr;
        llvm::BasicBlock *continue_target = nullptr;
    };

    explicit Impl(std::string module_name, OrlCodegenTarget target)
        : module_name_(std::move(module_name)),
          target_(target),
          context_(std::make_unique<llvm::LLVMContext>()),
          module_(std::make_unique<llvm::Module>(module_name_, *context_)),
          builder_(*context_) {}

    bool Generate(const Program &program) {
        errors_.clear();
        module_ = std::make_unique<llvm::Module>(module_name_, *context_);
        scopes_.clear();
        loops_.clear();
        struct_types_.clear();
        struct_field_indices_.clear();
        current_function_ = nullptr;
        current_function_return_type_ = nullptr;
        current_function_definition_ = nullptr;
        parallel_body_counter_ = 0;
        generating_parallel_body_ = false;

        if (target_ == OrlCodegenTarget::Host && !ApplyNativeDataLayout()) {
            return false;
        }

        for (const auto &item : program.items) {
            const auto *struct_definition = dynamic_cast<const StructDefinitionStatement *>(item.get());
            if (struct_definition == nullptr) {
                continue;
            }
            if (struct_types_.contains(struct_definition->name)) {
                AddError("Duplicate struct definition: " + struct_definition->name);
                continue;
            }
            struct_types_[struct_definition->name] = llvm::StructType::create(*context_, struct_definition->name);
        }

        for (const auto &item : program.items) {
            const auto *struct_definition = dynamic_cast<const StructDefinitionStatement *>(item.get());
            if (struct_definition == nullptr) {
                continue;
            }
            DefineStruct(*struct_definition);
        }

        for (const auto &item : program.items) {
            const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(item.get());
            if (function == nullptr) {
                if (dynamic_cast<const StructDefinitionStatement *>(item.get()) == nullptr) {
                    AddError("Unsupported top-level statement in IR codegen");
                }
                continue;
            }
            PredeclareFunction(*function);
        }

        for (const auto &item : program.items) {
            const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(item.get());
            if (function != nullptr) {
                GenerateFunction(*function);
            }
        }

        if (target_ == OrlCodegenTarget::Host) {
            for (const auto &item : program.items) {
                const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(item.get());
                if (function != nullptr) {
                    GenerateHostEntryWrapper(*function);
                }
            }
        }

        return errors_.empty();
    }

    std::string DumpIR() const {
        std::string ir;
        llvm::raw_string_ostream stream(ir);
        module_->print(stream, nullptr);
        return ir;
    }

    llvm::Type *MapTypeName(const std::string &type_name) {
        if (type_name == "int") {
            return builder_.getInt64Ty();
        }
        if (type_name == "float") {
            return builder_.getDoubleTy();
        }
        if (type_name == "string") {
            return builder_.getPtrTy();
        }
        if (type_name == "vector" || type_name == "normal" || type_name == "point" ||
            type_name == "vec3" || type_name == "dvec3") {
            return llvm::FixedVectorType::get(builder_.getDoubleTy(), 3);
        }
        if (type_name == "vec4" || type_name == "dvec4" || type_name == "quat") {
            return llvm::FixedVectorType::get(builder_.getDoubleTy(), 4);
        }
        if (type_name == "matrix") {
            return llvm::ArrayType::get(builder_.getDoubleTy(), 16);
        }
        const auto custom_type = struct_types_.find(type_name);
        if (custom_type != struct_types_.end()) {
            return custom_type->second;
        }
        return nullptr;
    }

    bool DefineStruct(const StructDefinitionStatement &definition) {
        const auto type_it = struct_types_.find(definition.name);
        if (type_it == struct_types_.end()) {
            AddError("Internal error: missing struct type " + definition.name);
            return false;
        }

        std::vector<llvm::Type *> field_types;
        field_types.reserve(definition.fields.size());
        std::unordered_map<std::string, unsigned int> field_indices;
        for (unsigned int i = 0; i < definition.fields.size(); ++i) {
            const StructField &field = definition.fields[i];
            llvm::Type *field_type = MapTypeName(field.type_name);
            if (field_type == nullptr) {
                AddError("Unsupported field type '" + field.type_name + "' in struct " + definition.name);
                return false;
            }
            if (!field_indices.emplace(field.name, i).second) {
                AddError("Duplicate field '" + field.name + "' in struct " + definition.name);
                return false;
            }
            field_types.push_back(field_type);
        }
        type_it->second->setBody(field_types, false);
        struct_field_indices_[definition.name] = std::move(field_indices);
        return true;
    }

    bool IsNumericType(llvm::Type *type) const {
        return type != nullptr && (type->isIntegerTy(64) || type->isDoubleTy());
    }

    llvm::Constant *DefaultValueFor(llvm::Type *type) {
        if (type == nullptr) {
            return nullptr;
        }
        return llvm::Constant::getNullValue(type);
    }

    void EnterScope() {
        scopes_.emplace_back();
    }

    void LeaveScope() {
        if (!scopes_.empty()) {
            scopes_.pop_back();
        }
    }

    VariableInfo *FindVariable(const std::string &name) {
        for (auto it = scopes_.rbegin(); it != scopes_.rend(); ++it) {
            const auto variable = it->find(name);
            if (variable != it->end()) {
                return &variable->second;
            }
        }
        return nullptr;
    }

    void AddVariable(const std::string &name, VariableInfo info) {
        if (scopes_.empty()) {
            EnterScope();
        }
        scopes_.back()[name] = info;
    }

    bool ApplyNativeDataLayout() {
        llvm::InitializeNativeTarget();
        llvm::InitializeNativeTargetAsmPrinter();
        const std::string triple = llvm::sys::getDefaultTargetTriple();
        std::string error;
        const llvm::Target *target = llvm::TargetRegistry::lookupTarget(triple, error);
        if (target == nullptr) {
            AddError("Failed to find native LLVM target: " + error);
            return false;
        }

        const llvm::TargetOptions options;
        std::unique_ptr<llvm::TargetMachine> machine(target->createTargetMachine(
            triple, "generic", "", options, std::nullopt, std::nullopt, llvm::CodeGenOptLevel::Default));
        if (!machine) {
            AddError("Failed to create native LLVM target machine");
            return false;
        }

        module_->setTargetTriple(triple);
        module_->setDataLayout(machine->createDataLayout());
        return true;
    }

    llvm::AllocaInst *CreateEntryAlloca(const std::string &name, llvm::Type *type) {
        llvm::IRBuilder<> entry_builder(&current_function_->getEntryBlock(), current_function_->getEntryBlock().begin());
        return entry_builder.CreateAlloca(type, nullptr, name);
    }

    llvm::Value *CastValue(llvm::Value *value, llvm::Type *target_type, const char *context) {
        if (value == nullptr || target_type == nullptr) {
            return nullptr;
        }

        llvm::Type *source_type = value->getType();
        if (source_type == target_type) {
            return value;
        }

        if (source_type->isIntegerTy(1) && target_type->isIntegerTy(64)) {
            return builder_.CreateZExt(value, target_type, "zexttmp");
        }
        if (source_type->isIntegerTy(1) && target_type->isDoubleTy()) {
            return builder_.CreateUIToFP(value, target_type, "booltodouble");
        }
        if (source_type->isIntegerTy(64) && target_type->isDoubleTy()) {
            return builder_.CreateSIToFP(value, target_type, "sitofptmp");
        }
        if (source_type->isDoubleTy() && target_type->isIntegerTy(64)) {
            return builder_.CreateFPToSI(value, target_type, "fptositmp");
        }
        if (target_type->isIntegerTy(1)) {
            if (source_type->isIntegerTy(64)) {
                return builder_.CreateICmpNE(value, llvm::ConstantInt::get(builder_.getInt64Ty(), 0), "inttobool");
            }
            if (source_type->isDoubleTy()) {
                return builder_.CreateFCmpONE(value, llvm::ConstantFP::get(builder_.getDoubleTy(), 0.0), "doubletobool");
            }
        }

        std::ostringstream oss;
        oss << "Type mismatch in " << context << ": cannot cast from ";
        std::string source_text;
        llvm::raw_string_ostream source_stream(source_text);
        source_type->print(source_stream);
        oss << source_stream.str() << " to ";
        std::string target_text;
        llvm::raw_string_ostream target_stream(target_text);
        target_type->print(target_stream);
        oss << target_stream.str();
        AddError(oss.str());
        return nullptr;
    }

    llvm::Value *ToBoolean(llvm::Value *value, const char *context) {
        return CastValue(value, builder_.getInt1Ty(), context);
    }

    void PredeclareFunction(const FunctionDefinitionStatement &function_definition) {
        if (module_->getFunction(function_definition.name) != nullptr) {
            AddError("Duplicate function definition: " + function_definition.name);
            return;
        }

        llvm::Type *return_type = MapTypeName(function_definition.return_type);
        if (return_type == nullptr) {
            AddError("Unsupported function return type: " + function_definition.return_type);
            return;
        }

        std::vector<llvm::Type *> parameter_types;
        parameter_types.reserve(function_definition.parameters.size());
        for (const auto &parameter : function_definition.parameters) {
            llvm::Type *parameter_type = MapTypeName(parameter.type_name);
            if (parameter_type == nullptr) {
                AddError("Unsupported parameter type '" + parameter.type_name + "' in function " + function_definition.name);
                return;
            }
            parameter_types.push_back(parameter.is_buffer ? builder_.getPtrTy() : parameter_type);
        }

        auto *function_type = llvm::FunctionType::get(return_type, parameter_types, false);
        llvm::Function::Create(function_type, llvm::Function::ExternalLinkage, function_definition.name, module_.get());
    }

    bool GenerateFunction(const FunctionDefinitionStatement &function_definition) {
        llvm::Function *function = module_->getFunction(function_definition.name);
        if (function == nullptr) {
            AddError("Internal error: missing predeclared function " + function_definition.name);
            return false;
        }
        if (!function->empty()) {
            return true;
        }

        auto *entry_block = llvm::BasicBlock::Create(*context_, "entry", function);
        builder_.SetInsertPoint(entry_block);
        current_function_ = function;
        current_function_return_type_ = function->getReturnType();
        current_function_definition_ = &function_definition;
        EnterScope();

        std::size_t parameter_index = 0;
        for (auto &argument : function->args()) {
            const auto &parameter_ast = function_definition.parameters[parameter_index++];
            argument.setName(parameter_ast.name);
            if (parameter_ast.is_buffer) {
                llvm::Type *element_type = MapTypeName(parameter_ast.type_name);
                AddVariable(parameter_ast.name,
                            VariableInfo{&argument, element_type, true});
                continue;
            }
            llvm::AllocaInst *slot = CreateEntryAlloca(parameter_ast.name, argument.getType());
            builder_.CreateStore(&argument, slot);
            AddVariable(parameter_ast.name, VariableInfo{slot, argument.getType()});
        }

        GenerateBlock(*function_definition.body);

        if (builder_.GetInsertBlock() != nullptr && builder_.GetInsertBlock()->getTerminator() == nullptr) {
            llvm::Constant *fallback = DefaultValueFor(current_function_return_type_);
            if (fallback != nullptr) {
                builder_.CreateRet(fallback);
            }
        }

        LeaveScope();
        current_function_ = nullptr;
        current_function_return_type_ = nullptr;
        current_function_definition_ = nullptr;

        if (llvm::verifyFunction(*function, &llvm::errs())) {
            AddError("LLVM verifier failed for function: " + function_definition.name);
            return false;
        }
        return true;
    }

    // ORL functions use their natural typed ABI internally. The integration
    // runtime needs one stable, dynamic call shape, so generate a thin wrapper
    // for int-returning functions whose parameters are runtime-supported
    // buffers, ints, and floats.
    void GenerateHostEntryWrapper(const FunctionDefinitionStatement &definition) {
        llvm::Function *target = module_->getFunction(definition.name);
        if (target == nullptr || !target->getReturnType()->isIntegerTy(64)) {
            return;
        }

        std::vector<llvm::Type *> parameter_types = {
            builder_.getPtrTy(), // void* const* buffers
            builder_.getPtrTy(), // const int64_t* integers
            builder_.getPtrTy(), // const double* floats
        };
        const std::string wrapper_name = "__orl_host_entry_" + definition.name;
        if (module_->getFunction(wrapper_name) != nullptr) {
            return;
        }

        std::size_t buffer_index = 0;
        std::size_t int_index = 0;
        std::size_t float_index = 0;
        for (const Parameter &parameter : definition.parameters) {
            if (parameter.is_buffer) {
                ++buffer_index;
            } else if (parameter.type_name == "int") {
                ++int_index;
            } else if (parameter.type_name == "float") {
                ++float_index;
            } else {
                return;
            }
        }

        llvm::Function *wrapper = llvm::Function::Create(
            llvm::FunctionType::get(builder_.getInt64Ty(), parameter_types, false),
            llvm::Function::ExternalLinkage, wrapper_name, module_.get());
        auto argument = wrapper->arg_begin();
        llvm::Value *buffers = &*argument++;
        llvm::Value *integers = &*argument++;
        llvm::Value *floats = &*argument++;

        llvm::BasicBlock *entry = llvm::BasicBlock::Create(*context_, "entry", wrapper);
        llvm::IRBuilder<> wrapper_builder(entry);
        std::vector<llvm::Value *> call_arguments;
        call_arguments.reserve(definition.parameters.size());
        buffer_index = 0;
        int_index = 0;
        float_index = 0;

        for (const Parameter &parameter : definition.parameters) {
            if (parameter.is_buffer) {
                llvm::Value *slot = wrapper_builder.CreateInBoundsGEP(
                    builder_.getPtrTy(), buffers, wrapper_builder.getInt64(buffer_index++));
                call_arguments.push_back(wrapper_builder.CreateLoad(builder_.getPtrTy(), slot));
            } else if (parameter.type_name == "int") {
                llvm::Value *slot = wrapper_builder.CreateInBoundsGEP(
                    builder_.getInt64Ty(), integers, wrapper_builder.getInt64(int_index++));
                call_arguments.push_back(wrapper_builder.CreateLoad(builder_.getInt64Ty(), slot));
            } else {
                llvm::Value *slot = wrapper_builder.CreateInBoundsGEP(
                    builder_.getDoubleTy(), floats, wrapper_builder.getInt64(float_index++));
                call_arguments.push_back(wrapper_builder.CreateLoad(builder_.getDoubleTy(), slot));
            }
        }

        llvm::Value *result = wrapper_builder.CreateCall(target, call_arguments, "orl.exec.result");
        wrapper_builder.CreateRet(result);
    }

    bool GenerateBlock(const BlockStatement &block) {
        EnterScope();
        for (const auto &statement : block.statements) {
            if (builder_.GetInsertBlock() == nullptr || builder_.GetInsertBlock()->getTerminator() != nullptr) {
                break;
            }
            if (!GenerateStatement(*statement)) {
                return false;
            }
        }
        LeaveScope();
        return true;
    }

    bool GenerateStatement(const Statement &statement) {
        if (const auto *block = dynamic_cast<const BlockStatement *>(&statement)) {
            return GenerateBlock(*block);
        }
        if (const auto *expression_statement = dynamic_cast<const ExpressionStatement *>(&statement)) {
            return GenerateExpression(*expression_statement->expression) != nullptr;
        }
        if (const auto *declaration = dynamic_cast<const DeclarationStatement *>(&statement)) {
            return GenerateDeclaration(*declaration);
        }
        if (const auto *return_statement = dynamic_cast<const ReturnStatement *>(&statement)) {
            return GenerateReturn(*return_statement);
        }
        if (const auto *if_statement = dynamic_cast<const IfStatement *>(&statement)) {
            return GenerateIf(*if_statement);
        }
        if (const auto *while_statement = dynamic_cast<const WhileStatement *>(&statement)) {
            return GenerateWhile(*while_statement);
        }
        if (const auto *do_while_statement = dynamic_cast<const DoWhileStatement *>(&statement)) {
            return GenerateDoWhile(*do_while_statement);
        }
        if (const auto *for_statement = dynamic_cast<const ForStatement *>(&statement)) {
            return GenerateFor(*for_statement);
        }
        if (const auto *parallel_for = dynamic_cast<const ParallelForStatement *>(&statement)) {
            if (generating_parallel_body_) {
                AddError("Nested parallel for is not supported on the host target");
                return false;
            }
            return GenerateParallelFor(*parallel_for);
        }
        if (const auto *loop_control = dynamic_cast<const LoopControlStatement *>(&statement)) {
            return GenerateLoopControl(*loop_control);
        }

        AddError("Unsupported statement kind in codegen");
        return false;
    }

    bool GenerateDeclaration(const DeclarationStatement &declaration) {
        llvm::Type *element_type = MapTypeName(declaration.type_name);
        if (element_type == nullptr) {
            AddError("Unsupported declaration type: " + declaration.type_name);
            return false;
        }
        if (FindVariable(declaration.variable_name) != nullptr && !scopes_.empty() && scopes_.back().contains(declaration.variable_name)) {
            AddError("Duplicate variable declaration in scope: " + declaration.variable_name);
            return false;
        }

        if (declaration.array_size != 0) {
            llvm::Type *array_type = llvm::ArrayType::get(element_type, declaration.array_size);
            llvm::AllocaInst *slot = CreateEntryAlloca(declaration.variable_name, array_type);
            builder_.CreateStore(DefaultValueFor(array_type), slot);
            AddVariable(declaration.variable_name, VariableInfo{slot, array_type});
            return true;
        }

        llvm::AllocaInst *slot = CreateEntryAlloca(declaration.variable_name, element_type);
        llvm::Value *value = nullptr;

        if (declaration.initializer != nullptr) {
            value = GenerateExpression(*declaration.initializer);
            value = CastValue(value, element_type, "declaration initializer");
        } else if (!declaration.constructor_arguments.empty()) {
            value = BuildConstructedValue(declaration);
        } else {
            value = DefaultValueFor(element_type);
        }

        if (value == nullptr) {
            return false;
        }

        builder_.CreateStore(value, slot);
        AddVariable(declaration.variable_name, VariableInfo{slot, element_type});
        return true;
    }

    llvm::Value *BuildValueFromArgs(const std::string &type_name, const std::vector<llvm::Value *> &args) {
        llvm::Type *declared_type = MapTypeName(type_name);
        if (declared_type == nullptr) {
            return nullptr;
        }

        if (declared_type->isIntegerTy(64) || declared_type->isDoubleTy() || declared_type->isPointerTy()) {
            if (args.size() != 1) {
                AddError("Constructor for type '" + type_name + "' expects exactly one argument");
                return nullptr;
            }
            return CastValue(args.front(), declared_type, "constructor argument");
        }

        if (auto *vector_type = llvm::dyn_cast<llvm::FixedVectorType>(declared_type)) {
            const unsigned int count = vector_type->getNumElements();
            if (args.size() != count) {
                AddError("Constructor for '" + type_name + "' expects " + std::to_string(count) + " arguments");
                return nullptr;
            }

            llvm::Value *aggregate = llvm::UndefValue::get(vector_type);
            for (unsigned int i = 0; i < count; ++i) {
                llvm::Value *component = CastValue(args[i], builder_.getDoubleTy(), "vector constructor argument");
                if (component == nullptr) {
                    return nullptr;
                }
                aggregate = builder_.CreateInsertElement(aggregate, component, builder_.getInt32(i), "vecins");
            }
            return aggregate;
        }

        if (auto *struct_type = llvm::dyn_cast<llvm::StructType>(declared_type)) {
            if (args.size() != struct_type->getNumElements()) {
                AddError("Constructor for '" + type_name + "' expects " +
                         std::to_string(struct_type->getNumElements()) + " arguments");
                return nullptr;
            }
            llvm::Value *aggregate = llvm::UndefValue::get(struct_type);
            for (unsigned int i = 0; i < struct_type->getNumElements(); ++i) {
                llvm::Value *field = CastValue(args[i], struct_type->getElementType(i), "struct constructor argument");
                if (field == nullptr) {
                    return nullptr;
                }
                aggregate = builder_.CreateInsertValue(aggregate, field, {i}, "structins");
            }
            return aggregate;
        }

        if (auto *array_type = llvm::dyn_cast<llvm::ArrayType>(declared_type)) {
            const std::size_t count = array_type->getNumElements();
            if (args.size() != count) {
                AddError("Constructor for '" + type_name + "' expects " + std::to_string(count) + " arguments");
                return nullptr;
            }

            llvm::Value *aggregate = llvm::UndefValue::get(array_type);
            for (std::size_t i = 0; i < count; ++i) {
                llvm::Value *component = CastValue(args[i], builder_.getDoubleTy(), "matrix constructor argument");
                if (component == nullptr) {
                    return nullptr;
                }
                aggregate = builder_.CreateInsertValue(aggregate, component, {static_cast<unsigned int>(i)}, "arrins");
            }
            return aggregate;
        }

        AddError("Unsupported constructor target type: " + type_name);
        return nullptr;
    }

    llvm::Value *BuildConstructedValue(const DeclarationStatement &declaration) {
        std::vector<llvm::Value *> args;
        args.reserve(declaration.constructor_arguments.size());
        for (const auto &argument : declaration.constructor_arguments) {
            llvm::Value *value = GenerateExpression(*argument);
            if (value == nullptr) {
                return nullptr;
            }
            args.push_back(value);
        }
        return BuildValueFromArgs(declaration.type_name, args);
    }

    bool GenerateReturn(const ReturnStatement &return_statement) {
        llvm::Value *return_value = nullptr;
        if (return_statement.value != nullptr) {
            return_value = GenerateExpression(*return_statement.value);
            return_value = CastValue(return_value, current_function_return_type_, "return");
        } else {
            return_value = DefaultValueFor(current_function_return_type_);
        }

        if (return_value == nullptr) {
            return false;
        }
        builder_.CreateRet(return_value);
        return true;
    }

    bool GenerateIf(const IfStatement &if_statement) {
        llvm::Value *condition_value = GenerateExpression(*if_statement.condition);
        condition_value = ToBoolean(condition_value, "if condition");
        if (condition_value == nullptr) {
            return false;
        }

        llvm::Function *function = builder_.GetInsertBlock()->getParent();
        auto *then_block = llvm::BasicBlock::Create(*context_, "if.then", function);
        auto *merge_block = llvm::BasicBlock::Create(*context_, "if.end", function);
        auto *else_block = if_statement.else_branch != nullptr ? llvm::BasicBlock::Create(*context_, "if.else", function) : merge_block;

        builder_.CreateCondBr(condition_value, then_block, else_block);

        builder_.SetInsertPoint(then_block);
        if (!GenerateStatement(*if_statement.then_branch)) {
            return false;
        }
        if (builder_.GetInsertBlock()->getTerminator() == nullptr) {
            builder_.CreateBr(merge_block);
        }

        if (if_statement.else_branch != nullptr) {
            builder_.SetInsertPoint(else_block);
            if (!GenerateStatement(*if_statement.else_branch)) {
                return false;
            }
            if (builder_.GetInsertBlock()->getTerminator() == nullptr) {
                builder_.CreateBr(merge_block);
            }
        }

        builder_.SetInsertPoint(merge_block);
        return true;
    }

    bool GenerateWhile(const WhileStatement &while_statement) {
        llvm::Function *function = builder_.GetInsertBlock()->getParent();
        auto *condition_block = llvm::BasicBlock::Create(*context_, "while.cond", function);
        auto *body_block = llvm::BasicBlock::Create(*context_, "while.body", function);
        auto *after_block = llvm::BasicBlock::Create(*context_, "while.end", function);

        builder_.CreateBr(condition_block);

        builder_.SetInsertPoint(condition_block);
        llvm::Value *condition = GenerateExpression(*while_statement.condition);
        condition = ToBoolean(condition, "while condition");
        if (condition == nullptr) {
            return false;
        }
        builder_.CreateCondBr(condition, body_block, after_block);

        loops_.push_back(LoopContext{after_block, condition_block});
        builder_.SetInsertPoint(body_block);
        if (!GenerateStatement(*while_statement.body)) {
            loops_.pop_back();
            return false;
        }
        if (builder_.GetInsertBlock()->getTerminator() == nullptr) {
            builder_.CreateBr(condition_block);
        }
        loops_.pop_back();

        builder_.SetInsertPoint(after_block);
        return true;
    }

    bool GenerateDoWhile(const DoWhileStatement &do_while_statement) {
        llvm::Function *function = builder_.GetInsertBlock()->getParent();
        auto *body_block = llvm::BasicBlock::Create(*context_, "do.body", function);
        auto *condition_block = llvm::BasicBlock::Create(*context_, "do.cond", function);
        auto *after_block = llvm::BasicBlock::Create(*context_, "do.end", function);

        builder_.CreateBr(body_block);

        loops_.push_back(LoopContext{after_block, condition_block});
        builder_.SetInsertPoint(body_block);
        if (!GenerateStatement(*do_while_statement.body)) {
            loops_.pop_back();
            return false;
        }
        if (builder_.GetInsertBlock()->getTerminator() == nullptr) {
            builder_.CreateBr(condition_block);
        }

        builder_.SetInsertPoint(condition_block);
        llvm::Value *condition = GenerateExpression(*do_while_statement.condition);
        condition = ToBoolean(condition, "do-while condition");
        if (condition == nullptr) {
            loops_.pop_back();
            return false;
        }
        builder_.CreateCondBr(condition, body_block, after_block);
        loops_.pop_back();

        builder_.SetInsertPoint(after_block);
        return true;
    }

    bool GenerateFor(const ForStatement &for_statement) {
        llvm::Function *function = builder_.GetInsertBlock()->getParent();
        auto *condition_block = llvm::BasicBlock::Create(*context_, "for.cond", function);
        auto *body_block = llvm::BasicBlock::Create(*context_, "for.body", function);
        auto *increment_block = llvm::BasicBlock::Create(*context_, "for.inc", function);
        auto *after_block = llvm::BasicBlock::Create(*context_, "for.end", function);

        if (for_statement.init != nullptr && GenerateExpression(*for_statement.init) == nullptr) {
            return false;
        }
        builder_.CreateBr(condition_block);

        builder_.SetInsertPoint(condition_block);
        llvm::Value *condition = nullptr;
        if (for_statement.condition != nullptr) {
            condition = GenerateExpression(*for_statement.condition);
            condition = ToBoolean(condition, "for condition");
            if (condition == nullptr) {
                return false;
            }
        } else {
            condition = llvm::ConstantInt::getTrue(builder_.getInt1Ty());
        }
        builder_.CreateCondBr(condition, body_block, after_block);

        loops_.push_back(LoopContext{after_block, increment_block});
        builder_.SetInsertPoint(body_block);
        if (!GenerateStatement(*for_statement.body)) {
            loops_.pop_back();
            return false;
        }
        if (builder_.GetInsertBlock()->getTerminator() == nullptr) {
            builder_.CreateBr(increment_block);
        }

        builder_.SetInsertPoint(increment_block);
        if (for_statement.increment != nullptr && GenerateExpression(*for_statement.increment) == nullptr) {
            loops_.pop_back();
            return false;
        }
        if (builder_.GetInsertBlock()->getTerminator() == nullptr) {
            builder_.CreateBr(condition_block);
        }
        loops_.pop_back();

        builder_.SetInsertPoint(after_block);
        return true;
    }

    llvm::Value *EmitParallelIndex() {
        llvm::FunctionCallee global_id = module_->getOrInsertFunction(
            "__orl_global_id", llvm::FunctionType::get(builder_.getInt64Ty(), false));
        return builder_.CreateCall(global_id, {}, "parallel_index");
    }

    bool GenerateHostParallelFor(const ParallelForStatement &parallel_for, llvm::Value *bound) {
        if (current_function_definition_ == nullptr) {
            AddError("Parallel for must be generated inside an ORL function");
            return false;
        }
        const auto *bound_identifier = dynamic_cast<const IdentifierExpression *>(parallel_for.bound.get());
        const auto is_function_parameter = [&](const std::string &name) {
            for (const Parameter &parameter : current_function_definition_->parameters) {
                if (parameter.name == name) {
                    return true;
                }
            }
            return false;
        };
        if (bound_identifier == nullptr || !is_function_parameter(bound_identifier->name)) {
            AddError("Parallel for bound must be a function parameter on the host target");
            return false;
        }

        llvm::Function *outer_function = current_function_;
        llvm::Type *outer_return_type = current_function_return_type_;
        const FunctionDefinitionStatement *outer_definition = current_function_definition_;
        llvm::BasicBlock *outer_block = builder_.GetInsertBlock();
        const std::uint32_t body_id = parallel_body_counter_++;

        std::vector<llvm::Type *> capture_types;
        capture_types.reserve(outer_function->arg_size());
        for (llvm::Argument &argument : outer_function->args()) {
            capture_types.push_back(argument.getType());
        }
        llvm::StructType *context_type = llvm::StructType::create(
            *context_, capture_types, "orl.parallel.context." + std::to_string(body_id));
        llvm::AllocaInst *context_slot = CreateEntryAlloca("parallel.context", context_type);
        for (std::size_t index = 0; index < outer_definition->parameters.size(); ++index) {
            const Parameter &parameter = outer_definition->parameters[index];
            VariableInfo *variable = FindVariable(parameter.name);
            if (variable == nullptr) {
                AddError("Parallel for cannot capture missing function parameter: " + parameter.name);
                return false;
            }
            llvm::Value *value = parameter.is_buffer
                                     ? variable->slot
                                     : builder_.CreateLoad(variable->type, variable->slot, parameter.name + ".capture");
            llvm::Value *field = builder_.CreateStructGEP(context_type, context_slot, index, parameter.name + ".capture.field");
            builder_.CreateStore(value, field);
        }

        auto *body_type = llvm::FunctionType::get(
            builder_.getVoidTy(), {builder_.getPtrTy(), builder_.getInt64Ty()}, false);
        llvm::Function *body_function = llvm::Function::Create(
            body_type,
            llvm::GlobalValue::ExternalLinkage,
            "orl.parallel.body." + std::to_string(body_id),
            module_.get());
        body_function->setCallingConv(llvm::CallingConv::C);
        body_function->addFnAttr(llvm::Attribute::NoUnwind);
        auto body_argument = body_function->arg_begin();
        body_argument->setName("context");
        llvm::Value *body_context = &*body_argument++;
        body_argument->setName(parallel_for.index_name);

        auto outer_scopes = std::move(scopes_);
        const bool outer_generating_parallel_body = generating_parallel_body_;
        current_function_ = body_function;
        current_function_return_type_ = body_function->getReturnType();
        scopes_.clear();
        generating_parallel_body_ = true;
        auto *entry_block = llvm::BasicBlock::Create(*context_, "entry", body_function);
        builder_.SetInsertPoint(entry_block);
        EnterScope();
        llvm::AllocaInst *index_slot = CreateEntryAlloca(parallel_for.index_name, builder_.getInt64Ty());
        builder_.CreateStore(&*body_argument, index_slot);
        AddVariable(parallel_for.index_name, VariableInfo{index_slot, builder_.getInt64Ty()});

        for (std::size_t index = 0; index < outer_definition->parameters.size(); ++index) {
            const Parameter &parameter = outer_definition->parameters[index];
            llvm::Value *field = builder_.CreateStructGEP(context_type, body_context, index, parameter.name + ".field");
            llvm::Type *field_type = capture_types[index];
            llvm::Value *value = builder_.CreateLoad(field_type, field, parameter.name + ".capture");
            if (parameter.is_buffer) {
                llvm::Type *element_type = MapTypeName(parameter.type_name);
                AddVariable(parameter.name, VariableInfo{value, element_type, true});
            } else {
                llvm::AllocaInst *slot = CreateEntryAlloca(parameter.name, field_type);
                builder_.CreateStore(value, slot);
                AddVariable(parameter.name, VariableInfo{slot, field_type});
            }
        }

        const bool generated_body = GenerateStatement(*parallel_for.body);
        if (generated_body && builder_.GetInsertBlock()->getTerminator() == nullptr) {
            builder_.CreateRetVoid();
        }
        LeaveScope();
        scopes_ = std::move(outer_scopes);
        generating_parallel_body_ = outer_generating_parallel_body;
        current_function_ = outer_function;
        current_function_return_type_ = outer_return_type;
        current_function_definition_ = outer_definition;
        builder_.SetInsertPoint(outer_block);
        if (!generated_body) {
            return false;
        }

        llvm::FunctionCallee runtime = module_->getOrInsertFunction(
            "__orl_parallel_for",
            llvm::FunctionType::get(
                builder_.getVoidTy(),
                {builder_.getInt64Ty(), builder_.getInt64Ty(), builder_.getPtrTy(), builder_.getPtrTy()},
                false));
        if (auto *runtime_fn = module_->getFunction("__orl_parallel_for")) {
            runtime_fn->setCallingConv(llvm::CallingConv::C);
        }
        auto *call = builder_.CreateCall(runtime,
            {builder_.getInt64(0), bound, body_function, context_slot});
        call->setCallingConv(llvm::CallingConv::C);
        return true;
    }

    bool GenerateParallelFor(const ParallelForStatement &parallel_for) {
        llvm::Function *function = builder_.GetInsertBlock()->getParent();
        llvm::Value *bound = GenerateExpression(*parallel_for.bound);
        bound = CastValue(bound, builder_.getInt64Ty(), "parallel for bound");
        if (bound == nullptr) {
            return false;
        }
        if (target_ == OrlCodegenTarget::Host) {
            return GenerateHostParallelFor(parallel_for, bound);
        }

        EnterScope();
        llvm::AllocaInst *index_slot = CreateEntryAlloca(parallel_for.index_name, builder_.getInt64Ty());
        llvm::Value *index_value = EmitParallelIndex();
        builder_.CreateStore(index_value, index_slot);
        AddVariable(parallel_for.index_name, VariableInfo{index_slot, builder_.getInt64Ty()});

        auto *body_block = llvm::BasicBlock::Create(*context_, "parallel.body", function);
        auto *after_block = llvm::BasicBlock::Create(*context_, "parallel.end", function);
        llvm::Value *condition = builder_.CreateICmpSLT(index_value, bound, "parallel.condition");
        builder_.CreateCondBr(condition, body_block, after_block);

        builder_.SetInsertPoint(body_block);
        if (!GenerateStatement(*parallel_for.body)) {
            LeaveScope();
            return false;
        }
        if (builder_.GetInsertBlock()->getTerminator() == nullptr) {
            builder_.CreateBr(after_block);
        }
        builder_.SetInsertPoint(after_block);
        LeaveScope();
        return true;
    }

    bool GenerateLoopControl(const LoopControlStatement &loop_control) {
        if (generating_parallel_body_) {
            AddError("break and continue are not supported inside host parallel for");
            return false;
        }
        if (loops_.empty()) {
            AddError(loop_control.kind == LoopControlKind::Break
                         ? "break statement is not inside a loop"
                         : "continue statement is not inside a loop");
            return false;
        }

        const LoopContext &loop = loops_.back();
        if (loop_control.kind == LoopControlKind::Break) {
            builder_.CreateBr(loop.break_target);
        } else {
            builder_.CreateBr(loop.continue_target);
        }
        return true;
    }

    llvm::Value *GenerateExpression(const Expression &expression) {
        if (const auto *identifier = dynamic_cast<const IdentifierExpression *>(&expression)) {
            return GenerateIdentifier(*identifier);
        }
        if (const auto *literal = dynamic_cast<const LiteralExpression *>(&expression)) {
            return GenerateLiteral(*literal);
        }
        if (const auto *unary = dynamic_cast<const UnaryExpression *>(&expression)) {
            return GenerateUnary(*unary);
        }
        if (const auto *binary = dynamic_cast<const BinaryExpression *>(&expression)) {
            return GenerateBinary(*binary);
        }
        if (const auto *assignment = dynamic_cast<const AssignmentExpression *>(&expression)) {
            return GenerateAssignment(*assignment);
        }
        if (const auto *index = dynamic_cast<const IndexExpression *>(&expression)) {
            return GenerateIndex(*index);
        }
        if (const auto *assignment = dynamic_cast<const IndexAssignmentExpression *>(&expression)) {
            return GenerateIndexAssignment(*assignment);
        }
        if (const auto *component = dynamic_cast<const ComponentExpression *>(&expression)) {
            return GenerateComponent(*component);
        }
        if (const auto *assignment = dynamic_cast<const MemberAssignmentExpression *>(&expression)) {
            return GenerateMemberAssignment(*assignment);
        }
        if (const auto *call = dynamic_cast<const CallExpression *>(&expression)) {
            return GenerateCall(*call);
        }

        AddError("Unsupported expression kind in codegen");
        return nullptr;
    }

    llvm::Value *GenerateIndexAddress(const IndexExpression &index, llvm::Type **element_type) {
        const auto *base_identifier = dynamic_cast<const IdentifierExpression *>(index.base.get());
        if (base_identifier == nullptr) {
            AddError("Array indexing currently requires an array variable");
            return nullptr;
        }

        VariableInfo *array_variable = FindVariable(base_identifier->name);
        if (array_variable == nullptr) {
            AddError("Undefined array: " + base_identifier->name);
            return nullptr;
        }
        if (array_variable->is_buffer) {
            llvm::Value *index_value = GenerateExpression(*index.index);
            index_value = CastValue(index_value, builder_.getInt64Ty(), "buffer index");
            if (index_value == nullptr) {
                return nullptr;
            }

            *element_type = array_variable->type;
            return builder_.CreateInBoundsGEP(*element_type,
                                              array_variable->slot,
                                              index_value,
                                              base_identifier->name + ".element");
        }

        auto *array_type = llvm::dyn_cast<llvm::ArrayType>(array_variable->type);
        if (array_type == nullptr) {
            AddError("Cannot index non-array variable: " + base_identifier->name);
            return nullptr;
        }

        llvm::Value *index_value = GenerateExpression(*index.index);
        index_value = CastValue(index_value, builder_.getInt64Ty(), "array index");
        if (index_value == nullptr) {
            return nullptr;
        }

        *element_type = array_type->getElementType();
        return builder_.CreateInBoundsGEP(array_type,
                                          array_variable->slot,
                                          {builder_.getInt64(0), index_value},
                                          base_identifier->name + ".element");
    }

    llvm::Value *GenerateIndex(const IndexExpression &index) {
        llvm::Type *element_type = nullptr;
        llvm::Value *address = GenerateIndexAddress(index, &element_type);
        if (address == nullptr) {
            return nullptr;
        }
        return builder_.CreateLoad(element_type, address, "arrayload");
    }

    bool FindStructField(llvm::StructType *struct_type, const std::string &field_name, unsigned int *index) {
        const auto fields = struct_field_indices_.find(struct_type->getName().str());
        if (fields == struct_field_indices_.end()) {
            AddError("Unknown struct type: " + struct_type->getName().str());
            return false;
        }
        const auto field = fields->second.find(field_name);
        if (field == fields->second.end()) {
            AddError("Unknown field '" + field_name + "' on struct " + struct_type->getName().str());
            return false;
        }
        *index = field->second;
        return true;
    }

    llvm::Value *GenerateAddress(const Expression &expression, llvm::Type **value_type) {
        if (const auto *identifier = dynamic_cast<const IdentifierExpression *>(&expression)) {
            VariableInfo *variable = FindVariable(identifier->name);
            if (variable == nullptr) {
                AddError("Undefined variable: " + identifier->name);
                return nullptr;
            }
            if (variable->is_buffer) {
                AddError("Buffer variable must be indexed: " + identifier->name);
                return nullptr;
            }
            *value_type = variable->type;
            return variable->slot;
        }
        if (const auto *index = dynamic_cast<const IndexExpression *>(&expression)) {
            return GenerateIndexAddress(*index, value_type);
        }
        if (const auto *component = dynamic_cast<const ComponentExpression *>(&expression)) {
            llvm::Type *base_type = nullptr;
            llvm::Value *base_address = GenerateAddress(*component->base, &base_type);
            auto *struct_type = llvm::dyn_cast_or_null<llvm::StructType>(base_type);
            if (base_address == nullptr || struct_type == nullptr) {
                if (base_address != nullptr) {
                    AddError("Member assignment requires a struct value");
                }
                return nullptr;
            }
            unsigned int field_index = 0;
            if (!FindStructField(struct_type, component->component, &field_index)) {
                return nullptr;
            }
            *value_type = struct_type->getElementType(field_index);
            return builder_.CreateStructGEP(struct_type, base_address, field_index, "fieldaddr");
        }
        AddError("Invalid assignment target");
        return nullptr;
    }

    llvm::Value *GenerateComponent(const ComponentExpression &component) {
        llvm::Value *value = GenerateExpression(*component.base);
        if (value == nullptr) {
            return nullptr;
        }

        if (auto *struct_type = llvm::dyn_cast<llvm::StructType>(value->getType())) {
            unsigned int field_index = 0;
            if (!FindStructField(struct_type, component.component, &field_index)) {
                return nullptr;
            }
            return builder_.CreateExtractValue(value, {field_index}, "field");
        }

        const auto *vector_type = llvm::dyn_cast<llvm::FixedVectorType>(value->getType());
        if (vector_type == nullptr || !vector_type->getElementType()->isDoubleTy()) {
            AddError("Member access requires a struct or floating-point vector");
            return nullptr;
        }

        unsigned int index = 0;
        if (component.component == "x") {
            index = 0;
        } else if (component.component == "y") {
            index = 1;
        } else if (component.component == "z") {
            index = 2;
        } else if (component.component == "w") {
            index = 3;
        } else {
            AddError("Unknown vector component: " + component.component);
            return nullptr;
        }

        if (index >= vector_type->getNumElements()) {
            AddError("Vector component '" + component.component + "' is unavailable on a " +
                     std::to_string(vector_type->getNumElements()) + "-component vector");
            return nullptr;
        }
        return builder_.CreateExtractElement(value, builder_.getInt32(index), "component");
    }

    llvm::Value *GenerateIdentifier(const IdentifierExpression &identifier) {
        VariableInfo *variable = FindVariable(identifier.name);
        if (variable == nullptr) {
            AddError("Undefined variable: " + identifier.name);
            return nullptr;
        }
        if (variable->is_buffer) {
            return variable->slot;
        }
        return builder_.CreateLoad(variable->type, variable->slot, identifier.name + ".val");
    }

    llvm::Value *GenerateLiteral(const LiteralExpression &literal) {
        switch (literal.kind) {
        case LiteralKind::Int:
            return llvm::ConstantInt::get(builder_.getInt64Ty(), literal.int_value, true);
        case LiteralKind::Float:
            return llvm::ConstantFP::get(builder_.getDoubleTy(), literal.float_value);
        case LiteralKind::String:
            return builder_.CreateGlobalStringPtr(UnescapeStringLexeme(literal.raw_lexeme), "str");
        }
        AddError("Unsupported literal kind");
        return nullptr;
    }

    llvm::Value *GenerateUnary(const UnaryExpression &unary) {
        llvm::Value *operand = GenerateExpression(*unary.operand);
        if (operand == nullptr) {
            return nullptr;
        }

        switch (unary.op) {
        case UnaryOp::Plus:
            if (!IsNumericType(operand->getType())) {
                AddError("Unary '+' requires numeric operand");
                return nullptr;
            }
            return operand;
        case UnaryOp::Minus:
            if (operand->getType()->isDoubleTy()) {
                return builder_.CreateFNeg(operand, "negtmp");
            }
            if (operand->getType()->isIntegerTy(64)) {
                return builder_.CreateNeg(operand, "negtmp");
            }
            if (operand->getType()->isVectorTy()) {
                return builder_.CreateFNeg(operand, "vecnegtmp");
            }
            AddError("Unary '-' requires numeric operand");
            return nullptr;
        case UnaryOp::Not:
        {
            llvm::Value *as_bool = ToBoolean(operand, "logical not");
            if (as_bool == nullptr) {
                return nullptr;
            }
            return builder_.CreateNot(as_bool, "nottmp");
        }
        }
        AddError("Unsupported unary operator");
        return nullptr;
    }

    bool IsThreeComponentVector(llvm::Type *type) const {
        const auto *vector_type = llvm::dyn_cast_or_null<llvm::FixedVectorType>(type);
        return vector_type != nullptr &&
               vector_type->getNumElements() == 3 &&
               vector_type->getElementType()->isDoubleTy();
    }

    bool IsMatrix4x4(llvm::Type *type) const {
        const auto *array_type = llvm::dyn_cast_or_null<llvm::ArrayType>(type);
        return array_type != nullptr &&
               array_type->getNumElements() == 16 &&
               array_type->getElementType()->isDoubleTy();
    }

    llvm::Value *GenerateMatrixVectorMultiply(llvm::Value *matrix, llvm::Value *vector) {
        if (!IsMatrix4x4(matrix->getType()) || !IsThreeComponentVector(vector->getType())) {
            AddError("Matrix multiplication requires a matrix and a three-component point/vector");
            return nullptr;
        }

        // ORL matrix constructors are row-major. Points/vectors are promoted
        // to homogeneous (x, y, z, 1) so translation in the final matrix
        // column affects skinning positions.
        auto *vector_type = llvm::cast<llvm::FixedVectorType>(vector->getType());
        llvm::Value *result = llvm::UndefValue::get(vector_type);
        for (unsigned int row = 0; row < 3; ++row) {
            llvm::Value *sum = llvm::ConstantFP::get(builder_.getDoubleTy(), 0.0);
            for (unsigned int column = 0; column < 4; ++column) {
                llvm::Value *matrix_element =
                    builder_.CreateExtractValue(matrix, {row * 4 + column}, "matelt");
                llvm::Value *component = column == 3
                                             ? llvm::ConstantFP::get(builder_.getDoubleTy(), 1.0)
                                             : builder_.CreateExtractElement(vector, builder_.getInt32(column), "vecelt");
                sum = builder_.CreateFAdd(sum, builder_.CreateFMul(matrix_element, component, "matmul"), "matsum");
            }
            result = builder_.CreateInsertElement(result, sum, builder_.getInt32(row), "matvec");
        }
        return result;
    }

    llvm::Value *GenerateBinary(const BinaryExpression &binary) {
        llvm::Value *left = GenerateExpression(*binary.left);
        llvm::Value *right = GenerateExpression(*binary.right);
        if (left == nullptr || right == nullptr) {
            return nullptr;
        }

        const bool left_is_vector = IsThreeComponentVector(left->getType());
        const bool right_is_vector = IsThreeComponentVector(right->getType());
        const bool left_is_matrix = IsMatrix4x4(left->getType());
        const bool right_is_matrix = IsMatrix4x4(right->getType());
        if (binary.op == BinaryOp::Multiply && left_is_matrix && right_is_vector) {
            return GenerateMatrixVectorMultiply(left, right);
        }

        if ((binary.op == BinaryOp::Add || binary.op == BinaryOp::Subtract ||
             binary.op == BinaryOp::Multiply || binary.op == BinaryOp::Divide) &&
            (left_is_vector || right_is_vector)) {
            llvm::Value *vector = left_is_vector ? left : right;
            llvm::Value *other = left_is_vector ? right : left;
            if (!IsThreeComponentVector(vector->getType())) {
                AddError("Vector operation requires three-component vectors");
                return nullptr;
            }

            if (IsThreeComponentVector(other->getType())) {
                if (binary.op == BinaryOp::Add) return builder_.CreateFAdd(left, right, "vecadd");
                if (binary.op == BinaryOp::Subtract) return builder_.CreateFSub(left, right, "vecsub");
                if (binary.op == BinaryOp::Multiply) return builder_.CreateFMul(left, right, "vecmul");
                return builder_.CreateFDiv(left, right, "vecdiv");
            }

            if (!other->getType()->isDoubleTy() && !other->getType()->isIntegerTy(64)) {
                AddError("Vector scalar operation requires an int or float scalar");
                return nullptr;
            }
            if (binary.op == BinaryOp::Add || binary.op == BinaryOp::Subtract) {
                AddError("Vector addition/subtraction requires another vector");
                return nullptr;
            }
            if (binary.op == BinaryOp::Divide && !left_is_vector) {
                AddError("Scalar divided by vector is unsupported");
                return nullptr;
            }

            llvm::Value *scalar = CastValue(other, builder_.getDoubleTy(), "vector scalar operation");
            if (scalar == nullptr) {
                return nullptr;
            }
            llvm::Value *splat = builder_.CreateVectorSplat(3, scalar, "vecsplat");
            if (binary.op == BinaryOp::Multiply) {
                return builder_.CreateFMul(vector, splat, "vecscale");
            }
            return builder_.CreateFDiv(vector, splat, "vecscale");
        }

        if (left_is_matrix || right_is_matrix) {
            AddError("Supported matrix operation is matrix * point/vector");
            return nullptr;
        }

        const bool use_float = left->getType()->isDoubleTy() || right->getType()->isDoubleTy();
        const bool use_int = left->getType()->isIntegerTy(64) && right->getType()->isIntegerTy(64);

        auto cast_to_common_numeric = [&](llvm::Value *value) -> llvm::Value * {
            if (use_float) {
                return CastValue(value, builder_.getDoubleTy(), "binary numeric conversion");
            }
            if (use_int) {
                return CastValue(value, builder_.getInt64Ty(), "binary numeric conversion");
            }
            return value;
        };

        switch (binary.op) {
        case BinaryOp::Add:
        case BinaryOp::Subtract:
        case BinaryOp::Multiply:
        case BinaryOp::Divide:
        {
            left = cast_to_common_numeric(left);
            right = cast_to_common_numeric(right);
            if (left == nullptr || right == nullptr || !(left->getType()->isDoubleTy() || left->getType()->isIntegerTy(64))) {
                AddError("Arithmetic operators require int/float operands");
                return nullptr;
            }

            if (left->getType()->isDoubleTy()) {
                switch (binary.op) {
                case BinaryOp::Add: return builder_.CreateFAdd(left, right, "faddtmp");
                case BinaryOp::Subtract: return builder_.CreateFSub(left, right, "fsubtmp");
                case BinaryOp::Multiply: return builder_.CreateFMul(left, right, "fmultmp");
                case BinaryOp::Divide: return builder_.CreateFDiv(left, right, "fdivtmp");
                default: break;
                }
            } else {
                switch (binary.op) {
                case BinaryOp::Add: return builder_.CreateAdd(left, right, "addtmp");
                case BinaryOp::Subtract: return builder_.CreateSub(left, right, "subtmp");
                case BinaryOp::Multiply: return builder_.CreateMul(left, right, "multmp");
                case BinaryOp::Divide: return builder_.CreateSDiv(left, right, "divtmp");
                default: break;
                }
            }
            break;
        }
        case BinaryOp::Modulo:
            if (!(left->getType()->isIntegerTy(64) && right->getType()->isIntegerTy(64))) {
                AddError("Modulo operator requires integer operands");
                return nullptr;
            }
            return builder_.CreateSRem(left, right, "modtmp");
        case BinaryOp::Less:
        case BinaryOp::LessEqual:
        case BinaryOp::Greater:
        case BinaryOp::GreaterEqual:
        case BinaryOp::Equal:
        case BinaryOp::NotEqual:
        {
            if (left->getType()->isDoubleTy() || right->getType()->isDoubleTy()) {
                left = CastValue(left, builder_.getDoubleTy(), "comparison lhs");
                right = CastValue(right, builder_.getDoubleTy(), "comparison rhs");
                if (left == nullptr || right == nullptr) {
                    return nullptr;
                }
                switch (binary.op) {
                case BinaryOp::Less: return builder_.CreateFCmpOLT(left, right, "cmptmp");
                case BinaryOp::LessEqual: return builder_.CreateFCmpOLE(left, right, "cmptmp");
                case BinaryOp::Greater: return builder_.CreateFCmpOGT(left, right, "cmptmp");
                case BinaryOp::GreaterEqual: return builder_.CreateFCmpOGE(left, right, "cmptmp");
                case BinaryOp::Equal: return builder_.CreateFCmpOEQ(left, right, "cmptmp");
                case BinaryOp::NotEqual: return builder_.CreateFCmpONE(left, right, "cmptmp");
                default: break;
                }
            } else if (left->getType()->isIntegerTy(64) && right->getType()->isIntegerTy(64)) {
                switch (binary.op) {
                case BinaryOp::Less: return builder_.CreateICmpSLT(left, right, "cmptmp");
                case BinaryOp::LessEqual: return builder_.CreateICmpSLE(left, right, "cmptmp");
                case BinaryOp::Greater: return builder_.CreateICmpSGT(left, right, "cmptmp");
                case BinaryOp::GreaterEqual: return builder_.CreateICmpSGE(left, right, "cmptmp");
                case BinaryOp::Equal: return builder_.CreateICmpEQ(left, right, "cmptmp");
                case BinaryOp::NotEqual: return builder_.CreateICmpNE(left, right, "cmptmp");
                default: break;
                }
            } else {
                AddError("Comparison operators currently support only int/float operands");
                return nullptr;
            }
            break;
        }
        case BinaryOp::LogicalAnd:
        case BinaryOp::LogicalOr:
        {
            left = ToBoolean(left, "logical lhs");
            right = ToBoolean(right, "logical rhs");
            if (left == nullptr || right == nullptr) {
                return nullptr;
            }
            return binary.op == BinaryOp::LogicalAnd ? builder_.CreateAnd(left, right, "andtmp")
                                                     : builder_.CreateOr(left, right, "ortmp");
        }
        }

        AddError("Unsupported binary operator");
        return nullptr;
    }

    llvm::Value *GenerateAssignment(const AssignmentExpression &assignment) {
        VariableInfo *variable = FindVariable(assignment.target_name);
        if (variable == nullptr) {
            AddError("Assignment to undefined variable: " + assignment.target_name);
            return nullptr;
        }

        llvm::Value *value = GenerateExpression(*assignment.value);
        value = CastValue(value, variable->type, "assignment");
        if (value == nullptr) {
            return nullptr;
        }

        builder_.CreateStore(value, variable->slot);
        return builder_.CreateLoad(variable->type, variable->slot, assignment.target_name + ".assigned");
    }

    llvm::Value *GenerateIndexAssignment(const IndexAssignmentExpression &assignment) {
        llvm::Type *element_type = nullptr;
        llvm::Value *address = GenerateIndexAddress(*assignment.target, &element_type);
        if (address == nullptr) {
            return nullptr;
        }

        llvm::Value *value = GenerateExpression(*assignment.value);
        value = CastValue(value, element_type, "array element assignment");
        if (value == nullptr) {
            return nullptr;
        }

        builder_.CreateStore(value, address);
        return value;
    }

    llvm::Value *GenerateMemberAssignment(const MemberAssignmentExpression &assignment) {
        llvm::Type *field_type = nullptr;
        llvm::Value *address = GenerateAddress(*assignment.target, &field_type);
        if (address == nullptr) {
            return nullptr;
        }
        llvm::Value *value = GenerateExpression(*assignment.value);
        value = CastValue(value, field_type, "struct field assignment");
        if (value == nullptr) {
            return nullptr;
        }
        builder_.CreateStore(value, address);
        return value;
    }

    llvm::Function *GetOrCreateExtern(const std::string &name, const std::vector<llvm::Value *> &args) {
        if (llvm::Function *existing = module_->getFunction(name)) {
            return existing;
        }

        std::vector<llvm::Type *> parameter_types;
        parameter_types.reserve(args.size());
        for (llvm::Value *arg : args) {
            parameter_types.push_back(arg->getType());
        }

        llvm::Type *return_type = builder_.getInt64Ty();
        if (name == "print") {
            return_type = builder_.getVoidTy();
        } else if (name == "dot" &&
                   parameter_types.size() == 2 &&
                   parameter_types[0] == llvm::FixedVectorType::get(builder_.getDoubleTy(), 3) &&
                   parameter_types[1] == llvm::FixedVectorType::get(builder_.getDoubleTy(), 3)) {
            return_type = builder_.getDoubleTy();
        }

        auto *signature = llvm::FunctionType::get(return_type, parameter_types, false);
        return llvm::Function::Create(signature, llvm::Function::ExternalLinkage, name, module_.get());
    }

    bool IsQuaternionType(llvm::Type *type) const {
        const auto *vector_type = llvm::dyn_cast_or_null<llvm::FixedVectorType>(type);
        return vector_type != nullptr &&
               vector_type->getNumElements() == 4 &&
               vector_type->getElementType()->isDoubleTy();
    }

    llvm::Value *ExtractVectorElement(llvm::Value *value, unsigned int index, const char *name) {
        return builder_.CreateExtractElement(value, builder_.getInt32(index), name);
    }

    llvm::Value *BuildQuaternion(llvm::Value *x, llvm::Value *y, llvm::Value *z, llvm::Value *w) {
        auto *type = llvm::FixedVectorType::get(builder_.getDoubleTy(), 4);
        llvm::Value *result = llvm::UndefValue::get(type);
        result = builder_.CreateInsertElement(result, x, builder_.getInt32(0), "quatx");
        result = builder_.CreateInsertElement(result, y, builder_.getInt32(1), "quaty");
        result = builder_.CreateInsertElement(result, z, builder_.getInt32(2), "quatz");
        return builder_.CreateInsertElement(result, w, builder_.getInt32(3), "quatw");
    }

    llvm::Value *BuildVector3(llvm::Value *x, llvm::Value *y, llvm::Value *z) {
        auto *type = llvm::FixedVectorType::get(builder_.getDoubleTy(), 3);
        llvm::Value *result = llvm::UndefValue::get(type);
        result = builder_.CreateInsertElement(result, x, builder_.getInt32(0), "vecx");
        result = builder_.CreateInsertElement(result, y, builder_.getInt32(1), "vecy");
        return builder_.CreateInsertElement(result, z, builder_.getInt32(2), "vecz");
    }

    llvm::Value *GenerateQuaternionMultiply(llvm::Value *left, llvm::Value *right) {
        if (!IsQuaternionType(left->getType()) || !IsQuaternionType(right->getType())) {
            AddError("quat_mul requires two quaternions");
            return nullptr;
        }

        llvm::Value *x1 = ExtractVectorElement(left, 0, "q1x");
        llvm::Value *y1 = ExtractVectorElement(left, 1, "q1y");
        llvm::Value *z1 = ExtractVectorElement(left, 2, "q1z");
        llvm::Value *w1 = ExtractVectorElement(left, 3, "q1w");
        llvm::Value *x2 = ExtractVectorElement(right, 0, "q2x");
        llvm::Value *y2 = ExtractVectorElement(right, 1, "q2y");
        llvm::Value *z2 = ExtractVectorElement(right, 2, "q2z");
        llvm::Value *w2 = ExtractVectorElement(right, 3, "q2w");

        const auto mul = [this](llvm::Value *a, llvm::Value *b) {
            return builder_.CreateFMul(a, b, "quatmul");
        };
        const auto add = [this](llvm::Value *a, llvm::Value *b) {
            return builder_.CreateFAdd(a, b, "quatadd");
        };
        const auto sub = [this](llvm::Value *a, llvm::Value *b) {
            return builder_.CreateFSub(a, b, "quatsub");
        };

        llvm::Value *x = add(add(mul(w1, x2), mul(x1, w2)), sub(mul(y1, z2), mul(z1, y2)));
        llvm::Value *y = add(add(mul(w1, y2), mul(y1, w2)), sub(mul(z1, x2), mul(x1, z2)));
        llvm::Value *z = add(add(mul(w1, z2), mul(z1, w2)), sub(mul(x1, y2), mul(y1, x2)));
        llvm::Value *w = sub(sub(mul(w1, w2), mul(x1, x2)), add(mul(y1, y2), mul(z1, z2)));
        return BuildQuaternion(x, y, z, w);
    }

    llvm::Value *GenerateQuaternionConjugate(llvm::Value *quaternion) {
        if (!IsQuaternionType(quaternion->getType())) {
            AddError("quat_conjugate requires a quaternion");
            return nullptr;
        }
        return BuildQuaternion(builder_.CreateFNeg(ExtractVectorElement(quaternion, 0, "qx"), "qconjx"),
                               builder_.CreateFNeg(ExtractVectorElement(quaternion, 1, "qy"), "qconjy"),
                               builder_.CreateFNeg(ExtractVectorElement(quaternion, 2, "qz"), "qconjz"),
                               ExtractVectorElement(quaternion, 3, "qw"));
    }

    llvm::Value *GenerateQuaternionNormalize(llvm::Value *quaternion) {
        if (!IsQuaternionType(quaternion->getType())) {
            AddError("quat_normalize requires a quaternion");
            return nullptr;
        }

        llvm::Value *length_squared = llvm::ConstantFP::get(builder_.getDoubleTy(), 0.0);
        for (unsigned int i = 0; i < 4; ++i) {
            llvm::Value *component = ExtractVectorElement(quaternion, i, "qnormelt");
            length_squared = builder_.CreateFAdd(length_squared,
                                                 builder_.CreateFMul(component, component, "qnormmul"),
                                                 "qnormsum");
        }
        llvm::Function *sqrt = llvm::Intrinsic::getDeclaration(module_.get(), llvm::Intrinsic::sqrt, {builder_.getDoubleTy()});
        llvm::Value *length = builder_.CreateCall(sqrt, {length_squared}, "qnormlength");
        llvm::Value *splat = builder_.CreateVectorSplat(4, length, "qnormsplat");
        return builder_.CreateFDiv(quaternion, splat, "qnormalize");
    }

    llvm::Value *GenerateCrossProduct(llvm::Value *left, llvm::Value *right) {
        if (!IsThreeComponentVector(left->getType()) || !IsThreeComponentVector(right->getType())) {
            AddError("cross product requires two three-component vectors");
            return nullptr;
        }
        llvm::Value *x = builder_.CreateFSub(builder_.CreateFMul(ExtractVectorElement(left, 1, "lhs_y"),
                                                                 ExtractVectorElement(right, 2, "rhs_z")),
                                             builder_.CreateFMul(ExtractVectorElement(left, 2, "lhs_z"),
                                                                 ExtractVectorElement(right, 1, "rhs_y")),
                                             "crossx");
        llvm::Value *y = builder_.CreateFSub(builder_.CreateFMul(ExtractVectorElement(left, 2, "lhs_z"),
                                                                 ExtractVectorElement(right, 0, "rhs_x")),
                                             builder_.CreateFMul(ExtractVectorElement(left, 0, "lhs_x"),
                                                                 ExtractVectorElement(right, 2, "rhs_z")),
                                             "crossy");
        llvm::Value *z = builder_.CreateFSub(builder_.CreateFMul(ExtractVectorElement(left, 0, "lhs_x"),
                                                                 ExtractVectorElement(right, 1, "rhs_y")),
                                             builder_.CreateFMul(ExtractVectorElement(left, 1, "lhs_y"),
                                                                 ExtractVectorElement(right, 0, "rhs_x")),
                                             "crossz");
        return BuildVector3(x, y, z);
    }

    llvm::Value *GenerateQuaternionRotate(llvm::Value *quaternion, llvm::Value *vector) {
        if (!IsQuaternionType(quaternion->getType()) || !IsThreeComponentVector(vector->getType())) {
            AddError("quat_rotate requires a quaternion and a three-component vector");
            return nullptr;
        }

        llvm::Value *quaternion_vector = BuildVector3(ExtractVectorElement(quaternion, 0, "qx"),
                                                       ExtractVectorElement(quaternion, 1, "qy"),
                                                       ExtractVectorElement(quaternion, 2, "qz"));
        llvm::Value *twice_cross = GenerateCrossProduct(quaternion_vector, vector);
        if (twice_cross == nullptr) {
            return nullptr;
        }
        twice_cross = builder_.CreateFMul(twice_cross,
                                          builder_.CreateVectorSplat(3, llvm::ConstantFP::get(builder_.getDoubleTy(), 2.0)),
                                          "quattrotdouble");
        llvm::Value *w = ExtractVectorElement(quaternion, 3, "qw");
        llvm::Value *w_cross = builder_.CreateFMul(twice_cross, builder_.CreateVectorSplat(3, w), "quatrotw");
        llvm::Value *second_cross = GenerateCrossProduct(quaternion_vector, twice_cross);
        if (second_cross == nullptr) {
            return nullptr;
        }
        return builder_.CreateFAdd(vector, builder_.CreateFAdd(w_cross, second_cross, "quatrotdelta"), "quatrot");
    }

    llvm::Value *GenerateCall(const CallExpression &call) {
        const auto *callee_identifier = dynamic_cast<const IdentifierExpression *>(call.callee.get());
        if (callee_identifier == nullptr) {
            AddError("Only direct function name calls are supported");
            return nullptr;
        }

        std::vector<llvm::Value *> arguments;
        arguments.reserve(call.arguments.size());
        for (const auto &argument : call.arguments) {
            llvm::Value *value = GenerateExpression(*argument);
            if (value == nullptr) {
                return nullptr;
            }
            arguments.push_back(value);
        }

        if (MapTypeName(callee_identifier->name) != nullptr) {
            return BuildValueFromArgs(callee_identifier->name, arguments);
        }

        llvm::Value *intrinsic_result = nullptr;
        std::string intrinsic_error;
        if (OrlIntrinsicCodegen::TryGenerate(
                callee_identifier->name, arguments, builder_, *module_, &intrinsic_result, &intrinsic_error)) {
            if (intrinsic_result == nullptr) {
                AddError(intrinsic_error);
            }
            return intrinsic_result;
        }
        if (callee_identifier->name == "quat_mul") {
            if (arguments.size() != 2) {
                AddError("quat_mul requires exactly two arguments");
                return nullptr;
            }
            return GenerateQuaternionMultiply(arguments[0], arguments[1]);
        }
        if (callee_identifier->name == "quat_conjugate") {
            if (arguments.size() != 1) {
                AddError("quat_conjugate requires exactly one argument");
                return nullptr;
            }
            return GenerateQuaternionConjugate(arguments[0]);
        }
        if (callee_identifier->name == "quat_normalize") {
            if (arguments.size() != 1) {
                AddError("quat_normalize requires exactly one argument");
                return nullptr;
            }
            return GenerateQuaternionNormalize(arguments[0]);
        }
        if (callee_identifier->name == "quat_rotate") {
            if (arguments.size() != 2) {
                AddError("quat_rotate requires exactly two arguments");
                return nullptr;
            }
            return GenerateQuaternionRotate(arguments[0], arguments[1]);
        }

        llvm::Function *callee = GetOrCreateExtern(callee_identifier->name, arguments);
        if (callee == nullptr) {
            AddError("Failed to resolve function: " + callee_identifier->name);
            return nullptr;
        }

        if (callee->arg_size() != arguments.size()) {
            AddError("Argument count mismatch when calling function: " + callee_identifier->name);
            return nullptr;
        }

        std::vector<llvm::Value *> cast_arguments;
        cast_arguments.reserve(arguments.size());
        std::size_t i = 0;
        for (llvm::Argument &parameter : callee->args()) {
            llvm::Value *argument = CastValue(arguments[i++], parameter.getType(), "function call argument");
            if (argument == nullptr) {
                return nullptr;
            }
            cast_arguments.push_back(argument);
        }

        llvm::Value *call_result = nullptr;
        if (callee->getReturnType()->isVoidTy()) {
            call_result = builder_.CreateCall(callee, cast_arguments);
            (void)call_result;
            return llvm::ConstantInt::get(builder_.getInt64Ty(), 0);
        }

        return builder_.CreateCall(callee, cast_arguments, callee_identifier->name + ".call");
    }

    void AddError(const std::string &message) {
        errors_.push_back(message);
    }

    std::string module_name_;
    OrlCodegenTarget target_ = OrlCodegenTarget::Host;
    std::unique_ptr<llvm::LLVMContext> context_;
    std::unique_ptr<llvm::Module> module_;
    llvm::IRBuilder<> builder_;
    std::vector<std::unordered_map<std::string, VariableInfo>> scopes_;
    std::vector<LoopContext> loops_;
    std::unordered_map<std::string, llvm::StructType *> struct_types_;
    std::unordered_map<std::string, std::unordered_map<std::string, unsigned int>> struct_field_indices_;
    llvm::Function *current_function_ = nullptr;
    llvm::Type *current_function_return_type_ = nullptr;
    const FunctionDefinitionStatement *current_function_definition_ = nullptr;
    std::uint32_t parallel_body_counter_ = 0;
    bool generating_parallel_body_ = false;
    std::vector<std::string> errors_;
};

LlvmIrCodegen::LlvmIrCodegen(std::string module_name, OrlCodegenTarget target)
    : impl_(std::make_unique<Impl>(std::move(module_name), target)) {}
LlvmIrCodegen::~LlvmIrCodegen() = default;

bool LlvmIrCodegen::Generate(const Program &program) {
    return impl_->Generate(program);
}

const std::vector<std::string> &LlvmIrCodegen::Errors() const {
    return impl_->errors_;
}

std::string LlvmIrCodegen::DumpIR() const {
    return impl_->DumpIR();
}

const llvm::Module *LlvmIrCodegen::GetModule() const {
    return impl_->module_.get();
}

std::unique_ptr<llvm::Module> LlvmIrCodegen::ReleaseModule() {
    return std::move(impl_->module_);
}

std::unique_ptr<llvm::LLVMContext> LlvmIrCodegen::ReleaseContext() {
    return std::move(impl_->context_);
}

} // namespace orlcomp

#else

#include <utility>

namespace orlcomp {

struct LlvmIrCodegen::Impl {
    explicit Impl(std::string module_name, OrlCodegenTarget) : module_name_(std::move(module_name)) {}

    bool Generate(const Program &) {
        errors_.clear();
        errors_.push_back("LLVM headers are unavailable in this build environment");
        return false;
    }

    std::string DumpIR() const {
        return "";
    }

    std::string module_name_;
    std::vector<std::string> errors_;
};

LlvmIrCodegen::LlvmIrCodegen(std::string module_name, OrlCodegenTarget target)
    : impl_(std::make_unique<Impl>(std::move(module_name), target)) {}
LlvmIrCodegen::~LlvmIrCodegen() = default;

bool LlvmIrCodegen::Generate(const Program &program) {
    return impl_->Generate(program);
}

const std::vector<std::string> &LlvmIrCodegen::Errors() const {
    return impl_->errors_;
}

std::string LlvmIrCodegen::DumpIR() const {
    return impl_->DumpIR();
}

const llvm::Module *LlvmIrCodegen::GetModule() const {
    return nullptr;
}

std::unique_ptr<llvm::Module> LlvmIrCodegen::ReleaseModule() {
    return nullptr;
}

std::unique_ptr<llvm::LLVMContext> LlvmIrCodegen::ReleaseContext() {
    return nullptr;
}

} // namespace orlcomp

#endif
