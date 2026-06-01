#include "orl_codegen.h"

#if __has_include(<llvm/IR/BasicBlock.h>)

#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/raw_ostream.h>

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
        llvm::AllocaInst *slot = nullptr;
        llvm::Type *type = nullptr;
    };

    struct LoopContext {
        llvm::BasicBlock *break_target = nullptr;
        llvm::BasicBlock *continue_target = nullptr;
    };

    explicit Impl(std::string module_name)
        : module_name_(std::move(module_name)),
          context_(std::make_unique<llvm::LLVMContext>()),
          module_(std::make_unique<llvm::Module>(module_name_, *context_)),
          builder_(*context_) {}

    bool Generate(const Program &program) {
        errors_.clear();
        module_ = std::make_unique<llvm::Module>(module_name_, *context_);
        scopes_.clear();
        loops_.clear();
        current_function_ = nullptr;
        current_function_return_type_ = nullptr;

        for (const auto &item : program.items) {
            const auto *function = dynamic_cast<const FunctionDefinitionStatement *>(item.get());
            if (function == nullptr) {
                AddError("Only function definitions are supported at top-level in IR codegen");
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
        if (type_name == "vector" || type_name == "normal" || type_name == "point") {
            return llvm::FixedVectorType::get(builder_.getDoubleTy(), 3);
        }
        if (type_name == "matrix") {
            return llvm::ArrayType::get(builder_.getDoubleTy(), 16);
        }
        return nullptr;
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
            parameter_types.push_back(parameter_type);
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
        EnterScope();

        std::size_t parameter_index = 0;
        for (auto &argument : function->args()) {
            const auto &parameter_ast = function_definition.parameters[parameter_index++];
            argument.setName(parameter_ast.name);
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

        if (llvm::verifyFunction(*function, &llvm::errs())) {
            AddError("LLVM verifier failed for function: " + function_definition.name);
            return false;
        }
        return true;
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
        if (const auto *loop_control = dynamic_cast<const LoopControlStatement *>(&statement)) {
            return GenerateLoopControl(*loop_control);
        }

        AddError("Unsupported statement kind in codegen");
        return false;
    }

    bool GenerateDeclaration(const DeclarationStatement &declaration) {
        llvm::Type *declared_type = MapTypeName(declaration.type_name);
        if (declared_type == nullptr) {
            AddError("Unsupported declaration type: " + declaration.type_name);
            return false;
        }
        if (FindVariable(declaration.variable_name) != nullptr && !scopes_.empty() && scopes_.back().contains(declaration.variable_name)) {
            AddError("Duplicate variable declaration in scope: " + declaration.variable_name);
            return false;
        }

        llvm::AllocaInst *slot = CreateEntryAlloca(declaration.variable_name, declared_type);
        llvm::Value *value = nullptr;

        if (declaration.initializer != nullptr) {
            value = GenerateExpression(*declaration.initializer);
            value = CastValue(value, declared_type, "declaration initializer");
        } else if (!declaration.constructor_arguments.empty()) {
            value = BuildConstructedValue(declaration);
        } else {
            value = DefaultValueFor(declared_type);
        }

        if (value == nullptr) {
            return false;
        }

        builder_.CreateStore(value, slot);
        AddVariable(declaration.variable_name, VariableInfo{slot, declared_type});
        return true;
    }

    llvm::Value *BuildConstructedValue(const DeclarationStatement &declaration) {
        llvm::Type *declared_type = MapTypeName(declaration.type_name);
        if (declared_type == nullptr) {
            return nullptr;
        }

        if (declared_type->isIntegerTy(64) || declared_type->isDoubleTy() || declared_type->isPointerTy()) {
            if (declaration.constructor_arguments.size() != 1) {
                AddError("Constructor for type '" + declaration.type_name + "' expects exactly one argument");
                return nullptr;
            }
            llvm::Value *arg = GenerateExpression(*declaration.constructor_arguments.front());
            return CastValue(arg, declared_type, "constructor argument");
        }

        if (auto *vector_type = llvm::dyn_cast<llvm::FixedVectorType>(declared_type)) {
            const unsigned int count = vector_type->getNumElements();
            if (declaration.constructor_arguments.size() != count) {
                AddError("Constructor for '" + declaration.type_name + "' expects " + std::to_string(count) + " arguments");
                return nullptr;
            }

            llvm::Value *aggregate = llvm::UndefValue::get(vector_type);
            for (unsigned int i = 0; i < count; ++i) {
                llvm::Value *component = GenerateExpression(*declaration.constructor_arguments[i]);
                component = CastValue(component, builder_.getDoubleTy(), "vector constructor argument");
                if (component == nullptr) {
                    return nullptr;
                }
                aggregate = builder_.CreateInsertElement(aggregate, component, builder_.getInt32(i), "vecins");
            }
            return aggregate;
        }

        if (auto *array_type = llvm::dyn_cast<llvm::ArrayType>(declared_type)) {
            const std::size_t count = array_type->getNumElements();
            if (declaration.constructor_arguments.size() != count) {
                AddError("Constructor for '" + declaration.type_name + "' expects " + std::to_string(count) + " arguments");
                return nullptr;
            }

            llvm::Value *aggregate = llvm::UndefValue::get(array_type);
            for (std::size_t i = 0; i < count; ++i) {
                llvm::Value *component = GenerateExpression(*declaration.constructor_arguments[i]);
                component = CastValue(component, builder_.getDoubleTy(), "matrix constructor argument");
                if (component == nullptr) {
                    return nullptr;
                }
                aggregate = builder_.CreateInsertValue(aggregate, component, {static_cast<unsigned int>(i)}, "arrins");
            }
            return aggregate;
        }

        AddError("Unsupported constructor target type: " + declaration.type_name);
        return nullptr;
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

    bool GenerateLoopControl(const LoopControlStatement &loop_control) {
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
        if (const auto *call = dynamic_cast<const CallExpression *>(&expression)) {
            return GenerateCall(*call);
        }

        AddError("Unsupported expression kind in codegen");
        return nullptr;
    }

    llvm::Value *GenerateIdentifier(const IdentifierExpression &identifier) {
        VariableInfo *variable = FindVariable(identifier.name);
        if (variable == nullptr) {
            AddError("Undefined variable: " + identifier.name);
            return nullptr;
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

    llvm::Value *GenerateBinary(const BinaryExpression &binary) {
        llvm::Value *left = GenerateExpression(*binary.left);
        llvm::Value *right = GenerateExpression(*binary.right);
        if (left == nullptr || right == nullptr) {
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

        llvm::CallInst *call_inst = builder_.CreateCall(callee, cast_arguments, callee_identifier->name + ".call");
        if (callee->getReturnType()->isVoidTy()) {
            return llvm::ConstantInt::get(builder_.getInt64Ty(), 0);
        }
        return call_inst;
    }

    void AddError(const std::string &message) {
        errors_.push_back(message);
    }

    std::string module_name_;
    std::unique_ptr<llvm::LLVMContext> context_;
    std::unique_ptr<llvm::Module> module_;
    llvm::IRBuilder<> builder_;
    std::vector<std::unordered_map<std::string, VariableInfo>> scopes_;
    std::vector<LoopContext> loops_;
    llvm::Function *current_function_ = nullptr;
    llvm::Type *current_function_return_type_ = nullptr;
    std::vector<std::string> errors_;
};

LlvmIrCodegen::LlvmIrCodegen(std::string module_name) : impl_(std::make_unique<Impl>(std::move(module_name))) {}
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
    explicit Impl(std::string module_name) : module_name_(std::move(module_name)) {}

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

LlvmIrCodegen::LlvmIrCodegen(std::string module_name) : impl_(std::make_unique<Impl>(std::move(module_name))) {}
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
