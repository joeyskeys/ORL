#include "orl_intrinsics.h"

#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>

namespace orlcomp {

namespace {

bool IsFloatingVector(llvm::Type *type) {
    const auto *vector_type = llvm::dyn_cast_or_null<llvm::FixedVectorType>(type);
    return vector_type != nullptr && vector_type->getElementType()->isDoubleTy();
}

bool IsMatrix4x4(llvm::Type *type) {
    const auto *array_type = llvm::dyn_cast_or_null<llvm::ArrayType>(type);
    return array_type != nullptr && array_type->getNumElements() == 16 && array_type->getElementType()->isDoubleTy();
}

llvm::Value *ExtractElement(llvm::IRBuilder<> &builder, llvm::Value *value, unsigned int index, const char *name) {
    return builder.CreateExtractElement(value, builder.getInt32(index), name);
}

llvm::Value *BuildVector3(llvm::IRBuilder<> &builder, llvm::Value *x, llvm::Value *y, llvm::Value *z) {
    auto *type = llvm::FixedVectorType::get(builder.getDoubleTy(), 3);
    llvm::Value *result = llvm::UndefValue::get(type);
    result = builder.CreateInsertElement(result, x, builder.getInt32(0), "crossx");
    result = builder.CreateInsertElement(result, y, builder.getInt32(1), "crossy");
    return builder.CreateInsertElement(result, z, builder.getInt32(2), "crossz");
}

llvm::Value *GenerateDot(llvm::IRBuilder<> &builder, llvm::Value *left, llvm::Value *right) {
    llvm::Value *product = builder.CreateFMul(left, right, "dotmul");
    const auto *type = llvm::cast<llvm::FixedVectorType>(product->getType());
    llvm::Value *sum = llvm::ConstantFP::get(builder.getDoubleTy(), 0.0);
    for (unsigned int i = 0; i < type->getNumElements(); ++i) {
        sum = builder.CreateFAdd(sum, ExtractElement(builder, product, i, "dotelt"), "dotsum");
    }
    return sum;
}

llvm::Value *GenerateCross(llvm::IRBuilder<> &builder, llvm::Value *left, llvm::Value *right) {
    llvm::Value *x = builder.CreateFSub(builder.CreateFMul(ExtractElement(builder, left, 1, "lhs_y"),
                                                           ExtractElement(builder, right, 2, "rhs_z")),
                                       builder.CreateFMul(ExtractElement(builder, left, 2, "lhs_z"),
                                                           ExtractElement(builder, right, 1, "rhs_y")),
                                       "crossx");
    llvm::Value *y = builder.CreateFSub(builder.CreateFMul(ExtractElement(builder, left, 2, "lhs_z"),
                                                           ExtractElement(builder, right, 0, "rhs_x")),
                                       builder.CreateFMul(ExtractElement(builder, left, 0, "lhs_x"),
                                                           ExtractElement(builder, right, 2, "rhs_z")),
                                       "crossy");
    llvm::Value *z = builder.CreateFSub(builder.CreateFMul(ExtractElement(builder, left, 0, "lhs_x"),
                                                           ExtractElement(builder, right, 1, "rhs_y")),
                                       builder.CreateFMul(ExtractElement(builder, left, 1, "lhs_y"),
                                                           ExtractElement(builder, right, 0, "rhs_x")),
                                       "crossz");
    return BuildVector3(builder, x, y, z);
}

llvm::Value *GenerateLength(llvm::IRBuilder<> &builder, llvm::Module &module, llvm::Value *vector) {
    llvm::Function *sqrt =
        llvm::Intrinsic::getDeclaration(&module, llvm::Intrinsic::sqrt, {builder.getDoubleTy()});
    return builder.CreateCall(sqrt, {GenerateDot(builder, vector, vector)}, "veclength");
}

llvm::Value *GenerateLerp(llvm::IRBuilder<> &builder,
                          llvm::Value *start,
                          llvm::Value *end,
                          llvm::Value *factor) {
    return builder.CreateFAdd(start,
                              builder.CreateFMul(builder.CreateFSub(end, start, "lerpdifference"),
                                                 factor,
                                                 "lerpscale"),
                              "lerp");
}

llvm::Value *BuildMatrix(llvm::IRBuilder<> &builder, const std::vector<llvm::Value *> &elements, const char *name) {
    auto *type = llvm::ArrayType::get(builder.getDoubleTy(), 16);
    llvm::Value *result = llvm::UndefValue::get(type);
    for (unsigned int i = 0; i < 16; ++i) {
        result = builder.CreateInsertValue(result, elements[i], {i}, name);
    }
    return result;
}

llvm::Value *GenerateMatrixMultiply(llvm::IRBuilder<> &builder, llvm::Value *left, llvm::Value *right) {
    std::vector<llvm::Value *> elements;
    elements.reserve(16);
    for (unsigned int row = 0; row < 4; ++row) {
        for (unsigned int column = 0; column < 4; ++column) {
            llvm::Value *sum = llvm::ConstantFP::get(builder.getDoubleTy(), 0.0);
            for (unsigned int k = 0; k < 4; ++k) {
                llvm::Value *left_element = builder.CreateExtractValue(left, {row * 4 + k}, "matlhs");
                llvm::Value *right_element = builder.CreateExtractValue(right, {k * 4 + column}, "matrhs");
                sum = builder.CreateFAdd(sum, builder.CreateFMul(left_element, right_element, "matmul"), "matsum");
            }
            elements.push_back(sum);
        }
    }
    return BuildMatrix(builder, elements, "matresult");
}

llvm::Value *GenerateMatrixVectorMultiply(llvm::IRBuilder<> &builder, llvm::Value *matrix, llvm::Value *vector) {
    std::vector<llvm::Value *> result;
    result.reserve(3);
    for (unsigned int row = 0; row < 3; ++row) {
        llvm::Value *sum = llvm::ConstantFP::get(builder.getDoubleTy(), 0.0);
        for (unsigned int column = 0; column < 4; ++column) {
            llvm::Value *component = column == 3
                ? llvm::ConstantFP::get(builder.getDoubleTy(), 1.0)
                : ExtractElement(builder, vector, column, "vecelt");
            llvm::Value *matrix_element = builder.CreateExtractValue(matrix, {row * 4 + column}, "matelt");
            sum = builder.CreateFAdd(sum, builder.CreateFMul(matrix_element, component, "matvecmul"), "matvecsum");
        }
        result.push_back(sum);
    }
    return BuildVector3(builder, result[0], result[1], result[2]);
}

llvm::Value *GenerateMatrixTranspose(llvm::IRBuilder<> &builder, llvm::Value *matrix) {
    std::vector<llvm::Value *> elements;
    elements.reserve(16);
    for (unsigned int row = 0; row < 4; ++row) {
        for (unsigned int column = 0; column < 4; ++column) {
            elements.push_back(builder.CreateExtractValue(matrix, {column * 4 + row}, "mattransposeelt"));
        }
    }
    return BuildMatrix(builder, elements, "mattranspose");
}

llvm::Value *GenerateDeterminant3x3(llvm::IRBuilder<> &builder, const std::vector<llvm::Value *> &values) {
    const auto mul = [&](unsigned int left, unsigned int right) {
        return builder.CreateFMul(values[left], values[right], "matinv.mul");
    };
    const auto sub = [&](llvm::Value *left, llvm::Value *right) {
        return builder.CreateFSub(left, right, "matinv.sub");
    };

    llvm::Value *const cofactor0 = sub(mul(4, 8), mul(5, 7));
    llvm::Value *const cofactor1 = sub(mul(3, 8), mul(5, 6));
    llvm::Value *const cofactor2 = sub(mul(3, 7), mul(4, 6));

    llvm::Value *determinant = builder.CreateFMul(values[0], cofactor0, "matinv.term");
    determinant = builder.CreateFSub(determinant, builder.CreateFMul(values[1], cofactor1, "matinv.term"), "matinv.det");
    return builder.CreateFAdd(determinant, builder.CreateFMul(values[2], cofactor2, "matinv.term"), "matinv.det");
}

llvm::Value *GenerateMatrixInverse(llvm::IRBuilder<> &builder, llvm::Value *matrix) {
    std::vector<llvm::Value *> elements;
    elements.reserve(16);
    for (unsigned int index = 0; index < 16; ++index) {
        elements.push_back(builder.CreateExtractValue(matrix, {index}, "matinv.elt"));
    }

    std::vector<llvm::Value *> cofactors;
    cofactors.reserve(16);
    for (unsigned int excluded_row = 0; excluded_row < 4; ++excluded_row) {
        for (unsigned int excluded_column = 0; excluded_column < 4; ++excluded_column) {
            std::vector<llvm::Value *> minor;
            minor.reserve(9);
            for (unsigned int row = 0; row < 4; ++row) {
                if (row == excluded_row) {
                    continue;
                }
                for (unsigned int column = 0; column < 4; ++column) {
                    if (column != excluded_column) {
                        minor.push_back(elements[row * 4 + column]);
                    }
                }
            }

            llvm::Value *cofactor = GenerateDeterminant3x3(builder, minor);
            if ((excluded_row + excluded_column) % 2 != 0) {
                cofactor = builder.CreateFNeg(cofactor, "matinv.neg");
            }
            cofactors.push_back(cofactor);
        }
    }

    llvm::Value *determinant = llvm::ConstantFP::get(builder.getDoubleTy(), 0.0);
    for (unsigned int column = 0; column < 4; ++column) {
        determinant = builder.CreateFAdd(
            determinant,
            builder.CreateFMul(elements[column], cofactors[column], "matinv.detterm"),
            "matinv.det");
    }

    std::vector<llvm::Value *> inverse;
    inverse.reserve(16);
    for (unsigned int row = 0; row < 4; ++row) {
        for (unsigned int column = 0; column < 4; ++column) {
            inverse.push_back(builder.CreateFDiv(cofactors[column * 4 + row], determinant, "matinverse"));
        }
    }
    return BuildMatrix(builder, inverse, "matinverse");
}

llvm::Value *GenerateMatrixIdentity(llvm::IRBuilder<> &builder) {
    std::vector<llvm::Value *> elements;
    elements.reserve(16);
    for (unsigned int row = 0; row < 4; ++row) {
        for (unsigned int column = 0; column < 4; ++column) {
            elements.push_back(llvm::ConstantFP::get(builder.getDoubleTy(), row == column ? 1.0 : 0.0));
        }
    }
    return BuildMatrix(builder, elements, "matidentity");
}

} // namespace

bool OrlIntrinsicCodegen::TryGenerate(const std::string &name,
                                      const std::vector<llvm::Value *> &arguments,
                                      llvm::IRBuilder<> &builder,
                                      llvm::Module &module,
                                      llvm::Value **result,
                                      std::string *error) {
    const auto fail = [&](const char *message) {
        *result = nullptr;
        *error = message;
        return true;
    };
    const auto is_vector_argument = [](llvm::Value *value) {
        return IsFloatingVector(value->getType());
    };

    if (name == "global_id") {
        if (!arguments.empty()) {
            return fail("global_id requires no arguments");
        }
        llvm::FunctionCallee global_id = module.getOrInsertFunction(
            "__orl_global_id", llvm::FunctionType::get(builder.getInt64Ty(), false));
        *result = builder.CreateCall(global_id, {}, "global_id");
        return true;
    }
    if (name == "dot") {
        if (arguments.size() != 2 || !is_vector_argument(arguments[0]) ||
            arguments[0]->getType() != arguments[1]->getType()) {
            return fail("dot requires two floating-point vectors of the same size");
        }
        *result = GenerateDot(builder, arguments[0], arguments[1]);
        return true;
    }
    if (name == "cross") {
        if (arguments.size() != 2 || !IsFloatingVector(arguments[0]->getType()) ||
            !IsFloatingVector(arguments[1]->getType()) ||
            arguments[0]->getType() != arguments[1]->getType() ||
            llvm::cast<llvm::FixedVectorType>(arguments[0]->getType())->getNumElements() != 3) {
            return fail("cross requires two three-component floating-point vectors");
        }
        *result = GenerateCross(builder, arguments[0], arguments[1]);
        return true;
    }
    if (name == "length") {
        if (arguments.size() != 1 || !is_vector_argument(arguments[0])) {
            return fail("length requires exactly one floating-point vector");
        }
        *result = GenerateLength(builder, module, arguments[0]);
        return true;
    }
    if (name == "normalize") {
        if (arguments.size() != 1 || !is_vector_argument(arguments[0])) {
            return fail("normalize requires exactly one floating-point vector");
        }
        const unsigned int width = llvm::cast<llvm::FixedVectorType>(arguments[0]->getType())->getNumElements();
        llvm::Value *length = GenerateLength(builder, module, arguments[0]);
        *result = builder.CreateFDiv(arguments[0], builder.CreateVectorSplat(width, length), "vecnormalize");
        return true;
    }
    if (name == "clamp") {
        if (arguments.size() != 3) {
            return fail("clamp requires value, minimum, and maximum arguments");
        }
        llvm::Value *value = arguments[0];
        llvm::Value *minimum = arguments[1];
        llvm::Value *maximum = arguments[2];
        if (value->getType()->isDoubleTy() && minimum->getType()->isDoubleTy() && maximum->getType()->isDoubleTy()) {
            *result = builder.CreateSelect(builder.CreateFCmpOLT(value, minimum, "clamplow"),
                                           minimum,
                                           builder.CreateSelect(builder.CreateFCmpOGT(value, maximum, "clamphigh"),
                                                                maximum, value, "clampupper"),
                                           "clamp");
            return true;
        }
        if (!IsFloatingVector(value->getType())) {
            return fail("clamp requires a float or floating-point vector value");
        }
        const unsigned int width = llvm::cast<llvm::FixedVectorType>(value->getType())->getNumElements();
        if (minimum->getType()->isDoubleTy()) {
            minimum = builder.CreateVectorSplat(width, minimum, "clampminsplat");
        }
        if (maximum->getType()->isDoubleTy()) {
            maximum = builder.CreateVectorSplat(width, maximum, "clampmaxsplat");
        }
        if (minimum->getType() != value->getType() || maximum->getType() != value->getType()) {
            return fail("vector clamp bounds must be scalars or vectors of the same size");
        }
        *result = builder.CreateSelect(builder.CreateFCmpOLT(value, minimum, "clamplow"),
                                       minimum,
                                       builder.CreateSelect(builder.CreateFCmpOGT(value, maximum, "clamphigh"),
                                                            maximum, value, "clampupper"),
                                       "clamp");
        return true;
    }
    if (name == "lerp") {
        if (arguments.size() != 3) {
            return fail("lerp requires start, end, and factor arguments");
        }

        llvm::Value *start = arguments[0];
        llvm::Value *end = arguments[1];
        llvm::Value *factor = arguments[2];
        if (start->getType() != end->getType()) {
            return fail("lerp start and end arguments must have the same type");
        }
        if (start->getType()->isDoubleTy()) {
            if (!factor->getType()->isDoubleTy()) {
                return fail("scalar lerp requires a floating-point factor");
            }
            *result = GenerateLerp(builder, start, end, factor);
            return true;
        }
        if (!IsFloatingVector(start->getType())) {
            return fail("lerp requires floating-point scalar or vector arguments");
        }

        const unsigned int width = llvm::cast<llvm::FixedVectorType>(start->getType())->getNumElements();
        if (factor->getType()->isDoubleTy()) {
            factor = builder.CreateVectorSplat(width, factor, "lerpfactorsplat");
        }
        if (factor->getType() != start->getType()) {
            return fail("vector lerp factor must be a scalar or vector of the same size");
        }
        *result = GenerateLerp(builder, start, end, factor);
        return true;
    }
    if (name == "mat_identity") {
        if (!arguments.empty()) {
            return fail("mat_identity requires no arguments");
        }
        *result = GenerateMatrixIdentity(builder);
        return true;
    }
    if (name == "mat_transpose") {
        if (arguments.size() != 1 || !IsMatrix4x4(arguments[0]->getType())) {
            return fail("mat_transpose requires exactly one 4x4 matrix");
        }
        *result = GenerateMatrixTranspose(builder, arguments[0]);
        return true;
    }
    if (name == "mat_inverse") {
        if (arguments.size() != 1 || !IsMatrix4x4(arguments[0]->getType())) {
            return fail("mat_inverse requires exactly one 4x4 matrix");
        }
        *result = GenerateMatrixInverse(builder, arguments[0]);
        return true;
    }
    if (name == "mat_mul") {
        if (arguments.size() != 2 || !IsMatrix4x4(arguments[0]->getType())) {
            return fail("mat_mul requires a 4x4 matrix as its first argument");
        }
        if (IsMatrix4x4(arguments[1]->getType())) {
            *result = GenerateMatrixMultiply(builder, arguments[0], arguments[1]);
            return true;
        }
        if (IsFloatingVector(arguments[1]->getType()) &&
            llvm::cast<llvm::FixedVectorType>(arguments[1]->getType())->getNumElements() == 3) {
            *result = GenerateMatrixVectorMultiply(builder, arguments[0], arguments[1]);
            return true;
        }
        return fail("mat_mul requires a 4x4 matrix or three-component vector as its second argument");
    }

    return false;
}

} // namespace orlcomp
