//
// Copyright (C) 2014-2016 LunarG, Inc.
// Copyright (C) 2015-2020 Google, Inc.
// Copyright (C) 2017 ARM Limited.
// Modifications Copyright (C) 2020 Advanced Micro Devices, Inc. All rights reserved.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
//    Neither the name of 3Dlabs Inc. Ltd. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//
// Visit the nodes in the glslang intermediate tree representation to
// translate them to SPIR-V.
//

#include "spirv.hpp"
#include "GlslangToSpv.h"
#include "SpvBuilder.h"
namespace spv {
    #include "GLSL.std.450.h"
    #include "GLSL.ext.KHR.h"
    #include "GLSL.ext.EXT.h"
    #include "GLSL.ext.AMD.h"
    #include "GLSL.ext.NV.h"
    #include "NonSemanticDebugPrintf.h"
}

// Glslang includes
#include "../glslang/MachineIndependent/localintermediate.h"
#include "../glslang/MachineIndependent/SymbolTable.h"
#include "../glslang/Include/Common.h"

// Build-time generated includes
#include "glslang/build_info.h"

#include <fstream>
#include <iomanip>
#include <list>
#include <map>
#include <stack>
#include <string>
#include <vector>

namespace {

namespace {
class SpecConstantOpModeGuard {
public:
    SpecConstantOpModeGuard(spv::Builder* builder)
        : builder_(builder) {
        previous_flag_ = builder->isInSpecConstCodeGenMode();
    }
    ~SpecConstantOpModeGuard() {
        previous_flag_ ? builder_->setToSpecConstCodeGenMode()
                       : builder_->setToNormalCodeGenMode();
    }
    void turnOnSpecConstantOpMode() {
        builder_->setToSpecConstCodeGenMode();
    }

private:
    spv::Builder* builder_;
    bool previous_flag_;
};

struct OpDecorations {
    public:
        OpDecorations(spv::Decoration precision, spv::Decoration noContraction, spv::Decoration nonUniform) :
            precision(precision)
#ifndef GLSLANG_WEB
            ,
            noContraction(noContraction),
            nonUniform(nonUniform)
#endif
        { }

    spv::Decoration precision;

#ifdef GLSLANG_WEB
        void addNoContraction(spv::Builder&, spv::Id) const { }
        void addNonUniform(spv::Builder&, spv::Id) const { }
#else
        void addNoContraction(spv::Builder& builder, spv::Id t) { builder.addDecoration(t, noContraction); }
        void addNonUniform(spv::Builder& builder, spv::Id t)  { builder.addDecoration(t, nonUniform); }
    protected:
        spv::Decoration noContraction;
        spv::Decoration nonUniform;
#endif

};

} // namespace

//
// The main holder of information for translating glslang to SPIR-V.
//
// Derives from the AST walking base class.
//
class TGlslangToSpvTraverser : public qglslang::TIntermTraverser {
public:
    TGlslangToSpvTraverser(unsigned int spvVersion, const qglslang::TIntermediate*, spv::SpvBuildLogger* logger,
        qglslang::SpvOptions& options);
    virtual ~TGlslangToSpvTraverser() { }

    bool visitAggregate(qglslang::TVisit, qglslang::TIntermAggregate*);
    bool visitBinary(qglslang::TVisit, qglslang::TIntermBinary*);
    void visitConstantUnion(qglslang::TIntermConstantUnion*);
    bool visitSelection(qglslang::TVisit, qglslang::TIntermSelection*);
    bool visitSwitch(qglslang::TVisit, qglslang::TIntermSwitch*);
    void visitSymbol(qglslang::TIntermSymbol* symbol);
    bool visitUnary(qglslang::TVisit, qglslang::TIntermUnary*);
    bool visitLoop(qglslang::TVisit, qglslang::TIntermLoop*);
    bool visitBranch(qglslang::TVisit visit, qglslang::TIntermBranch*);

    void finishSpv();
    void dumpSpv(std::vector<unsigned int>& out);

protected:
    TGlslangToSpvTraverser(TGlslangToSpvTraverser&);
    TGlslangToSpvTraverser& operator=(TGlslangToSpvTraverser&);

    spv::Decoration TranslateInterpolationDecoration(const qglslang::TQualifier& qualifier);
    spv::Decoration TranslateAuxiliaryStorageDecoration(const qglslang::TQualifier& qualifier);
    spv::Decoration TranslateNonUniformDecoration(const qglslang::TQualifier& qualifier);
    spv::Decoration TranslateNonUniformDecoration(const spv::Builder::AccessChain::CoherentFlags& coherentFlags);
    spv::Builder::AccessChain::CoherentFlags TranslateCoherent(const qglslang::TType& type);
    spv::MemoryAccessMask TranslateMemoryAccess(const spv::Builder::AccessChain::CoherentFlags &coherentFlags);
    spv::ImageOperandsMask TranslateImageOperands(const spv::Builder::AccessChain::CoherentFlags &coherentFlags);
    spv::Scope TranslateMemoryScope(const spv::Builder::AccessChain::CoherentFlags &coherentFlags);
    spv::BuiltIn TranslateBuiltInDecoration(qglslang::TBuiltInVariable, bool memberDeclaration);
    spv::ImageFormat TranslateImageFormat(const qglslang::TType& type);
    spv::SelectionControlMask TranslateSelectionControl(const qglslang::TIntermSelection&) const;
    spv::SelectionControlMask TranslateSwitchControl(const qglslang::TIntermSwitch&) const;
    spv::LoopControlMask TranslateLoopControl(const qglslang::TIntermLoop&, std::vector<unsigned int>& operands) const;
    spv::StorageClass TranslateStorageClass(const qglslang::TType&);
    void TranslateLiterals(const qglslang::TVector<const qglslang::TIntermConstantUnion*>&, std::vector<unsigned>&) const;
    void addIndirectionIndexCapabilities(const qglslang::TType& baseType, const qglslang::TType& indexType);
    spv::Id createSpvVariable(const qglslang::TIntermSymbol*, spv::Id forcedType);
    spv::Id getSampledType(const qglslang::TSampler&);
    spv::Id getInvertedSwizzleType(const qglslang::TIntermTyped&);
    spv::Id createInvertedSwizzle(spv::Decoration precision, const qglslang::TIntermTyped&, spv::Id parentResult);
    void convertSwizzle(const qglslang::TIntermAggregate&, std::vector<unsigned>& swizzle);
    spv::Id convertGlslangToSpvType(const qglslang::TType& type, bool forwardReferenceOnly = false);
    spv::Id convertGlslangToSpvType(const qglslang::TType& type, qglslang::TLayoutPacking, const qglslang::TQualifier&,
        bool lastBufferBlockMember, bool forwardReferenceOnly = false);
    bool filterMember(const qglslang::TType& member);
    spv::Id convertGlslangStructToSpvType(const qglslang::TType&, const qglslang::TTypeList* glslangStruct,
                                          qglslang::TLayoutPacking, const qglslang::TQualifier&);
    void decorateStructType(const qglslang::TType&, const qglslang::TTypeList* glslangStruct, qglslang::TLayoutPacking,
                            const qglslang::TQualifier&, spv::Id);
    spv::Id makeArraySizeId(const qglslang::TArraySizes&, int dim);
    spv::Id accessChainLoad(const qglslang::TType& type);
    void    accessChainStore(const qglslang::TType& type, spv::Id rvalue);
    void multiTypeStore(const qglslang::TType&, spv::Id rValue);
    spv::Id convertLoadedBoolInUniformToUint(const qglslang::TType& type, spv::Id nominalTypeId, spv::Id loadedId);
    qglslang::TLayoutPacking getExplicitLayout(const qglslang::TType& type) const;
    int getArrayStride(const qglslang::TType& arrayType, qglslang::TLayoutPacking, qglslang::TLayoutMatrix);
    int getMatrixStride(const qglslang::TType& matrixType, qglslang::TLayoutPacking, qglslang::TLayoutMatrix);
    void updateMemberOffset(const qglslang::TType& structType, const qglslang::TType& memberType, int& currentOffset,
                            int& nextOffset, qglslang::TLayoutPacking, qglslang::TLayoutMatrix);
    void declareUseOfStructMember(const qglslang::TTypeList& members, int glslangMember);

    bool isShaderEntryPoint(const qglslang::TIntermAggregate* node);
    bool writableParam(qglslang::TStorageQualifier) const;
    bool originalParam(qglslang::TStorageQualifier, const qglslang::TType&, bool implicitThisParam);
    void makeFunctions(const qglslang::TIntermSequence&);
    void makeGlobalInitializers(const qglslang::TIntermSequence&);
    void collectRayTracingLinkerObjects();
    void visitFunctions(const qglslang::TIntermSequence&);
    void handleFunctionEntry(const qglslang::TIntermAggregate* node);
    void translateArguments(const qglslang::TIntermAggregate& node, std::vector<spv::Id>& arguments,
        spv::Builder::AccessChain::CoherentFlags &lvalueCoherentFlags);
    void translateArguments(qglslang::TIntermUnary& node, std::vector<spv::Id>& arguments);
    spv::Id createImageTextureFunctionCall(qglslang::TIntermOperator* node);
    spv::Id handleUserFunctionCall(const qglslang::TIntermAggregate*);

    spv::Id createBinaryOperation(qglslang::TOperator op, OpDecorations&, spv::Id typeId, spv::Id left, spv::Id right,
                                  qglslang::TBasicType typeProxy, bool reduceComparison = true);
    spv::Id createBinaryMatrixOperation(spv::Op, OpDecorations&, spv::Id typeId, spv::Id left, spv::Id right);
    spv::Id createUnaryOperation(qglslang::TOperator op, OpDecorations&, spv::Id typeId, spv::Id operand,
                                 qglslang::TBasicType typeProxy,
                                 const spv::Builder::AccessChain::CoherentFlags &lvalueCoherentFlags);
    spv::Id createUnaryMatrixOperation(spv::Op op, OpDecorations&, spv::Id typeId, spv::Id operand,
                                       qglslang::TBasicType typeProxy);
    spv::Id createConversion(qglslang::TOperator op, OpDecorations&, spv::Id destTypeId, spv::Id operand,
                             qglslang::TBasicType typeProxy);
    spv::Id createIntWidthConversion(qglslang::TOperator op, spv::Id operand, int vectorSize);
    spv::Id makeSmearedConstant(spv::Id constant, int vectorSize);
    spv::Id createAtomicOperation(qglslang::TOperator op, spv::Decoration precision, spv::Id typeId,
        std::vector<spv::Id>& operands, qglslang::TBasicType typeProxy,
        const spv::Builder::AccessChain::CoherentFlags &lvalueCoherentFlags);
    spv::Id createInvocationsOperation(qglslang::TOperator op, spv::Id typeId, std::vector<spv::Id>& operands,
        qglslang::TBasicType typeProxy);
    spv::Id CreateInvocationsVectorOperation(spv::Op op, spv::GroupOperation groupOperation,
        spv::Id typeId, std::vector<spv::Id>& operands);
    spv::Id createSubgroupOperation(qglslang::TOperator op, spv::Id typeId, std::vector<spv::Id>& operands,
        qglslang::TBasicType typeProxy);
    spv::Id createMiscOperation(qglslang::TOperator op, spv::Decoration precision, spv::Id typeId,
        std::vector<spv::Id>& operands, qglslang::TBasicType typeProxy);
    spv::Id createNoArgOperation(qglslang::TOperator op, spv::Decoration precision, spv::Id typeId);
    spv::Id getSymbolId(const qglslang::TIntermSymbol* node);
    void addMeshNVDecoration(spv::Id id, int member, const qglslang::TQualifier & qualifier);
    spv::Id createSpvConstant(const qglslang::TIntermTyped&);
    spv::Id createSpvConstantFromConstUnionArray(const qglslang::TType& type, const qglslang::TConstUnionArray&,
        int& nextConst, bool specConstant);
    bool isTrivialLeaf(const qglslang::TIntermTyped* node);
    bool isTrivial(const qglslang::TIntermTyped* node);
    spv::Id createShortCircuit(qglslang::TOperator, qglslang::TIntermTyped& left, qglslang::TIntermTyped& right);
    spv::Id getExtBuiltins(const char* name);
    std::pair<spv::Id, spv::Id> getForcedType(qglslang::TBuiltInVariable builtIn, const qglslang::TType&);
    spv::Id translateForcedType(spv::Id object);
    spv::Id createCompositeConstruct(spv::Id typeId, std::vector<spv::Id> constituents);

    qglslang::SpvOptions& options;
    spv::Function* shaderEntry;
    spv::Function* currentFunction;
    spv::Instruction* entryPoint;
    int sequenceDepth;

    spv::SpvBuildLogger* logger;

    // There is a 1:1 mapping between a spv builder and a module; this is thread safe
    spv::Builder builder;
    bool inEntryPoint;
    bool entryPointTerminated;
    bool linkageOnly;                  // true when visiting the set of objects in the AST present only for
                                       // establishing interface, whether or not they were statically used
    std::set<spv::Id> iOSet;           // all input/output variables from either static use or declaration of interface
    const qglslang::TIntermediate* glslangIntermediate;
    bool nanMinMaxClamp;               // true if use NMin/NMax/NClamp instead of FMin/FMax/FClamp
    spv::Id stdBuiltins;
    spv::Id nonSemanticDebugPrintf;
    std::unordered_map<std::string, spv::Id> extBuiltinMap;

    std::unordered_map<long long, spv::Id> symbolValues;
    std::unordered_map<uint32_t, spv::Id> builtInVariableIds;
    std::unordered_set<long long> rValueParameters;  // set of formal function parameters passed as rValues,
                                               // rather than a pointer
    std::unordered_map<std::string, spv::Function*> functionMap;
    std::unordered_map<const qglslang::TTypeList*, spv::Id> structMap[qglslang::ElpCount][qglslang::ElmCount];
    // for mapping glslang block indices to spv indices (e.g., due to hidden members):
    std::unordered_map<long long, std::vector<int>> memberRemapper;
    // for mapping glslang symbol struct to symbol Id
    std::unordered_map<const qglslang::TTypeList*, long long> glslangTypeToIdMap;
    std::stack<bool> breakForLoop;  // false means break for switch
    std::unordered_map<std::string, const qglslang::TIntermSymbol*> counterOriginator;
    // Map pointee types for EbtReference to their forward pointers
    std::map<const qglslang::TType *, spv::Id> forwardPointers;
    // Type forcing, for when SPIR-V wants a different type than the AST,
    // requiring local translation to and from SPIR-V type on every access.
    // Maps <builtin-variable-id -> AST-required-type-id>
    std::unordered_map<spv::Id, spv::Id> forceType;

    // Used later for generating OpTraceKHR/OpExecuteCallableKHR
    std::unordered_map<unsigned int, qglslang::TIntermSymbol *> locationToSymbol[2];
};

//
// Helper functions for translating glslang representations to SPIR-V enumerants.
//

// Translate glslang profile to SPIR-V source language.
spv::SourceLanguage TranslateSourceLanguage(qglslang::EShSource source, EProfile profile)
{
#ifdef GLSLANG_WEB
    return spv::SourceLanguageESSL;
#elif defined(GLSLANG_ANGLE)
    return spv::SourceLanguageGLSL;
#endif

    switch (source) {
    case qglslang::EShSourceGlsl:
        switch (profile) {
        case ENoProfile:
        case ECoreProfile:
        case ECompatibilityProfile:
            return spv::SourceLanguageGLSL;
        case EEsProfile:
            return spv::SourceLanguageESSL;
        default:
            return spv::SourceLanguageUnknown;
        }
    case qglslang::EShSourceHlsl:
        return spv::SourceLanguageHLSL;
    default:
        return spv::SourceLanguageUnknown;
    }
}

// Translate glslang language (stage) to SPIR-V execution model.
spv::ExecutionModel TranslateExecutionModel(EShLanguage stage)
{
    switch (stage) {
    case EShLangVertex:           return spv::ExecutionModelVertex;
    case EShLangFragment:         return spv::ExecutionModelFragment;
    case EShLangCompute:          return spv::ExecutionModelGLCompute;
#ifndef GLSLANG_WEB
    case EShLangTessControl:      return spv::ExecutionModelTessellationControl;
    case EShLangTessEvaluation:   return spv::ExecutionModelTessellationEvaluation;
    case EShLangGeometry:         return spv::ExecutionModelGeometry;
    case EShLangRayGen:           return spv::ExecutionModelRayGenerationKHR;
    case EShLangIntersect:        return spv::ExecutionModelIntersectionKHR;
    case EShLangAnyHit:           return spv::ExecutionModelAnyHitKHR;
    case EShLangClosestHit:       return spv::ExecutionModelClosestHitKHR;
    case EShLangMiss:             return spv::ExecutionModelMissKHR;
    case EShLangCallable:         return spv::ExecutionModelCallableKHR;
    case EShLangTaskNV:           return spv::ExecutionModelTaskNV;
    case EShLangMeshNV:           return spv::ExecutionModelMeshNV;
#endif
    default:
        assert(0);
        return spv::ExecutionModelFragment;
    }
}

// Translate glslang sampler type to SPIR-V dimensionality.
spv::Dim TranslateDimensionality(const qglslang::TSampler& sampler)
{
    switch (sampler.dim) {
    case qglslang::Esd1D:      return spv::Dim1D;
    case qglslang::Esd2D:      return spv::Dim2D;
    case qglslang::Esd3D:      return spv::Dim3D;
    case qglslang::EsdCube:    return spv::DimCube;
    case qglslang::EsdRect:    return spv::DimRect;
    case qglslang::EsdBuffer:  return spv::DimBuffer;
    case qglslang::EsdSubpass: return spv::DimSubpassData;
    default:
        assert(0);
        return spv::Dim2D;
    }
}

// Translate glslang precision to SPIR-V precision decorations.
spv::Decoration TranslatePrecisionDecoration(qglslang::TPrecisionQualifier glslangPrecision)
{
    switch (glslangPrecision) {
    case qglslang::EpqLow:    return spv::DecorationRelaxedPrecision;
    case qglslang::EpqMedium: return spv::DecorationRelaxedPrecision;
    default:
        return spv::NoPrecision;
    }
}

// Translate glslang type to SPIR-V precision decorations.
spv::Decoration TranslatePrecisionDecoration(const qglslang::TType& type)
{
    return TranslatePrecisionDecoration(type.getQualifier().precision);
}

// Translate glslang type to SPIR-V block decorations.
spv::Decoration TranslateBlockDecoration(const qglslang::TType& type, bool useStorageBuffer)
{
    if (type.getBasicType() == qglslang::EbtBlock) {
        switch (type.getQualifier().storage) {
        case qglslang::EvqUniform:      return spv::DecorationBlock;
        case qglslang::EvqBuffer:       return useStorageBuffer ? spv::DecorationBlock : spv::DecorationBufferBlock;
        case qglslang::EvqVaryingIn:    return spv::DecorationBlock;
        case qglslang::EvqVaryingOut:   return spv::DecorationBlock;
        case qglslang::EvqShared:       return spv::DecorationBlock;
#ifndef GLSLANG_WEB
        case qglslang::EvqPayload:      return spv::DecorationBlock;
        case qglslang::EvqPayloadIn:    return spv::DecorationBlock;
        case qglslang::EvqHitAttr:      return spv::DecorationBlock;
        case qglslang::EvqCallableData:   return spv::DecorationBlock;
        case qglslang::EvqCallableDataIn: return spv::DecorationBlock;
#endif
        default:
            assert(0);
            break;
        }
    }

    return spv::DecorationMax;
}

// Translate glslang type to SPIR-V memory decorations.
void TranslateMemoryDecoration(const qglslang::TQualifier& qualifier, std::vector<spv::Decoration>& memory,
    bool useVulkanMemoryModel)
{
    if (!useVulkanMemoryModel) {
        if (qualifier.isCoherent())
            memory.push_back(spv::DecorationCoherent);
        if (qualifier.isVolatile()) {
            memory.push_back(spv::DecorationVolatile);
            memory.push_back(spv::DecorationCoherent);
        }
    }
    if (qualifier.isRestrict())
        memory.push_back(spv::DecorationRestrict);
    if (qualifier.isReadOnly())
        memory.push_back(spv::DecorationNonWritable);
    if (qualifier.isWriteOnly())
       memory.push_back(spv::DecorationNonReadable);
}

// Translate glslang type to SPIR-V layout decorations.
spv::Decoration TranslateLayoutDecoration(const qglslang::TType& type, qglslang::TLayoutMatrix matrixLayout)
{
    if (type.isMatrix()) {
        switch (matrixLayout) {
        case qglslang::ElmRowMajor:
            return spv::DecorationRowMajor;
        case qglslang::ElmColumnMajor:
            return spv::DecorationColMajor;
        default:
            // opaque layouts don't need a majorness
            return spv::DecorationMax;
        }
    } else {
        switch (type.getBasicType()) {
        default:
            return spv::DecorationMax;
            break;
        case qglslang::EbtBlock:
            switch (type.getQualifier().storage) {
            case qglslang::EvqShared:
            case qglslang::EvqUniform:
            case qglslang::EvqBuffer:
                switch (type.getQualifier().layoutPacking) {
                case qglslang::ElpShared:  return spv::DecorationGLSLShared;
                case qglslang::ElpPacked:  return spv::DecorationGLSLPacked;
                default:
                    return spv::DecorationMax;
                }
            case qglslang::EvqVaryingIn:
            case qglslang::EvqVaryingOut:
                if (type.getQualifier().isTaskMemory()) {
                    switch (type.getQualifier().layoutPacking) {
                    case qglslang::ElpShared:  return spv::DecorationGLSLShared;
                    case qglslang::ElpPacked:  return spv::DecorationGLSLPacked;
                    default: break;
                    }
                } else {
                    assert(type.getQualifier().layoutPacking == qglslang::ElpNone);
                }
                return spv::DecorationMax;
#ifndef GLSLANG_WEB
            case qglslang::EvqPayload:
            case qglslang::EvqPayloadIn:
            case qglslang::EvqHitAttr:
            case qglslang::EvqCallableData:
            case qglslang::EvqCallableDataIn:
                return spv::DecorationMax;
#endif
            default:
                assert(0);
                return spv::DecorationMax;
            }
        }
    }
}

// Translate glslang type to SPIR-V interpolation decorations.
// Returns spv::DecorationMax when no decoration
// should be applied.
spv::Decoration TGlslangToSpvTraverser::TranslateInterpolationDecoration(const qglslang::TQualifier& qualifier)
{
    if (qualifier.smooth)
        // Smooth decoration doesn't exist in SPIR-V 1.0
        return spv::DecorationMax;
    else if (qualifier.isNonPerspective())
        return spv::DecorationNoPerspective;
    else if (qualifier.flat)
        return spv::DecorationFlat;
    else if (qualifier.isExplicitInterpolation()) {
        builder.addExtension(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        return spv::DecorationExplicitInterpAMD;
    }
    else
        return spv::DecorationMax;
}

// Translate glslang type to SPIR-V auxiliary storage decorations.
// Returns spv::DecorationMax when no decoration
// should be applied.
spv::Decoration TGlslangToSpvTraverser::TranslateAuxiliaryStorageDecoration(const qglslang::TQualifier& qualifier)
{
    if (qualifier.centroid)
        return spv::DecorationCentroid;
#ifndef GLSLANG_WEB
    else if (qualifier.patch)
        return spv::DecorationPatch;
    else if (qualifier.sample) {
        builder.addCapability(spv::CapabilitySampleRateShading);
        return spv::DecorationSample;
    }
#endif

    return spv::DecorationMax;
}

// If glslang type is invariant, return SPIR-V invariant decoration.
spv::Decoration TranslateInvariantDecoration(const qglslang::TQualifier& qualifier)
{
    if (qualifier.invariant)
        return spv::DecorationInvariant;
    else
        return spv::DecorationMax;
}

// If glslang type is noContraction, return SPIR-V NoContraction decoration.
spv::Decoration TranslateNoContractionDecoration(const qglslang::TQualifier& qualifier)
{
#ifndef GLSLANG_WEB
    if (qualifier.isNoContraction())
        return spv::DecorationNoContraction;
    else
#endif
        return spv::DecorationMax;
}

// If glslang type is nonUniform, return SPIR-V NonUniform decoration.
spv::Decoration TGlslangToSpvTraverser::TranslateNonUniformDecoration(const qglslang::TQualifier& qualifier)
{
#ifndef GLSLANG_WEB
    if (qualifier.isNonUniform()) {
        builder.addIncorporatedExtension("SPV_EXT_descriptor_indexing", spv::Spv_1_5);
        builder.addCapability(spv::CapabilityShaderNonUniformEXT);
        return spv::DecorationNonUniformEXT;
    } else
#endif
        return spv::DecorationMax;
}

// If lvalue flags contains nonUniform, return SPIR-V NonUniform decoration.
spv::Decoration TGlslangToSpvTraverser::TranslateNonUniformDecoration(
    const spv::Builder::AccessChain::CoherentFlags& coherentFlags)
{
#ifndef GLSLANG_WEB
    if (coherentFlags.isNonUniform()) {
        builder.addIncorporatedExtension("SPV_EXT_descriptor_indexing", spv::Spv_1_5);
        builder.addCapability(spv::CapabilityShaderNonUniformEXT);
        return spv::DecorationNonUniformEXT;
    } else
#endif
        return spv::DecorationMax;
}

spv::MemoryAccessMask TGlslangToSpvTraverser::TranslateMemoryAccess(
    const spv::Builder::AccessChain::CoherentFlags &coherentFlags)
{
    spv::MemoryAccessMask mask = spv::MemoryAccessMaskNone;

#ifndef GLSLANG_WEB
    if (!glslangIntermediate->usingVulkanMemoryModel() || coherentFlags.isImage)
        return mask;

    if (coherentFlags.isVolatile() || coherentFlags.anyCoherent()) {
        mask = mask | spv::MemoryAccessMakePointerAvailableKHRMask |
                      spv::MemoryAccessMakePointerVisibleKHRMask;
    }

    if (coherentFlags.nonprivate) {
        mask = mask | spv::MemoryAccessNonPrivatePointerKHRMask;
    }
    if (coherentFlags.volatil) {
        mask = mask | spv::MemoryAccessVolatileMask;
    }
    if (mask != spv::MemoryAccessMaskNone) {
        builder.addCapability(spv::CapabilityVulkanMemoryModelKHR);
    }
#endif

    return mask;
}

spv::ImageOperandsMask TGlslangToSpvTraverser::TranslateImageOperands(
    const spv::Builder::AccessChain::CoherentFlags &coherentFlags)
{
    spv::ImageOperandsMask mask = spv::ImageOperandsMaskNone;

#ifndef GLSLANG_WEB
    if (!glslangIntermediate->usingVulkanMemoryModel())
        return mask;

    if (coherentFlags.volatil ||
        coherentFlags.anyCoherent()) {
        mask = mask | spv::ImageOperandsMakeTexelAvailableKHRMask |
                      spv::ImageOperandsMakeTexelVisibleKHRMask;
    }
    if (coherentFlags.nonprivate) {
        mask = mask | spv::ImageOperandsNonPrivateTexelKHRMask;
    }
    if (coherentFlags.volatil) {
        mask = mask | spv::ImageOperandsVolatileTexelKHRMask;
    }
    if (mask != spv::ImageOperandsMaskNone) {
        builder.addCapability(spv::CapabilityVulkanMemoryModelKHR);
    }
#endif

    return mask;
}

spv::Builder::AccessChain::CoherentFlags TGlslangToSpvTraverser::TranslateCoherent(const qglslang::TType& type)
{
    spv::Builder::AccessChain::CoherentFlags flags = {};
#ifndef GLSLANG_WEB
    flags.coherent = type.getQualifier().coherent;
    flags.devicecoherent = type.getQualifier().devicecoherent;
    flags.queuefamilycoherent = type.getQualifier().queuefamilycoherent;
    // shared variables are implicitly workgroupcoherent in GLSL.
    flags.workgroupcoherent = type.getQualifier().workgroupcoherent ||
                              type.getQualifier().storage == qglslang::EvqShared;
    flags.subgroupcoherent = type.getQualifier().subgroupcoherent;
    flags.shadercallcoherent = type.getQualifier().shadercallcoherent;
    flags.volatil = type.getQualifier().volatil;
    // *coherent variables are implicitly nonprivate in GLSL
    flags.nonprivate = type.getQualifier().nonprivate ||
                       flags.anyCoherent() ||
                       flags.volatil;
    flags.isImage = type.getBasicType() == qglslang::EbtSampler;
#endif
    flags.nonUniform = type.getQualifier().nonUniform;
    return flags;
}

spv::Scope TGlslangToSpvTraverser::TranslateMemoryScope(
    const spv::Builder::AccessChain::CoherentFlags &coherentFlags)
{
    spv::Scope scope = spv::ScopeMax;

#ifndef GLSLANG_WEB
    if (coherentFlags.volatil || coherentFlags.coherent) {
        // coherent defaults to Device scope in the old model, QueueFamilyKHR scope in the new model
        scope = glslangIntermediate->usingVulkanMemoryModel() ? spv::ScopeQueueFamilyKHR : spv::ScopeDevice;
    } else if (coherentFlags.devicecoherent) {
        scope = spv::ScopeDevice;
    } else if (coherentFlags.queuefamilycoherent) {
        scope = spv::ScopeQueueFamilyKHR;
    } else if (coherentFlags.workgroupcoherent) {
        scope = spv::ScopeWorkgroup;
    } else if (coherentFlags.subgroupcoherent) {
        scope = spv::ScopeSubgroup;
    } else if (coherentFlags.shadercallcoherent) {
        scope = spv::ScopeShaderCallKHR;
    }
    if (glslangIntermediate->usingVulkanMemoryModel() && scope == spv::ScopeDevice) {
        builder.addCapability(spv::CapabilityVulkanMemoryModelDeviceScopeKHR);
    }
#endif

    return scope;
}

// Translate a glslang built-in variable to a SPIR-V built in decoration.  Also generate
// associated capabilities when required.  For some built-in variables, a capability
// is generated only when using the variable in an executable instruction, but not when
// just declaring a struct member variable with it.  This is true for PointSize,
// ClipDistance, and CullDistance.
spv::BuiltIn TGlslangToSpvTraverser::TranslateBuiltInDecoration(qglslang::TBuiltInVariable builtIn,
    bool memberDeclaration)
{
    switch (builtIn) {
    case qglslang::EbvPointSize:
#ifndef GLSLANG_WEB
        // Defer adding the capability until the built-in is actually used.
        if (! memberDeclaration) {
            switch (glslangIntermediate->getStage()) {
            case EShLangGeometry:
                builder.addCapability(spv::CapabilityGeometryPointSize);
                break;
            case EShLangTessControl:
            case EShLangTessEvaluation:
                builder.addCapability(spv::CapabilityTessellationPointSize);
                break;
            default:
                break;
            }
        }
#endif
        return spv::BuiltInPointSize;

    case qglslang::EbvPosition:             return spv::BuiltInPosition;
    case qglslang::EbvVertexId:             return spv::BuiltInVertexId;
    case qglslang::EbvInstanceId:           return spv::BuiltInInstanceId;
    case qglslang::EbvVertexIndex:          return spv::BuiltInVertexIndex;
    case qglslang::EbvInstanceIndex:        return spv::BuiltInInstanceIndex;

    case qglslang::EbvFragCoord:            return spv::BuiltInFragCoord;
    case qglslang::EbvPointCoord:           return spv::BuiltInPointCoord;
    case qglslang::EbvFace:                 return spv::BuiltInFrontFacing;
    case qglslang::EbvFragDepth:            return spv::BuiltInFragDepth;

    case qglslang::EbvNumWorkGroups:        return spv::BuiltInNumWorkgroups;
    case qglslang::EbvWorkGroupSize:        return spv::BuiltInWorkgroupSize;
    case qglslang::EbvWorkGroupId:          return spv::BuiltInWorkgroupId;
    case qglslang::EbvLocalInvocationId:    return spv::BuiltInLocalInvocationId;
    case qglslang::EbvLocalInvocationIndex: return spv::BuiltInLocalInvocationIndex;
    case qglslang::EbvGlobalInvocationId:   return spv::BuiltInGlobalInvocationId;

#ifndef GLSLANG_WEB
    // These *Distance capabilities logically belong here, but if the member is declared and
    // then never used, consumers of SPIR-V prefer the capability not be declared.
    // They are now generated when used, rather than here when declared.
    // Potentially, the specification should be more clear what the minimum
    // use needed is to trigger the capability.
    //
    case qglslang::EbvClipDistance:
        if (!memberDeclaration)
            builder.addCapability(spv::CapabilityClipDistance);
        return spv::BuiltInClipDistance;

    case qglslang::EbvCullDistance:
        if (!memberDeclaration)
            builder.addCapability(spv::CapabilityCullDistance);
        return spv::BuiltInCullDistance;

    case qglslang::EbvViewportIndex:
        if (glslangIntermediate->getStage() == EShLangGeometry ||
            glslangIntermediate->getStage() == EShLangFragment) {
            builder.addCapability(spv::CapabilityMultiViewport);
        }
        if (glslangIntermediate->getStage() == EShLangVertex ||
            glslangIntermediate->getStage() == EShLangTessControl ||
            glslangIntermediate->getStage() == EShLangTessEvaluation) {

            if (builder.getSpvVersion() < spv::Spv_1_5) {
                builder.addIncorporatedExtension(spv::E_SPV_EXT_shader_viewport_index_layer, spv::Spv_1_5);
                builder.addCapability(spv::CapabilityShaderViewportIndexLayerEXT);
            }
            else
                builder.addCapability(spv::CapabilityShaderViewportIndex);
        }
        return spv::BuiltInViewportIndex;

    case qglslang::EbvSampleId:
        builder.addCapability(spv::CapabilitySampleRateShading);
        return spv::BuiltInSampleId;

    case qglslang::EbvSamplePosition:
        builder.addCapability(spv::CapabilitySampleRateShading);
        return spv::BuiltInSamplePosition;

    case qglslang::EbvSampleMask:
        return spv::BuiltInSampleMask;

    case qglslang::EbvLayer:
        if (glslangIntermediate->getStage() == EShLangMeshNV) {
            return spv::BuiltInLayer;
        }
        if (glslangIntermediate->getStage() == EShLangGeometry ||
            glslangIntermediate->getStage() == EShLangFragment) {
            builder.addCapability(spv::CapabilityGeometry);
        }
        if (glslangIntermediate->getStage() == EShLangVertex ||
            glslangIntermediate->getStage() == EShLangTessControl ||
            glslangIntermediate->getStage() == EShLangTessEvaluation) {

            if (builder.getSpvVersion() < spv::Spv_1_5) {
                builder.addIncorporatedExtension(spv::E_SPV_EXT_shader_viewport_index_layer, spv::Spv_1_5);
                builder.addCapability(spv::CapabilityShaderViewportIndexLayerEXT);
            } else
                builder.addCapability(spv::CapabilityShaderLayer);
        }
        return spv::BuiltInLayer;

    case qglslang::EbvBaseVertex:
        builder.addIncorporatedExtension(spv::E_SPV_KHR_shader_draw_parameters, spv::Spv_1_3);
        builder.addCapability(spv::CapabilityDrawParameters);
        return spv::BuiltInBaseVertex;

    case qglslang::EbvBaseInstance:
        builder.addIncorporatedExtension(spv::E_SPV_KHR_shader_draw_parameters, spv::Spv_1_3);
        builder.addCapability(spv::CapabilityDrawParameters);
        return spv::BuiltInBaseInstance;

    case qglslang::EbvDrawId:
        builder.addIncorporatedExtension(spv::E_SPV_KHR_shader_draw_parameters, spv::Spv_1_3);
        builder.addCapability(spv::CapabilityDrawParameters);
        return spv::BuiltInDrawIndex;

    case qglslang::EbvPrimitiveId:
        if (glslangIntermediate->getStage() == EShLangFragment)
            builder.addCapability(spv::CapabilityGeometry);
        return spv::BuiltInPrimitiveId;

    case qglslang::EbvFragStencilRef:
        builder.addExtension(spv::E_SPV_EXT_shader_stencil_export);
        builder.addCapability(spv::CapabilityStencilExportEXT);
        return spv::BuiltInFragStencilRefEXT;

    case qglslang::EbvShadingRateKHR:
        builder.addExtension(spv::E_SPV_KHR_fragment_shading_rate);
        builder.addCapability(spv::CapabilityFragmentShadingRateKHR);
        return spv::BuiltInShadingRateKHR;

    case qglslang::EbvPrimitiveShadingRateKHR:
        builder.addExtension(spv::E_SPV_KHR_fragment_shading_rate);
        builder.addCapability(spv::CapabilityFragmentShadingRateKHR);
        return spv::BuiltInPrimitiveShadingRateKHR;

    case qglslang::EbvInvocationId:         return spv::BuiltInInvocationId;
    case qglslang::EbvTessLevelInner:       return spv::BuiltInTessLevelInner;
    case qglslang::EbvTessLevelOuter:       return spv::BuiltInTessLevelOuter;
    case qglslang::EbvTessCoord:            return spv::BuiltInTessCoord;
    case qglslang::EbvPatchVertices:        return spv::BuiltInPatchVertices;
    case qglslang::EbvHelperInvocation:     return spv::BuiltInHelperInvocation;

    case qglslang::EbvSubGroupSize:
        builder.addExtension(spv::E_SPV_KHR_shader_ballot);
        builder.addCapability(spv::CapabilitySubgroupBallotKHR);
        return spv::BuiltInSubgroupSize;

    case qglslang::EbvSubGroupInvocation:
        builder.addExtension(spv::E_SPV_KHR_shader_ballot);
        builder.addCapability(spv::CapabilitySubgroupBallotKHR);
        return spv::BuiltInSubgroupLocalInvocationId;

    case qglslang::EbvSubGroupEqMask:
        builder.addExtension(spv::E_SPV_KHR_shader_ballot);
        builder.addCapability(spv::CapabilitySubgroupBallotKHR);
        return spv::BuiltInSubgroupEqMask;

    case qglslang::EbvSubGroupGeMask:
        builder.addExtension(spv::E_SPV_KHR_shader_ballot);
        builder.addCapability(spv::CapabilitySubgroupBallotKHR);
        return spv::BuiltInSubgroupGeMask;

    case qglslang::EbvSubGroupGtMask:
        builder.addExtension(spv::E_SPV_KHR_shader_ballot);
        builder.addCapability(spv::CapabilitySubgroupBallotKHR);
        return spv::BuiltInSubgroupGtMask;

    case qglslang::EbvSubGroupLeMask:
        builder.addExtension(spv::E_SPV_KHR_shader_ballot);
        builder.addCapability(spv::CapabilitySubgroupBallotKHR);
        return spv::BuiltInSubgroupLeMask;

    case qglslang::EbvSubGroupLtMask:
        builder.addExtension(spv::E_SPV_KHR_shader_ballot);
        builder.addCapability(spv::CapabilitySubgroupBallotKHR);
        return spv::BuiltInSubgroupLtMask;

    case qglslang::EbvNumSubgroups:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        return spv::BuiltInNumSubgroups;

    case qglslang::EbvSubgroupID:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        return spv::BuiltInSubgroupId;

    case qglslang::EbvSubgroupSize2:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        return spv::BuiltInSubgroupSize;

    case qglslang::EbvSubgroupInvocation2:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        return spv::BuiltInSubgroupLocalInvocationId;

    case qglslang::EbvSubgroupEqMask2:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformBallot);
        return spv::BuiltInSubgroupEqMask;

    case qglslang::EbvSubgroupGeMask2:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformBallot);
        return spv::BuiltInSubgroupGeMask;

    case qglslang::EbvSubgroupGtMask2:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformBallot);
        return spv::BuiltInSubgroupGtMask;

    case qglslang::EbvSubgroupLeMask2:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformBallot);
        return spv::BuiltInSubgroupLeMask;

    case qglslang::EbvSubgroupLtMask2:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformBallot);
        return spv::BuiltInSubgroupLtMask;

    case qglslang::EbvBaryCoordNoPersp:
        builder.addExtension(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        return spv::BuiltInBaryCoordNoPerspAMD;

    case qglslang::EbvBaryCoordNoPerspCentroid:
        builder.addExtension(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        return spv::BuiltInBaryCoordNoPerspCentroidAMD;

    case qglslang::EbvBaryCoordNoPerspSample:
        builder.addExtension(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        return spv::BuiltInBaryCoordNoPerspSampleAMD;

    case qglslang::EbvBaryCoordSmooth:
        builder.addExtension(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        return spv::BuiltInBaryCoordSmoothAMD;

    case qglslang::EbvBaryCoordSmoothCentroid:
        builder.addExtension(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        return spv::BuiltInBaryCoordSmoothCentroidAMD;

    case qglslang::EbvBaryCoordSmoothSample:
        builder.addExtension(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        return spv::BuiltInBaryCoordSmoothSampleAMD;

    case qglslang::EbvBaryCoordPullModel:
        builder.addExtension(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        return spv::BuiltInBaryCoordPullModelAMD;

    case qglslang::EbvDeviceIndex:
        builder.addIncorporatedExtension(spv::E_SPV_KHR_device_group, spv::Spv_1_3);
        builder.addCapability(spv::CapabilityDeviceGroup);
        return spv::BuiltInDeviceIndex;

    case qglslang::EbvViewIndex:
        builder.addIncorporatedExtension(spv::E_SPV_KHR_multiview, spv::Spv_1_3);
        builder.addCapability(spv::CapabilityMultiView);
        return spv::BuiltInViewIndex;

    case qglslang::EbvFragSizeEXT:
        builder.addExtension(spv::E_SPV_EXT_fragment_invocation_density);
        builder.addCapability(spv::CapabilityFragmentDensityEXT);
        return spv::BuiltInFragSizeEXT;

    case qglslang::EbvFragInvocationCountEXT:
        builder.addExtension(spv::E_SPV_EXT_fragment_invocation_density);
        builder.addCapability(spv::CapabilityFragmentDensityEXT);
        return spv::BuiltInFragInvocationCountEXT;

    case qglslang::EbvViewportMaskNV:
        if (!memberDeclaration) {
            builder.addExtension(spv::E_SPV_NV_viewport_array2);
            builder.addCapability(spv::CapabilityShaderViewportMaskNV);
        }
        return spv::BuiltInViewportMaskNV;
    case qglslang::EbvSecondaryPositionNV:
        if (!memberDeclaration) {
            builder.addExtension(spv::E_SPV_NV_stereo_view_rendering);
            builder.addCapability(spv::CapabilityShaderStereoViewNV);
        }
        return spv::BuiltInSecondaryPositionNV;
    case qglslang::EbvSecondaryViewportMaskNV:
        if (!memberDeclaration) {
            builder.addExtension(spv::E_SPV_NV_stereo_view_rendering);
            builder.addCapability(spv::CapabilityShaderStereoViewNV);
        }
        return spv::BuiltInSecondaryViewportMaskNV;
    case qglslang::EbvPositionPerViewNV:
        if (!memberDeclaration) {
            builder.addExtension(spv::E_SPV_NVX_multiview_per_view_attributes);
            builder.addCapability(spv::CapabilityPerViewAttributesNV);
        }
        return spv::BuiltInPositionPerViewNV;
    case qglslang::EbvViewportMaskPerViewNV:
        if (!memberDeclaration) {
            builder.addExtension(spv::E_SPV_NVX_multiview_per_view_attributes);
            builder.addCapability(spv::CapabilityPerViewAttributesNV);
        }
        return spv::BuiltInViewportMaskPerViewNV;
    case qglslang::EbvFragFullyCoveredNV:
        builder.addExtension(spv::E_SPV_EXT_fragment_fully_covered);
        builder.addCapability(spv::CapabilityFragmentFullyCoveredEXT);
        return spv::BuiltInFullyCoveredEXT;
    case qglslang::EbvFragmentSizeNV:
        builder.addExtension(spv::E_SPV_NV_shading_rate);
        builder.addCapability(spv::CapabilityShadingRateNV);
        return spv::BuiltInFragmentSizeNV;
    case qglslang::EbvInvocationsPerPixelNV:
        builder.addExtension(spv::E_SPV_NV_shading_rate);
        builder.addCapability(spv::CapabilityShadingRateNV);
        return spv::BuiltInInvocationsPerPixelNV;

    // ray tracing
    case qglslang::EbvLaunchId:
        return spv::BuiltInLaunchIdKHR;
    case qglslang::EbvLaunchSize:
        return spv::BuiltInLaunchSizeKHR;
    case qglslang::EbvWorldRayOrigin:
        return spv::BuiltInWorldRayOriginKHR;
    case qglslang::EbvWorldRayDirection:
        return spv::BuiltInWorldRayDirectionKHR;
    case qglslang::EbvObjectRayOrigin:
        return spv::BuiltInObjectRayOriginKHR;
    case qglslang::EbvObjectRayDirection:
        return spv::BuiltInObjectRayDirectionKHR;
    case qglslang::EbvRayTmin:
        return spv::BuiltInRayTminKHR;
    case qglslang::EbvRayTmax:
        return spv::BuiltInRayTmaxKHR;
    case qglslang::EbvCullMask:
        return spv::BuiltInCullMaskKHR;
    case qglslang::EbvInstanceCustomIndex:
        return spv::BuiltInInstanceCustomIndexKHR;
    case qglslang::EbvHitT:
        {
            // this is a GLSL alias of RayTmax
            // in SPV_NV_ray_tracing it has a dedicated builtin
            // but in SPV_KHR_ray_tracing it gets mapped to RayTmax
            auto& extensions = glslangIntermediate->getRequestedExtensions();
            if (extensions.find("GL_NV_ray_tracing") != extensions.end()) {
                return spv::BuiltInHitTNV;
            } else {
                return spv::BuiltInRayTmaxKHR;
            }
        }
    case qglslang::EbvHitKind:
        return spv::BuiltInHitKindKHR;
    case qglslang::EbvObjectToWorld:
    case qglslang::EbvObjectToWorld3x4:
        return spv::BuiltInObjectToWorldKHR;
    case qglslang::EbvWorldToObject:
    case qglslang::EbvWorldToObject3x4:
        return spv::BuiltInWorldToObjectKHR;
    case qglslang::EbvIncomingRayFlags:
        return spv::BuiltInIncomingRayFlagsKHR;
    case qglslang::EbvGeometryIndex:
        return spv::BuiltInRayGeometryIndexKHR;
    case qglslang::EbvCurrentRayTimeNV:
        builder.addExtension(spv::E_SPV_NV_ray_tracing_motion_blur);
        builder.addCapability(spv::CapabilityRayTracingMotionBlurNV);
        return spv::BuiltInCurrentRayTimeNV;

    // barycentrics
    case qglslang::EbvBaryCoordNV:
        builder.addExtension(spv::E_SPV_NV_fragment_shader_barycentric);
        builder.addCapability(spv::CapabilityFragmentBarycentricNV);
        return spv::BuiltInBaryCoordNV;
    case qglslang::EbvBaryCoordNoPerspNV:
        builder.addExtension(spv::E_SPV_NV_fragment_shader_barycentric);
        builder.addCapability(spv::CapabilityFragmentBarycentricNV);
        return spv::BuiltInBaryCoordNoPerspNV;

    case qglslang::EbvBaryCoordEXT:
        builder.addExtension(spv::E_SPV_KHR_fragment_shader_barycentric);
        builder.addCapability(spv::CapabilityFragmentBarycentricKHR);
        return spv::BuiltInBaryCoordKHR;
    case qglslang::EbvBaryCoordNoPerspEXT:
        builder.addExtension(spv::E_SPV_KHR_fragment_shader_barycentric);
        builder.addCapability(spv::CapabilityFragmentBarycentricKHR);
        return spv::BuiltInBaryCoordNoPerspKHR;

    // mesh shaders
    case qglslang::EbvTaskCountNV:
        return spv::BuiltInTaskCountNV;
    case qglslang::EbvPrimitiveCountNV:
        return spv::BuiltInPrimitiveCountNV;
    case qglslang::EbvPrimitiveIndicesNV:
        return spv::BuiltInPrimitiveIndicesNV;
    case qglslang::EbvClipDistancePerViewNV:
        return spv::BuiltInClipDistancePerViewNV;
    case qglslang::EbvCullDistancePerViewNV:
        return spv::BuiltInCullDistancePerViewNV;
    case qglslang::EbvLayerPerViewNV:
        return spv::BuiltInLayerPerViewNV;
    case qglslang::EbvMeshViewCountNV:
        return spv::BuiltInMeshViewCountNV;
    case qglslang::EbvMeshViewIndicesNV:
        return spv::BuiltInMeshViewIndicesNV;

    // sm builtins
    case qglslang::EbvWarpsPerSM:
        builder.addExtension(spv::E_SPV_NV_shader_sm_builtins);
        builder.addCapability(spv::CapabilityShaderSMBuiltinsNV);
        return spv::BuiltInWarpsPerSMNV;
    case qglslang::EbvSMCount:
        builder.addExtension(spv::E_SPV_NV_shader_sm_builtins);
        builder.addCapability(spv::CapabilityShaderSMBuiltinsNV);
        return spv::BuiltInSMCountNV;
    case qglslang::EbvWarpID:
        builder.addExtension(spv::E_SPV_NV_shader_sm_builtins);
        builder.addCapability(spv::CapabilityShaderSMBuiltinsNV);
        return spv::BuiltInWarpIDNV;
    case qglslang::EbvSMID:
        builder.addExtension(spv::E_SPV_NV_shader_sm_builtins);
        builder.addCapability(spv::CapabilityShaderSMBuiltinsNV);
        return spv::BuiltInSMIDNV;
#endif

    default:
        return spv::BuiltInMax;
    }
}

// Translate glslang image layout format to SPIR-V image format.
spv::ImageFormat TGlslangToSpvTraverser::TranslateImageFormat(const qglslang::TType& type)
{
    assert(type.getBasicType() == qglslang::EbtSampler);

#ifdef GLSLANG_WEB
    return spv::ImageFormatUnknown;
#endif

    // Check for capabilities
    switch (type.getQualifier().getFormat()) {
    case qglslang::ElfRg32f:
    case qglslang::ElfRg16f:
    case qglslang::ElfR11fG11fB10f:
    case qglslang::ElfR16f:
    case qglslang::ElfRgba16:
    case qglslang::ElfRgb10A2:
    case qglslang::ElfRg16:
    case qglslang::ElfRg8:
    case qglslang::ElfR16:
    case qglslang::ElfR8:
    case qglslang::ElfRgba16Snorm:
    case qglslang::ElfRg16Snorm:
    case qglslang::ElfRg8Snorm:
    case qglslang::ElfR16Snorm:
    case qglslang::ElfR8Snorm:

    case qglslang::ElfRg32i:
    case qglslang::ElfRg16i:
    case qglslang::ElfRg8i:
    case qglslang::ElfR16i:
    case qglslang::ElfR8i:

    case qglslang::ElfRgb10a2ui:
    case qglslang::ElfRg32ui:
    case qglslang::ElfRg16ui:
    case qglslang::ElfRg8ui:
    case qglslang::ElfR16ui:
    case qglslang::ElfR8ui:
        builder.addCapability(spv::CapabilityStorageImageExtendedFormats);
        break;

    case qglslang::ElfR64ui:
    case qglslang::ElfR64i:
        builder.addExtension(spv::E_SPV_EXT_shader_image_int64);
        builder.addCapability(spv::CapabilityInt64ImageEXT);
    default:
        break;
    }

    // do the translation
    switch (type.getQualifier().getFormat()) {
    case qglslang::ElfNone:          return spv::ImageFormatUnknown;
    case qglslang::ElfRgba32f:       return spv::ImageFormatRgba32f;
    case qglslang::ElfRgba16f:       return spv::ImageFormatRgba16f;
    case qglslang::ElfR32f:          return spv::ImageFormatR32f;
    case qglslang::ElfRgba8:         return spv::ImageFormatRgba8;
    case qglslang::ElfRgba8Snorm:    return spv::ImageFormatRgba8Snorm;
    case qglslang::ElfRg32f:         return spv::ImageFormatRg32f;
    case qglslang::ElfRg16f:         return spv::ImageFormatRg16f;
    case qglslang::ElfR11fG11fB10f:  return spv::ImageFormatR11fG11fB10f;
    case qglslang::ElfR16f:          return spv::ImageFormatR16f;
    case qglslang::ElfRgba16:        return spv::ImageFormatRgba16;
    case qglslang::ElfRgb10A2:       return spv::ImageFormatRgb10A2;
    case qglslang::ElfRg16:          return spv::ImageFormatRg16;
    case qglslang::ElfRg8:           return spv::ImageFormatRg8;
    case qglslang::ElfR16:           return spv::ImageFormatR16;
    case qglslang::ElfR8:            return spv::ImageFormatR8;
    case qglslang::ElfRgba16Snorm:   return spv::ImageFormatRgba16Snorm;
    case qglslang::ElfRg16Snorm:     return spv::ImageFormatRg16Snorm;
    case qglslang::ElfRg8Snorm:      return spv::ImageFormatRg8Snorm;
    case qglslang::ElfR16Snorm:      return spv::ImageFormatR16Snorm;
    case qglslang::ElfR8Snorm:       return spv::ImageFormatR8Snorm;
    case qglslang::ElfRgba32i:       return spv::ImageFormatRgba32i;
    case qglslang::ElfRgba16i:       return spv::ImageFormatRgba16i;
    case qglslang::ElfRgba8i:        return spv::ImageFormatRgba8i;
    case qglslang::ElfR32i:          return spv::ImageFormatR32i;
    case qglslang::ElfRg32i:         return spv::ImageFormatRg32i;
    case qglslang::ElfRg16i:         return spv::ImageFormatRg16i;
    case qglslang::ElfRg8i:          return spv::ImageFormatRg8i;
    case qglslang::ElfR16i:          return spv::ImageFormatR16i;
    case qglslang::ElfR8i:           return spv::ImageFormatR8i;
    case qglslang::ElfRgba32ui:      return spv::ImageFormatRgba32ui;
    case qglslang::ElfRgba16ui:      return spv::ImageFormatRgba16ui;
    case qglslang::ElfRgba8ui:       return spv::ImageFormatRgba8ui;
    case qglslang::ElfR32ui:         return spv::ImageFormatR32ui;
    case qglslang::ElfRg32ui:        return spv::ImageFormatRg32ui;
    case qglslang::ElfRg16ui:        return spv::ImageFormatRg16ui;
    case qglslang::ElfRgb10a2ui:     return spv::ImageFormatRgb10a2ui;
    case qglslang::ElfRg8ui:         return spv::ImageFormatRg8ui;
    case qglslang::ElfR16ui:         return spv::ImageFormatR16ui;
    case qglslang::ElfR8ui:          return spv::ImageFormatR8ui;
    case qglslang::ElfR64ui:         return spv::ImageFormatR64ui;
    case qglslang::ElfR64i:          return spv::ImageFormatR64i;
    default:                        return spv::ImageFormatMax;
    }
}

spv::SelectionControlMask TGlslangToSpvTraverser::TranslateSelectionControl(
    const qglslang::TIntermSelection& selectionNode) const
{
    if (selectionNode.getFlatten())
        return spv::SelectionControlFlattenMask;
    if (selectionNode.getDontFlatten())
        return spv::SelectionControlDontFlattenMask;
    return spv::SelectionControlMaskNone;
}

spv::SelectionControlMask TGlslangToSpvTraverser::TranslateSwitchControl(const qglslang::TIntermSwitch& switchNode)
    const
{
    if (switchNode.getFlatten())
        return spv::SelectionControlFlattenMask;
    if (switchNode.getDontFlatten())
        return spv::SelectionControlDontFlattenMask;
    return spv::SelectionControlMaskNone;
}

// return a non-0 dependency if the dependency argument must be set
spv::LoopControlMask TGlslangToSpvTraverser::TranslateLoopControl(const qglslang::TIntermLoop& loopNode,
    std::vector<unsigned int>& operands) const
{
    spv::LoopControlMask control = spv::LoopControlMaskNone;

    if (loopNode.getDontUnroll())
        control = control | spv::LoopControlDontUnrollMask;
    if (loopNode.getUnroll())
        control = control | spv::LoopControlUnrollMask;
    if (unsigned(loopNode.getLoopDependency()) == qglslang::TIntermLoop::dependencyInfinite)
        control = control | spv::LoopControlDependencyInfiniteMask;
    else if (loopNode.getLoopDependency() > 0) {
        control = control | spv::LoopControlDependencyLengthMask;
        operands.push_back((unsigned int)loopNode.getLoopDependency());
    }
    if (glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_4) {
        if (loopNode.getMinIterations() > 0) {
            control = control | spv::LoopControlMinIterationsMask;
            operands.push_back(loopNode.getMinIterations());
        }
        if (loopNode.getMaxIterations() < qglslang::TIntermLoop::iterationsInfinite) {
            control = control | spv::LoopControlMaxIterationsMask;
            operands.push_back(loopNode.getMaxIterations());
        }
        if (loopNode.getIterationMultiple() > 1) {
            control = control | spv::LoopControlIterationMultipleMask;
            operands.push_back(loopNode.getIterationMultiple());
        }
        if (loopNode.getPeelCount() > 0) {
            control = control | spv::LoopControlPeelCountMask;
            operands.push_back(loopNode.getPeelCount());
        }
        if (loopNode.getPartialCount() > 0) {
            control = control | spv::LoopControlPartialCountMask;
            operands.push_back(loopNode.getPartialCount());
        }
    }

    return control;
}

// Translate glslang type to SPIR-V storage class.
spv::StorageClass TGlslangToSpvTraverser::TranslateStorageClass(const qglslang::TType& type)
{
    if (type.getBasicType() == qglslang::EbtRayQuery)
        return spv::StorageClassPrivate;
#ifndef GLSLANG_WEB
    if (type.getQualifier().isSpirvByReference()) {
        if (type.getQualifier().isParamInput() || type.getQualifier().isParamOutput())
            return spv::StorageClassFunction;
    }
#endif
    if (type.getQualifier().isPipeInput())
        return spv::StorageClassInput;
    if (type.getQualifier().isPipeOutput())
        return spv::StorageClassOutput;

    if (glslangIntermediate->getSource() != qglslang::EShSourceHlsl ||
            type.getQualifier().storage == qglslang::EvqUniform) {
        if (type.isAtomic())
            return spv::StorageClassAtomicCounter;
        if (type.containsOpaque())
            return spv::StorageClassUniformConstant;
    }

    if (type.getQualifier().isUniformOrBuffer() &&
        type.getQualifier().isShaderRecord()) {
        return spv::StorageClassShaderRecordBufferKHR;
    }

    if (glslangIntermediate->usingStorageBuffer() && type.getQualifier().storage == qglslang::EvqBuffer) {
        builder.addIncorporatedExtension(spv::E_SPV_KHR_storage_buffer_storage_class, spv::Spv_1_3);
        return spv::StorageClassStorageBuffer;
    }

    if (type.getQualifier().isUniformOrBuffer()) {
        if (type.getQualifier().isPushConstant())
            return spv::StorageClassPushConstant;
        if (type.getBasicType() == qglslang::EbtBlock)
            return spv::StorageClassUniform;
        return spv::StorageClassUniformConstant;
    }

    if (type.getQualifier().storage == qglslang::EvqShared && type.getBasicType() == qglslang::EbtBlock) {
        builder.addExtension(spv::E_SPV_KHR_workgroup_memory_explicit_layout);
        builder.addCapability(spv::CapabilityWorkgroupMemoryExplicitLayoutKHR);
        return spv::StorageClassWorkgroup;
    }

    switch (type.getQualifier().storage) {
    case qglslang::EvqGlobal:        return spv::StorageClassPrivate;
    case qglslang::EvqConstReadOnly: return spv::StorageClassFunction;
    case qglslang::EvqTemporary:     return spv::StorageClassFunction;
    case qglslang::EvqShared:           return spv::StorageClassWorkgroup;
#ifndef GLSLANG_WEB
    case qglslang::EvqPayload:        return spv::StorageClassRayPayloadKHR;
    case qglslang::EvqPayloadIn:      return spv::StorageClassIncomingRayPayloadKHR;
    case qglslang::EvqHitAttr:        return spv::StorageClassHitAttributeKHR;
    case qglslang::EvqCallableData:   return spv::StorageClassCallableDataKHR;
    case qglslang::EvqCallableDataIn: return spv::StorageClassIncomingCallableDataKHR;
    case qglslang::EvqSpirvStorageClass: return static_cast<spv::StorageClass>(type.getQualifier().spirvStorageClass);
#endif
    default:
        assert(0);
        break;
    }

    return spv::StorageClassFunction;
}

// Translate glslang constants to SPIR-V literals
void TGlslangToSpvTraverser::TranslateLiterals(const qglslang::TVector<const qglslang::TIntermConstantUnion*>& constants,
                                               std::vector<unsigned>& literals) const
{
    for (auto constant : constants) {
        if (constant->getBasicType() == qglslang::EbtFloat) {
            float floatValue = static_cast<float>(constant->getConstArray()[0].getDConst());
            unsigned literal = *reinterpret_cast<unsigned*>(&floatValue);
            literals.push_back(literal);
        } else if (constant->getBasicType() == qglslang::EbtInt) {
            unsigned literal = constant->getConstArray()[0].getIConst();
            literals.push_back(literal);
        } else if (constant->getBasicType() == qglslang::EbtUint) {
            unsigned literal = constant->getConstArray()[0].getUConst();
            literals.push_back(literal);
        } else if (constant->getBasicType() == qglslang::EbtBool) {
            unsigned literal = constant->getConstArray()[0].getBConst();
            literals.push_back(literal);
        } else if (constant->getBasicType() == qglslang::EbtString) {
            auto str = constant->getConstArray()[0].getSConst()->c_str();
            unsigned literal = 0;
            char* literalPtr = reinterpret_cast<char*>(&literal);
            unsigned charCount = 0;
            char ch = 0;
            do {
                ch = *(str++);
                *(literalPtr++) = ch;
                ++charCount;
                if (charCount == 4) {
                    literals.push_back(literal);
                    literalPtr = reinterpret_cast<char*>(&literal);
                    charCount = 0;
                }
            } while (ch != 0);

            // Partial literal is padded with 0
            if (charCount > 0) {
                for (; charCount < 4; ++charCount)
                    *(literalPtr++) = 0;
                literals.push_back(literal);
            }
        } else
            assert(0); // Unexpected type
    }
}

// Add capabilities pertaining to how an array is indexed.
void TGlslangToSpvTraverser::addIndirectionIndexCapabilities(const qglslang::TType& baseType,
                                                             const qglslang::TType& indexType)
{
#ifndef GLSLANG_WEB
    if (indexType.getQualifier().isNonUniform()) {
        // deal with an asserted non-uniform index
        // SPV_EXT_descriptor_indexing already added in TranslateNonUniformDecoration
        if (baseType.getBasicType() == qglslang::EbtSampler) {
            if (baseType.getQualifier().hasAttachment())
                builder.addCapability(spv::CapabilityInputAttachmentArrayNonUniformIndexingEXT);
            else if (baseType.isImage() && baseType.getSampler().isBuffer())
                builder.addCapability(spv::CapabilityStorageTexelBufferArrayNonUniformIndexingEXT);
            else if (baseType.isTexture() && baseType.getSampler().isBuffer())
                builder.addCapability(spv::CapabilityUniformTexelBufferArrayNonUniformIndexingEXT);
            else if (baseType.isImage())
                builder.addCapability(spv::CapabilityStorageImageArrayNonUniformIndexingEXT);
            else if (baseType.isTexture())
                builder.addCapability(spv::CapabilitySampledImageArrayNonUniformIndexingEXT);
        } else if (baseType.getBasicType() == qglslang::EbtBlock) {
            if (baseType.getQualifier().storage == qglslang::EvqBuffer)
                builder.addCapability(spv::CapabilityStorageBufferArrayNonUniformIndexingEXT);
            else if (baseType.getQualifier().storage == qglslang::EvqUniform)
                builder.addCapability(spv::CapabilityUniformBufferArrayNonUniformIndexingEXT);
        }
    } else {
        // assume a dynamically uniform index
        if (baseType.getBasicType() == qglslang::EbtSampler) {
            if (baseType.getQualifier().hasAttachment()) {
                builder.addIncorporatedExtension("SPV_EXT_descriptor_indexing", spv::Spv_1_5);
                builder.addCapability(spv::CapabilityInputAttachmentArrayDynamicIndexingEXT);
            } else if (baseType.isImage() && baseType.getSampler().isBuffer()) {
                builder.addIncorporatedExtension("SPV_EXT_descriptor_indexing", spv::Spv_1_5);
                builder.addCapability(spv::CapabilityStorageTexelBufferArrayDynamicIndexingEXT);
            } else if (baseType.isTexture() && baseType.getSampler().isBuffer()) {
                builder.addIncorporatedExtension("SPV_EXT_descriptor_indexing", spv::Spv_1_5);
                builder.addCapability(spv::CapabilityUniformTexelBufferArrayDynamicIndexingEXT);
            }
        }
    }
#endif
}

// Return whether or not the given type is something that should be tied to a
// descriptor set.
bool IsDescriptorResource(const qglslang::TType& type)
{
    // uniform and buffer blocks are included, unless it is a push_constant
    if (type.getBasicType() == qglslang::EbtBlock)
        return type.getQualifier().isUniformOrBuffer() &&
        ! type.getQualifier().isShaderRecord() &&
        ! type.getQualifier().isPushConstant();

    // non block...
    // basically samplerXXX/subpass/sampler/texture are all included
    // if they are the global-scope-class, not the function parameter
    // (or local, if they ever exist) class.
    if (type.getBasicType() == qglslang::EbtSampler ||
        type.getBasicType() == qglslang::EbtAccStruct)
        return type.getQualifier().isUniformOrBuffer();

    // None of the above.
    return false;
}

void InheritQualifiers(qglslang::TQualifier& child, const qglslang::TQualifier& parent)
{
    if (child.layoutMatrix == qglslang::ElmNone)
        child.layoutMatrix = parent.layoutMatrix;

    if (parent.invariant)
        child.invariant = true;
    if (parent.flat)
        child.flat = true;
    if (parent.centroid)
        child.centroid = true;
#ifndef GLSLANG_WEB
    if (parent.nopersp)
        child.nopersp = true;
    if (parent.explicitInterp)
        child.explicitInterp = true;
    if (parent.perPrimitiveNV)
        child.perPrimitiveNV = true;
    if (parent.perViewNV)
        child.perViewNV = true;
    if (parent.perTaskNV)
        child.perTaskNV = true;
    if (parent.patch)
        child.patch = true;
    if (parent.sample)
        child.sample = true;
    if (parent.coherent)
        child.coherent = true;
    if (parent.devicecoherent)
        child.devicecoherent = true;
    if (parent.queuefamilycoherent)
        child.queuefamilycoherent = true;
    if (parent.workgroupcoherent)
        child.workgroupcoherent = true;
    if (parent.subgroupcoherent)
        child.subgroupcoherent = true;
    if (parent.shadercallcoherent)
        child.shadercallcoherent = true;
    if (parent.nonprivate)
        child.nonprivate = true;
    if (parent.volatil)
        child.volatil = true;
    if (parent.restrict)
        child.restrict = true;
    if (parent.readonly)
        child.readonly = true;
    if (parent.writeonly)
        child.writeonly = true;
#endif
    if (parent.nonUniform)
        child.nonUniform = true;
}

bool HasNonLayoutQualifiers(const qglslang::TType& type, const qglslang::TQualifier& qualifier)
{
    // This should list qualifiers that simultaneous satisfy:
    // - struct members might inherit from a struct declaration
    //     (note that non-block structs don't explicitly inherit,
    //      only implicitly, meaning no decoration involved)
    // - affect decorations on the struct members
    //     (note smooth does not, and expecting something like volatile
    //      to effect the whole object)
    // - are not part of the offset/st430/etc or row/column-major layout
    return qualifier.invariant || (qualifier.hasLocation() && type.getBasicType() == qglslang::EbtBlock);
}

//
// Implement the TGlslangToSpvTraverser class.
//

TGlslangToSpvTraverser::TGlslangToSpvTraverser(unsigned int spvVersion,
    const qglslang::TIntermediate* glslangIntermediate,
    spv::SpvBuildLogger* buildLogger, qglslang::SpvOptions& options) :
        TIntermTraverser(true, false, true),
        options(options),
        shaderEntry(nullptr), currentFunction(nullptr),
        sequenceDepth(0), logger(buildLogger),
        builder(spvVersion, (qglslang::GetKhronosToolId() << 16) | qglslang::GetSpirvGeneratorVersion(), logger),
        inEntryPoint(false), entryPointTerminated(false), linkageOnly(false),
        glslangIntermediate(glslangIntermediate),
        nanMinMaxClamp(glslangIntermediate->getNanMinMaxClamp()),
        nonSemanticDebugPrintf(0)
{
    spv::ExecutionModel executionModel = TranslateExecutionModel(glslangIntermediate->getStage());

    builder.clearAccessChain();
    builder.setSource(TranslateSourceLanguage(glslangIntermediate->getSource(), glslangIntermediate->getProfile()),
                      glslangIntermediate->getVersion());

    if (options.generateDebugInfo) {
        builder.setEmitOpLines();
        builder.setSourceFile(glslangIntermediate->getSourceFile());

        // Set the source shader's text. If for SPV version 1.0, include
        // a preamble in comments stating the OpModuleProcessed instructions.
        // Otherwise, emit those as actual instructions.
        std::string text;
        const std::vector<std::string>& processes = glslangIntermediate->getProcesses();
        for (int p = 0; p < (int)processes.size(); ++p) {
            if (glslangIntermediate->getSpv().spv < qglslang::EShTargetSpv_1_1) {
                text.append("// OpModuleProcessed ");
                text.append(processes[p]);
                text.append("\n");
            } else
                builder.addModuleProcessed(processes[p]);
        }
        if (glslangIntermediate->getSpv().spv < qglslang::EShTargetSpv_1_1 && (int)processes.size() > 0)
            text.append("#line 1\n");
        text.append(glslangIntermediate->getSourceText());
        builder.setSourceText(text);
        // Pass name and text for all included files
        const std::map<std::string, std::string>& include_txt = glslangIntermediate->getIncludeText();
        for (auto iItr = include_txt.begin(); iItr != include_txt.end(); ++iItr)
            builder.addInclude(iItr->first, iItr->second);
    }
    stdBuiltins = builder.import("GLSL.std.450");

    spv::AddressingModel addressingModel = spv::AddressingModelLogical;
    spv::MemoryModel memoryModel = spv::MemoryModelGLSL450;

    if (glslangIntermediate->usingPhysicalStorageBuffer()) {
        addressingModel = spv::AddressingModelPhysicalStorageBuffer64EXT;
        builder.addIncorporatedExtension(spv::E_SPV_KHR_physical_storage_buffer, spv::Spv_1_5);
        builder.addCapability(spv::CapabilityPhysicalStorageBufferAddressesEXT);
    }
    if (glslangIntermediate->usingVulkanMemoryModel()) {
        memoryModel = spv::MemoryModelVulkanKHR;
        builder.addCapability(spv::CapabilityVulkanMemoryModelKHR);
        builder.addIncorporatedExtension(spv::E_SPV_KHR_vulkan_memory_model, spv::Spv_1_5);
    }
    builder.setMemoryModel(addressingModel, memoryModel);

    if (glslangIntermediate->usingVariablePointers()) {
        builder.addCapability(spv::CapabilityVariablePointers);
    }

    shaderEntry = builder.makeEntryPoint(glslangIntermediate->getEntryPointName().c_str());
    entryPoint = builder.addEntryPoint(executionModel, shaderEntry, glslangIntermediate->getEntryPointName().c_str());

    // Add the source extensions
    const auto& sourceExtensions = glslangIntermediate->getRequestedExtensions();
    for (auto it = sourceExtensions.begin(); it != sourceExtensions.end(); ++it)
        builder.addSourceExtension(it->c_str());

    // Add the top-level modes for this shader.

    if (glslangIntermediate->getXfbMode()) {
        builder.addCapability(spv::CapabilityTransformFeedback);
        builder.addExecutionMode(shaderEntry, spv::ExecutionModeXfb);
    }

    if (glslangIntermediate->getLayoutPrimitiveCulling()) {
        builder.addCapability(spv::CapabilityRayTraversalPrimitiveCullingKHR);
    }

#ifndef GLSLANG_WEB
    if (glslangIntermediate->getSubgroupUniformControlFlow()) {
        builder.addExtension(spv::E_SPV_KHR_subgroup_uniform_control_flow);
        builder.addExecutionMode(shaderEntry, spv::ExecutionModeSubgroupUniformControlFlowKHR);
    }
#endif

    unsigned int mode;
    switch (glslangIntermediate->getStage()) {
    case EShLangVertex:
        builder.addCapability(spv::CapabilityShader);
        break;

    case EShLangFragment:
        builder.addCapability(spv::CapabilityShader);
        if (glslangIntermediate->getPixelCenterInteger())
            builder.addExecutionMode(shaderEntry, spv::ExecutionModePixelCenterInteger);

        if (glslangIntermediate->getOriginUpperLeft())
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeOriginUpperLeft);
        else
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeOriginLowerLeft);

        if (glslangIntermediate->getEarlyFragmentTests())
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeEarlyFragmentTests);

        if (glslangIntermediate->getPostDepthCoverage()) {
            builder.addCapability(spv::CapabilitySampleMaskPostDepthCoverage);
            builder.addExecutionMode(shaderEntry, spv::ExecutionModePostDepthCoverage);
            builder.addExtension(spv::E_SPV_KHR_post_depth_coverage);
        }

        if (glslangIntermediate->isDepthReplacing())
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeDepthReplacing);

#ifndef GLSLANG_WEB

        switch(glslangIntermediate->getDepth()) {
        case qglslang::EldGreater:   mode = spv::ExecutionModeDepthGreater;   break;
        case qglslang::EldLess:      mode = spv::ExecutionModeDepthLess;      break;
        case qglslang::EldUnchanged: mode = spv::ExecutionModeDepthUnchanged; break;
        default:                    mode = spv::ExecutionModeMax;            break;
        }
        if (mode != spv::ExecutionModeMax)
            builder.addExecutionMode(shaderEntry, (spv::ExecutionMode)mode);
        switch (glslangIntermediate->getInterlockOrdering()) {
        case qglslang::EioPixelInterlockOrdered:         mode = spv::ExecutionModePixelInterlockOrderedEXT;
            break;
        case qglslang::EioPixelInterlockUnordered:       mode = spv::ExecutionModePixelInterlockUnorderedEXT;
            break;
        case qglslang::EioSampleInterlockOrdered:        mode = spv::ExecutionModeSampleInterlockOrderedEXT;
            break;
        case qglslang::EioSampleInterlockUnordered:      mode = spv::ExecutionModeSampleInterlockUnorderedEXT;
            break;
        case qglslang::EioShadingRateInterlockOrdered:   mode = spv::ExecutionModeShadingRateInterlockOrderedEXT;
            break;
        case qglslang::EioShadingRateInterlockUnordered: mode = spv::ExecutionModeShadingRateInterlockUnorderedEXT;
            break;
        default:                                        mode = spv::ExecutionModeMax;
            break;
        }
        if (mode != spv::ExecutionModeMax) {
            builder.addExecutionMode(shaderEntry, (spv::ExecutionMode)mode);
            if (mode == spv::ExecutionModeShadingRateInterlockOrderedEXT ||
                mode == spv::ExecutionModeShadingRateInterlockUnorderedEXT) {
                builder.addCapability(spv::CapabilityFragmentShaderShadingRateInterlockEXT);
            } else if (mode == spv::ExecutionModePixelInterlockOrderedEXT ||
                       mode == spv::ExecutionModePixelInterlockUnorderedEXT) {
                builder.addCapability(spv::CapabilityFragmentShaderPixelInterlockEXT);
            } else {
                builder.addCapability(spv::CapabilityFragmentShaderSampleInterlockEXT);
            }
            builder.addExtension(spv::E_SPV_EXT_fragment_shader_interlock);
        }
#endif
    break;

    case EShLangCompute:
        builder.addCapability(spv::CapabilityShader);
        if (glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_6) {
          std::vector<spv::Id> dimConstId;
          for (int dim = 0; dim < 3; ++dim) {
            bool specConst = (glslangIntermediate->getLocalSizeSpecId(dim) != qglslang::TQualifier::layoutNotSet);
            dimConstId.push_back(builder.makeUintConstant(glslangIntermediate->getLocalSize(dim), specConst));
            if (specConst) {
                builder.addDecoration(dimConstId.back(), spv::DecorationSpecId,
                                      glslangIntermediate->getLocalSizeSpecId(dim));
            }
          }
          builder.addExecutionModeId(shaderEntry, spv::ExecutionModeLocalSizeId, dimConstId);
        } else {
          builder.addExecutionMode(shaderEntry, spv::ExecutionModeLocalSize, glslangIntermediate->getLocalSize(0),
                                                                             glslangIntermediate->getLocalSize(1),
                                                                             glslangIntermediate->getLocalSize(2));
        }
        if (glslangIntermediate->getLayoutDerivativeModeNone() == qglslang::LayoutDerivativeGroupQuads) {
            builder.addCapability(spv::CapabilityComputeDerivativeGroupQuadsNV);
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeDerivativeGroupQuadsNV);
            builder.addExtension(spv::E_SPV_NV_compute_shader_derivatives);
        } else if (glslangIntermediate->getLayoutDerivativeModeNone() == qglslang::LayoutDerivativeGroupLinear) {
            builder.addCapability(spv::CapabilityComputeDerivativeGroupLinearNV);
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeDerivativeGroupLinearNV);
            builder.addExtension(spv::E_SPV_NV_compute_shader_derivatives);
        }
        break;
#ifndef GLSLANG_WEB
    case EShLangTessEvaluation:
    case EShLangTessControl:
        builder.addCapability(spv::CapabilityTessellation);

        qglslang::TLayoutGeometry primitive;

        if (glslangIntermediate->getStage() == EShLangTessControl) {
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeOutputVertices,
                glslangIntermediate->getVertices());
            primitive = glslangIntermediate->getOutputPrimitive();
        } else {
            primitive = glslangIntermediate->getInputPrimitive();
        }

        switch (primitive) {
        case qglslang::ElgTriangles:           mode = spv::ExecutionModeTriangles;     break;
        case qglslang::ElgQuads:               mode = spv::ExecutionModeQuads;         break;
        case qglslang::ElgIsolines:            mode = spv::ExecutionModeIsolines;      break;
        default:                              mode = spv::ExecutionModeMax;           break;
        }
        if (mode != spv::ExecutionModeMax)
            builder.addExecutionMode(shaderEntry, (spv::ExecutionMode)mode);

        switch (glslangIntermediate->getVertexSpacing()) {
        case qglslang::EvsEqual:            mode = spv::ExecutionModeSpacingEqual;          break;
        case qglslang::EvsFractionalEven:   mode = spv::ExecutionModeSpacingFractionalEven; break;
        case qglslang::EvsFractionalOdd:    mode = spv::ExecutionModeSpacingFractionalOdd;  break;
        default:                           mode = spv::ExecutionModeMax;                   break;
        }
        if (mode != spv::ExecutionModeMax)
            builder.addExecutionMode(shaderEntry, (spv::ExecutionMode)mode);

        switch (glslangIntermediate->getVertexOrder()) {
        case qglslang::EvoCw:     mode = spv::ExecutionModeVertexOrderCw;  break;
        case qglslang::EvoCcw:    mode = spv::ExecutionModeVertexOrderCcw; break;
        default:                 mode = spv::ExecutionModeMax;            break;
        }
        if (mode != spv::ExecutionModeMax)
            builder.addExecutionMode(shaderEntry, (spv::ExecutionMode)mode);

        if (glslangIntermediate->getPointMode())
            builder.addExecutionMode(shaderEntry, spv::ExecutionModePointMode);
        break;

    case EShLangGeometry:
        builder.addCapability(spv::CapabilityGeometry);
        switch (glslangIntermediate->getInputPrimitive()) {
        case qglslang::ElgPoints:             mode = spv::ExecutionModeInputPoints;             break;
        case qglslang::ElgLines:              mode = spv::ExecutionModeInputLines;              break;
        case qglslang::ElgLinesAdjacency:     mode = spv::ExecutionModeInputLinesAdjacency;     break;
        case qglslang::ElgTriangles:          mode = spv::ExecutionModeTriangles;               break;
        case qglslang::ElgTrianglesAdjacency: mode = spv::ExecutionModeInputTrianglesAdjacency; break;
        default:                             mode = spv::ExecutionModeMax;                     break;
        }
        if (mode != spv::ExecutionModeMax)
            builder.addExecutionMode(shaderEntry, (spv::ExecutionMode)mode);

        builder.addExecutionMode(shaderEntry, spv::ExecutionModeInvocations, glslangIntermediate->getInvocations());

        switch (glslangIntermediate->getOutputPrimitive()) {
        case qglslang::ElgPoints:        mode = spv::ExecutionModeOutputPoints;                 break;
        case qglslang::ElgLineStrip:     mode = spv::ExecutionModeOutputLineStrip;              break;
        case qglslang::ElgTriangleStrip: mode = spv::ExecutionModeOutputTriangleStrip;          break;
        default:                        mode = spv::ExecutionModeMax;                          break;
        }
        if (mode != spv::ExecutionModeMax)
            builder.addExecutionMode(shaderEntry, (spv::ExecutionMode)mode);
        builder.addExecutionMode(shaderEntry, spv::ExecutionModeOutputVertices, glslangIntermediate->getVertices());
        break;

    case EShLangRayGen:
    case EShLangIntersect:
    case EShLangAnyHit:
    case EShLangClosestHit:
    case EShLangMiss:
    case EShLangCallable: 
    {
        auto& extensions = glslangIntermediate->getRequestedExtensions();
        if (extensions.find("GL_NV_ray_tracing") == extensions.end()) {
            builder.addCapability(spv::CapabilityRayTracingKHR);
            builder.addExtension("SPV_KHR_ray_tracing");
        }
        else {
            builder.addCapability(spv::CapabilityRayTracingNV);
            builder.addExtension("SPV_NV_ray_tracing");
        }
	if (glslangIntermediate->getStage() != EShLangRayGen && glslangIntermediate->getStage() != EShLangCallable)
	{
		if (extensions.find("GL_EXT_ray_cull_mask") != extensions.end()) {
			builder.addCapability(spv::CapabilityRayCullMaskKHR);
			builder.addExtension("SPV_KHR_ray_cull_mask");
		}
	}
        break;
    }
    case EShLangTaskNV:
    case EShLangMeshNV:
        builder.addCapability(spv::CapabilityMeshShadingNV);
        builder.addExtension(spv::E_SPV_NV_mesh_shader);
        if (glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_6) {
            std::vector<spv::Id> dimConstId;
            for (int dim = 0; dim < 3; ++dim) {
                bool specConst = (glslangIntermediate->getLocalSizeSpecId(dim) != qglslang::TQualifier::layoutNotSet);
                dimConstId.push_back(builder.makeUintConstant(glslangIntermediate->getLocalSize(dim), specConst));
                if (specConst) {
                    builder.addDecoration(dimConstId.back(), spv::DecorationSpecId,
                                          glslangIntermediate->getLocalSizeSpecId(dim));
                }
            }
            builder.addExecutionModeId(shaderEntry, spv::ExecutionModeLocalSizeId, dimConstId);
        } else {
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeLocalSize, glslangIntermediate->getLocalSize(0),
                                                                               glslangIntermediate->getLocalSize(1),
                                                                               glslangIntermediate->getLocalSize(2));
        }
        if (glslangIntermediate->getStage() == EShLangMeshNV) {
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeOutputVertices,
                glslangIntermediate->getVertices());
            builder.addExecutionMode(shaderEntry, spv::ExecutionModeOutputPrimitivesNV,
                glslangIntermediate->getPrimitives());

            switch (glslangIntermediate->getOutputPrimitive()) {
            case qglslang::ElgPoints:        mode = spv::ExecutionModeOutputPoints;      break;
            case qglslang::ElgLines:         mode = spv::ExecutionModeOutputLinesNV;     break;
            case qglslang::ElgTriangles:     mode = spv::ExecutionModeOutputTrianglesNV; break;
            default:                        mode = spv::ExecutionModeMax;               break;
            }
            if (mode != spv::ExecutionModeMax)
                builder.addExecutionMode(shaderEntry, (spv::ExecutionMode)mode);
        }
        break;
#endif

    default:
        break;
    }

#ifndef GLSLANG_WEB
    //
    // Add SPIR-V requirements (GL_EXT_spirv_intrinsics)
    //
    if (glslangIntermediate->hasSpirvRequirement()) {
        const qglslang::TSpirvRequirement& spirvRequirement = glslangIntermediate->getSpirvRequirement();

        // Add SPIR-V extension requirement
        for (auto& extension : spirvRequirement.extensions)
            builder.addExtension(extension.c_str());

        // Add SPIR-V capability requirement
        for (auto capability : spirvRequirement.capabilities)
            builder.addCapability(static_cast<spv::Capability>(capability));
    }

    //
    // Add SPIR-V execution mode qualifiers (GL_EXT_spirv_intrinsics)
    //
    if (glslangIntermediate->hasSpirvExecutionMode()) {
        const qglslang::TSpirvExecutionMode spirvExecutionMode = glslangIntermediate->getSpirvExecutionMode();

        // Add spirv_execution_mode
        for (auto& mode : spirvExecutionMode.modes) {
            if (!mode.second.empty()) {
                std::vector<unsigned> literals;
                TranslateLiterals(mode.second, literals);
                builder.addExecutionMode(shaderEntry, static_cast<spv::ExecutionMode>(mode.first), literals);
            } else
                builder.addExecutionMode(shaderEntry, static_cast<spv::ExecutionMode>(mode.first));
        }

        // Add spirv_execution_mode_id
        for (auto& modeId : spirvExecutionMode.modeIds) {
            std::vector<spv::Id> operandIds;
            assert(!modeId.second.empty());
            for (auto extraOperand : modeId.second) {
                if (extraOperand->getType().getQualifier().isSpecConstant())
                    operandIds.push_back(getSymbolId(extraOperand->getAsSymbolNode()));
                else
                    operandIds.push_back(createSpvConstant(*extraOperand));
            }
            builder.addExecutionModeId(shaderEntry, static_cast<spv::ExecutionMode>(modeId.first), operandIds);
        }
    }
#endif
}

// Finish creating SPV, after the traversal is complete.
void TGlslangToSpvTraverser::finishSpv()
{
    // Finish the entry point function
    if (! entryPointTerminated) {
        builder.setBuildPoint(shaderEntry->getLastBlock());
        builder.leaveFunction();
    }

    // finish off the entry-point SPV instruction by adding the Input/Output <id>
    for (auto it = iOSet.cbegin(); it != iOSet.cend(); ++it)
        entryPoint->addIdOperand(*it);

    // Add capabilities, extensions, remove unneeded decorations, etc.,
    // based on the resulting SPIR-V.
    // Note: WebGPU code generation must have the opportunity to aggressively
    // prune unreachable merge blocks and continue targets.
    builder.postProcess();
}

// Write the SPV into 'out'.
void TGlslangToSpvTraverser::dumpSpv(std::vector<unsigned int>& out)
{
    builder.dump(out);
}

//
// Implement the traversal functions.
//
// Return true from interior nodes to have the external traversal
// continue on to children.  Return false if children were
// already processed.
//

//
// Symbols can turn into
//  - uniform/input reads
//  - output writes
//  - complex lvalue base setups:  foo.bar[3]....  , where we see foo and start up an access chain
//  - something simple that degenerates into the last bullet
//
void TGlslangToSpvTraverser::visitSymbol(qglslang::TIntermSymbol* symbol)
{
    SpecConstantOpModeGuard spec_constant_op_mode_setter(&builder);
    if (symbol->getType().isStruct())
        glslangTypeToIdMap[symbol->getType().getStruct()] = symbol->getId();

    if (symbol->getType().getQualifier().isSpecConstant())
        spec_constant_op_mode_setter.turnOnSpecConstantOpMode();

#ifdef ENABLE_HLSL
    // Skip symbol handling if it is string-typed
    if (symbol->getBasicType() == qglslang::EbtString)
        return;
#endif

    // getSymbolId() will set up all the IO decorations on the first call.
    // Formal function parameters were mapped during makeFunctions().
    spv::Id id = getSymbolId(symbol);

    if (builder.isPointer(id)) {
        if (!symbol->getType().getQualifier().isParamInput() &&
            !symbol->getType().getQualifier().isParamOutput()) {
            // Include all "static use" and "linkage only" interface variables on the OpEntryPoint instruction
            // Consider adding to the OpEntryPoint interface list.
            // Only looking at structures if they have at least one member.
            if (!symbol->getType().isStruct() || symbol->getType().getStruct()->size() > 0) {
                spv::StorageClass sc = builder.getStorageClass(id);
                // Before SPIR-V 1.4, we only want to include Input and Output.
                // Starting with SPIR-V 1.4, we want all globals.
                if ((glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_4 && builder.isGlobalStorage(id)) ||
                    (sc == spv::StorageClassInput || sc == spv::StorageClassOutput)) {
                    iOSet.insert(id);
                }
            }
        }

        // If the SPIR-V type is required to be different than the AST type
        // (for ex SubgroupMasks or 3x4 ObjectToWorld/WorldToObject matrices),
        // translate now from the SPIR-V type to the AST type, for the consuming
        // operation.
        // Note this turns it from an l-value to an r-value.
        // Currently, all symbols needing this are inputs; avoid the map lookup when non-input.
        if (symbol->getType().getQualifier().storage == qglslang::EvqVaryingIn)
            id = translateForcedType(id);
    }

    // Only process non-linkage-only nodes for generating actual static uses
    if (! linkageOnly || symbol->getQualifier().isSpecConstant()) {
        // Prepare to generate code for the access

        // L-value chains will be computed left to right.  We're on the symbol now,
        // which is the left-most part of the access chain, so now is "clear" time,
        // followed by setting the base.
        builder.clearAccessChain();

        // For now, we consider all user variables as being in memory, so they are pointers,
        // except for
        // A) R-Value arguments to a function, which are an intermediate object.
        //    See comments in handleUserFunctionCall().
        // B) Specialization constants (normal constants don't even come in as a variable),
        //    These are also pure R-values.
        // C) R-Values from type translation, see above call to translateForcedType()
        qglslang::TQualifier qualifier = symbol->getQualifier();
        if (qualifier.isSpecConstant() || rValueParameters.find(symbol->getId()) != rValueParameters.end() ||
            !builder.isPointerType(builder.getTypeId(id)))
            builder.setAccessChainRValue(id);
        else
            builder.setAccessChainLValue(id);
    }

#ifdef ENABLE_HLSL
    // Process linkage-only nodes for any special additional interface work.
    if (linkageOnly) {
        if (glslangIntermediate->getHlslFunctionality1()) {
            // Map implicit counter buffers to their originating buffers, which should have been
            // seen by now, given earlier pruning of unused counters, and preservation of order
            // of declaration.
            if (symbol->getType().getQualifier().isUniformOrBuffer()) {
                if (!glslangIntermediate->hasCounterBufferName(symbol->getName())) {
                    // Save possible originating buffers for counter buffers, keyed by
                    // making the potential counter-buffer name.
                    std::string keyName = symbol->getName().c_str();
                    keyName = glslangIntermediate->addCounterBufferName(keyName);
                    counterOriginator[keyName] = symbol;
                } else {
                    // Handle a counter buffer, by finding the saved originating buffer.
                    std::string keyName = symbol->getName().c_str();
                    auto it = counterOriginator.find(keyName);
                    if (it != counterOriginator.end()) {
                        id = getSymbolId(it->second);
                        if (id != spv::NoResult) {
                            spv::Id counterId = getSymbolId(symbol);
                            if (counterId != spv::NoResult) {
                                builder.addExtension("SPV_GOOGLE_hlsl_functionality1");
                                builder.addDecorationId(id, spv::DecorationHlslCounterBufferGOOGLE, counterId);
                            }
                        }
                    }
                }
            }
        }
    }
#endif
}

bool TGlslangToSpvTraverser::visitBinary(qglslang::TVisit /* visit */, qglslang::TIntermBinary* node)
{
    builder.setLine(node->getLoc().line, node->getLoc().getFilename());
    if (node->getLeft()->getAsSymbolNode() != nullptr && node->getLeft()->getType().isStruct()) {
        glslangTypeToIdMap[node->getLeft()->getType().getStruct()] = node->getLeft()->getAsSymbolNode()->getId();
    }
    if (node->getRight()->getAsSymbolNode() != nullptr && node->getRight()->getType().isStruct()) {
        glslangTypeToIdMap[node->getRight()->getType().getStruct()] = node->getRight()->getAsSymbolNode()->getId();
    }

    SpecConstantOpModeGuard spec_constant_op_mode_setter(&builder);
    if (node->getType().getQualifier().isSpecConstant())
        spec_constant_op_mode_setter.turnOnSpecConstantOpMode();

    // First, handle special cases
    switch (node->getOp()) {
    case qglslang::EOpAssign:
    case qglslang::EOpAddAssign:
    case qglslang::EOpSubAssign:
    case qglslang::EOpMulAssign:
    case qglslang::EOpVectorTimesMatrixAssign:
    case qglslang::EOpVectorTimesScalarAssign:
    case qglslang::EOpMatrixTimesScalarAssign:
    case qglslang::EOpMatrixTimesMatrixAssign:
    case qglslang::EOpDivAssign:
    case qglslang::EOpModAssign:
    case qglslang::EOpAndAssign:
    case qglslang::EOpInclusiveOrAssign:
    case qglslang::EOpExclusiveOrAssign:
    case qglslang::EOpLeftShiftAssign:
    case qglslang::EOpRightShiftAssign:
        // A bin-op assign "a += b" means the same thing as "a = a + b"
        // where a is evaluated before b. For a simple assignment, GLSL
        // says to evaluate the left before the right.  So, always, left
        // node then right node.
        {
            // get the left l-value, save it away
            builder.clearAccessChain();
            node->getLeft()->traverse(this);
            spv::Builder::AccessChain lValue = builder.getAccessChain();

            // evaluate the right
            builder.clearAccessChain();
            node->getRight()->traverse(this);
            spv::Id rValue = accessChainLoad(node->getRight()->getType());

            if (node->getOp() != qglslang::EOpAssign) {
                // the left is also an r-value
                builder.setAccessChain(lValue);
                spv::Id leftRValue = accessChainLoad(node->getLeft()->getType());

                // do the operation
                spv::Builder::AccessChain::CoherentFlags coherentFlags = TranslateCoherent(node->getLeft()->getType());
                coherentFlags |= TranslateCoherent(node->getRight()->getType());
                OpDecorations decorations = { TranslatePrecisionDecoration(node->getOperationPrecision()),
                                              TranslateNoContractionDecoration(node->getType().getQualifier()),
                                              TranslateNonUniformDecoration(coherentFlags) };
                rValue = createBinaryOperation(node->getOp(), decorations,
                                               convertGlslangToSpvType(node->getType()), leftRValue, rValue,
                                               node->getType().getBasicType());

                // these all need their counterparts in createBinaryOperation()
                assert(rValue != spv::NoResult);
            }

            // store the result
            builder.setAccessChain(lValue);
            multiTypeStore(node->getLeft()->getType(), rValue);

            // assignments are expressions having an rValue after they are evaluated...
            builder.clearAccessChain();
            builder.setAccessChainRValue(rValue);
        }
        return false;
    case qglslang::EOpIndexDirect:
    case qglslang::EOpIndexDirectStruct:
        {
            // Structure, array, matrix, or vector indirection with statically known index.
            // Get the left part of the access chain.
            node->getLeft()->traverse(this);

            // Add the next element in the chain

            const int glslangIndex = node->getRight()->getAsConstantUnion()->getConstArray()[0].getIConst();
            if (! node->getLeft()->getType().isArray() &&
                node->getLeft()->getType().isVector() &&
                node->getOp() == qglslang::EOpIndexDirect) {
                // Swizzle is uniform so propagate uniform into access chain
                spv::Builder::AccessChain::CoherentFlags coherentFlags = TranslateCoherent(node->getLeft()->getType());
                coherentFlags.nonUniform = 0;
                // This is essentially a hard-coded vector swizzle of size 1,
                // so short circuit the access-chain stuff with a swizzle.
                std::vector<unsigned> swizzle;
                swizzle.push_back(glslangIndex);
                int dummySize;
                builder.accessChainPushSwizzle(swizzle, convertGlslangToSpvType(node->getLeft()->getType()),
                                               coherentFlags,
                                               glslangIntermediate->getBaseAlignmentScalar(
                                                   node->getLeft()->getType(), dummySize));
            } else {

                // Load through a block reference is performed with a dot operator that
                // is mapped to EOpIndexDirectStruct. When we get to the actual reference,
                // do a load and reset the access chain.
                if (node->getLeft()->isReference() &&
                    !node->getLeft()->getType().isArray() &&
                    node->getOp() == qglslang::EOpIndexDirectStruct)
                {
                    spv::Id left = accessChainLoad(node->getLeft()->getType());
                    builder.clearAccessChain();
                    builder.setAccessChainLValue(left);
                }

                int spvIndex = glslangIndex;
                if (node->getLeft()->getBasicType() == qglslang::EbtBlock &&
                    node->getOp() == qglslang::EOpIndexDirectStruct)
                {
                    // This may be, e.g., an anonymous block-member selection, which generally need
                    // index remapping due to hidden members in anonymous blocks.
                    long long glslangId = glslangTypeToIdMap[node->getLeft()->getType().getStruct()];
                    if (memberRemapper.find(glslangId) != memberRemapper.end()) {
                        std::vector<int>& remapper = memberRemapper[glslangId];
                        assert(remapper.size() > 0);
                        spvIndex = remapper[glslangIndex];
                    }
                }

                // Struct reference propagates uniform lvalue
                spv::Builder::AccessChain::CoherentFlags coherentFlags =
                        TranslateCoherent(node->getLeft()->getType());
                coherentFlags.nonUniform = 0;

                // normal case for indexing array or structure or block
                builder.accessChainPush(builder.makeIntConstant(spvIndex),
                        coherentFlags,
                        node->getLeft()->getType().getBufferReferenceAlignment());

                // Add capabilities here for accessing PointSize and clip/cull distance.
                // We have deferred generation of associated capabilities until now.
                if (node->getLeft()->getType().isStruct() && ! node->getLeft()->getType().isArray())
                    declareUseOfStructMember(*(node->getLeft()->getType().getStruct()), glslangIndex);
            }
        }
        return false;
    case qglslang::EOpIndexIndirect:
        {
            // Array, matrix, or vector indirection with variable index.
            // Will use native SPIR-V access-chain for and array indirection;
            // matrices are arrays of vectors, so will also work for a matrix.
            // Will use the access chain's 'component' for variable index into a vector.

            // This adapter is building access chains left to right.
            // Set up the access chain to the left.
            node->getLeft()->traverse(this);

            // save it so that computing the right side doesn't trash it
            spv::Builder::AccessChain partial = builder.getAccessChain();

            // compute the next index in the chain
            builder.clearAccessChain();
            node->getRight()->traverse(this);
            spv::Id index = accessChainLoad(node->getRight()->getType());

            addIndirectionIndexCapabilities(node->getLeft()->getType(), node->getRight()->getType());

            // restore the saved access chain
            builder.setAccessChain(partial);

            // Only if index is nonUniform should we propagate nonUniform into access chain
            spv::Builder::AccessChain::CoherentFlags index_flags = TranslateCoherent(node->getRight()->getType());
            spv::Builder::AccessChain::CoherentFlags coherent_flags = TranslateCoherent(node->getLeft()->getType());
            coherent_flags.nonUniform = index_flags.nonUniform;

            if (! node->getLeft()->getType().isArray() && node->getLeft()->getType().isVector()) {
                int dummySize;
                builder.accessChainPushComponent(
                    index, convertGlslangToSpvType(node->getLeft()->getType()), coherent_flags,
                                                glslangIntermediate->getBaseAlignmentScalar(node->getLeft()->getType(),
                                                dummySize));
            } else
                builder.accessChainPush(index, coherent_flags,
                                        node->getLeft()->getType().getBufferReferenceAlignment());
        }
        return false;
    case qglslang::EOpVectorSwizzle:
        {
            node->getLeft()->traverse(this);
            std::vector<unsigned> swizzle;
            convertSwizzle(*node->getRight()->getAsAggregate(), swizzle);
            int dummySize;
            builder.accessChainPushSwizzle(swizzle, convertGlslangToSpvType(node->getLeft()->getType()),
                                           TranslateCoherent(node->getLeft()->getType()),
                                           glslangIntermediate->getBaseAlignmentScalar(node->getLeft()->getType(),
                                               dummySize));
        }
        return false;
    case qglslang::EOpMatrixSwizzle:
        logger->missingFunctionality("matrix swizzle");
        return true;
    case qglslang::EOpLogicalOr:
    case qglslang::EOpLogicalAnd:
        {

            // These may require short circuiting, but can sometimes be done as straight
            // binary operations.  The right operand must be short circuited if it has
            // side effects, and should probably be if it is complex.
            if (isTrivial(node->getRight()->getAsTyped()))
                break; // handle below as a normal binary operation
            // otherwise, we need to do dynamic short circuiting on the right operand
            spv::Id result = createShortCircuit(node->getOp(), *node->getLeft()->getAsTyped(),
                *node->getRight()->getAsTyped());
            builder.clearAccessChain();
            builder.setAccessChainRValue(result);
        }
        return false;
    default:
        break;
    }

    // Assume generic binary op...

    // get right operand
    builder.clearAccessChain();
    node->getLeft()->traverse(this);
    spv::Id left = accessChainLoad(node->getLeft()->getType());

    // get left operand
    builder.clearAccessChain();
    node->getRight()->traverse(this);
    spv::Id right = accessChainLoad(node->getRight()->getType());

    // get result
    OpDecorations decorations = { TranslatePrecisionDecoration(node->getOperationPrecision()),
                                  TranslateNoContractionDecoration(node->getType().getQualifier()),
                                  TranslateNonUniformDecoration(node->getType().getQualifier()) };
    spv::Id result = createBinaryOperation(node->getOp(), decorations,
                                           convertGlslangToSpvType(node->getType()), left, right,
                                           node->getLeft()->getType().getBasicType());

    builder.clearAccessChain();
    if (! result) {
        logger->missingFunctionality("unknown glslang binary operation");
        return true;  // pick up a child as the place-holder result
    } else {
        builder.setAccessChainRValue(result);
        return false;
    }
}

spv::Id TGlslangToSpvTraverser::convertLoadedBoolInUniformToUint(const qglslang::TType& type,
                                                                 spv::Id nominalTypeId,
                                                                 spv::Id loadedId)
{
    if (builder.isScalarType(nominalTypeId)) {
        // Conversion for bool
        spv::Id boolType = builder.makeBoolType();
        if (nominalTypeId != boolType)
            return builder.createBinOp(spv::OpINotEqual, boolType, loadedId, builder.makeUintConstant(0));
    } else if (builder.isVectorType(nominalTypeId)) {
        // Conversion for bvec
        int vecSize = builder.getNumTypeComponents(nominalTypeId);
        spv::Id bvecType = builder.makeVectorType(builder.makeBoolType(), vecSize);
        if (nominalTypeId != bvecType)
            loadedId = builder.createBinOp(spv::OpINotEqual, bvecType, loadedId,
                makeSmearedConstant(builder.makeUintConstant(0), vecSize));
    } else if (builder.isArrayType(nominalTypeId)) {
        // Conversion for bool array
        spv::Id boolArrayTypeId = convertGlslangToSpvType(type);
        if (nominalTypeId != boolArrayTypeId)
        {
            // Use OpCopyLogical from SPIR-V 1.4 if available.
            if (glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_4)
                return builder.createUnaryOp(spv::OpCopyLogical, boolArrayTypeId, loadedId);

            qglslang::TType glslangElementType(type, 0);
            spv::Id elementNominalTypeId = builder.getContainedTypeId(nominalTypeId);
            std::vector<spv::Id> constituents;
            for (int index = 0; index < type.getOuterArraySize(); ++index) {
                // get the element
                spv::Id elementValue = builder.createCompositeExtract(loadedId, elementNominalTypeId, index);

                // recursively convert it
                spv::Id elementConvertedValue = convertLoadedBoolInUniformToUint(glslangElementType, elementNominalTypeId, elementValue);
                constituents.push_back(elementConvertedValue);
            }
            return builder.createCompositeConstruct(boolArrayTypeId, constituents);
        }
    }

    return loadedId;
}

// Figure out what, if any, type changes are needed when accessing a specific built-in.
// Returns <the type SPIR-V requires for declarion, the type to translate to on use>.
// Also see comment for 'forceType', regarding tracking SPIR-V-required types.
std::pair<spv::Id, spv::Id> TGlslangToSpvTraverser::getForcedType(qglslang::TBuiltInVariable glslangBuiltIn,
    const qglslang::TType& glslangType)
{
    switch(glslangBuiltIn)
    {
        case qglslang::EbvSubGroupEqMask:
        case qglslang::EbvSubGroupGeMask:
        case qglslang::EbvSubGroupGtMask:
        case qglslang::EbvSubGroupLeMask:
        case qglslang::EbvSubGroupLtMask: {
            // these require changing a 64-bit scaler -> a vector of 32-bit components
            if (glslangType.isVector())
                break;
            spv::Id ivec4_type = builder.makeVectorType(builder.makeUintType(32), 4);
            spv::Id uint64_type = builder.makeUintType(64);
            std::pair<spv::Id, spv::Id> ret(ivec4_type, uint64_type);
            return ret;
        }
        // There are no SPIR-V builtins defined for these and map onto original non-transposed
        // builtins. During visitBinary we insert a transpose
        case qglslang::EbvWorldToObject3x4:
        case qglslang::EbvObjectToWorld3x4: {
            spv::Id mat43 = builder.makeMatrixType(builder.makeFloatType(32), 4, 3);
            spv::Id mat34 = builder.makeMatrixType(builder.makeFloatType(32), 3, 4);
            std::pair<spv::Id, spv::Id> ret(mat43, mat34);
            return ret;
        }
        default:
            break;
    }

    std::pair<spv::Id, spv::Id> ret(spv::NoType, spv::NoType);
    return ret;
}

// For an object previously identified (see getForcedType() and forceType)
// as needing type translations, do the translation needed for a load, turning
// an L-value into in R-value.
spv::Id TGlslangToSpvTraverser::translateForcedType(spv::Id object)
{
    const auto forceIt = forceType.find(object);
    if (forceIt == forceType.end())
        return object;

    spv::Id desiredTypeId = forceIt->second;
    spv::Id objectTypeId = builder.getTypeId(object);
    assert(builder.isPointerType(objectTypeId));
    objectTypeId = builder.getContainedTypeId(objectTypeId);
    if (builder.isVectorType(objectTypeId) &&
        builder.getScalarTypeWidth(builder.getContainedTypeId(objectTypeId)) == 32) {
        if (builder.getScalarTypeWidth(desiredTypeId) == 64) {
            // handle 32-bit v.xy* -> 64-bit
            builder.clearAccessChain();
            builder.setAccessChainLValue(object);
            object = builder.accessChainLoad(spv::NoPrecision, spv::DecorationMax, spv::DecorationMax, objectTypeId);
            std::vector<spv::Id> components;
            components.push_back(builder.createCompositeExtract(object, builder.getContainedTypeId(objectTypeId), 0));
            components.push_back(builder.createCompositeExtract(object, builder.getContainedTypeId(objectTypeId), 1));

            spv::Id vecType = builder.makeVectorType(builder.getContainedTypeId(objectTypeId), 2);
            return builder.createUnaryOp(spv::OpBitcast, desiredTypeId,
                                         builder.createCompositeConstruct(vecType, components));
        } else {
            logger->missingFunctionality("forcing 32-bit vector type to non 64-bit scalar");
        }
    } else if (builder.isMatrixType(objectTypeId)) {
            // There are no SPIR-V builtins defined for 3x4 variants of ObjectToWorld/WorldToObject
            // and we insert a transpose after loading the original non-transposed builtins
            builder.clearAccessChain();
            builder.setAccessChainLValue(object);
            object = builder.accessChainLoad(spv::NoPrecision, spv::DecorationMax, spv::DecorationMax, objectTypeId);
            return builder.createUnaryOp(spv::OpTranspose, desiredTypeId, object);

    } else  {
        logger->missingFunctionality("forcing non 32-bit vector type");
    }

    return object;
}

bool TGlslangToSpvTraverser::visitUnary(qglslang::TVisit /* visit */, qglslang::TIntermUnary* node)
{
    builder.setLine(node->getLoc().line, node->getLoc().getFilename());

    SpecConstantOpModeGuard spec_constant_op_mode_setter(&builder);
    if (node->getType().getQualifier().isSpecConstant())
        spec_constant_op_mode_setter.turnOnSpecConstantOpMode();

    spv::Id result = spv::NoResult;

    // try texturing first
    result = createImageTextureFunctionCall(node);
    if (result != spv::NoResult) {
        builder.clearAccessChain();
        builder.setAccessChainRValue(result);

        return false; // done with this node
    }

    // Non-texturing.

    if (node->getOp() == qglslang::EOpArrayLength) {
        // Quite special; won't want to evaluate the operand.

        // Currently, the front-end does not allow .length() on an array until it is sized,
        // except for the last block membeor of an SSBO.
        // TODO: If this changes, link-time sized arrays might show up here, and need their
        // size extracted.

        // Normal .length() would have been constant folded by the front-end.
        // So, this has to be block.lastMember.length().
        // SPV wants "block" and member number as the operands, go get them.

        spv::Id length;
        if (node->getOperand()->getType().isCoopMat()) {
            spec_constant_op_mode_setter.turnOnSpecConstantOpMode();

            spv::Id typeId = convertGlslangToSpvType(node->getOperand()->getType());
            assert(builder.isCooperativeMatrixType(typeId));

            length = builder.createCooperativeMatrixLength(typeId);
        } else {
            qglslang::TIntermTyped* block = node->getOperand()->getAsBinaryNode()->getLeft();
            block->traverse(this);
            unsigned int member = node->getOperand()->getAsBinaryNode()->getRight()->getAsConstantUnion()
                ->getConstArray()[0].getUConst();
            length = builder.createArrayLength(builder.accessChainGetLValue(), member);
        }

        // GLSL semantics say the result of .length() is an int, while SPIR-V says
        // signedness must be 0. So, convert from SPIR-V unsigned back to GLSL's
        // AST expectation of a signed result.
        if (glslangIntermediate->getSource() == qglslang::EShSourceGlsl) {
            if (builder.isInSpecConstCodeGenMode()) {
                length = builder.createBinOp(spv::OpIAdd, builder.makeIntType(32), length, builder.makeIntConstant(0));
            } else {
                length = builder.createUnaryOp(spv::OpBitcast, builder.makeIntType(32), length);
            }
        }

        builder.clearAccessChain();
        builder.setAccessChainRValue(length);

        return false;
    }

    // Start by evaluating the operand

    // Does it need a swizzle inversion?  If so, evaluation is inverted;
    // operate first on the swizzle base, then apply the swizzle.
    spv::Id invertedType = spv::NoType;
    auto resultType = [&invertedType, &node, this](){ return invertedType != spv::NoType ?
        invertedType : convertGlslangToSpvType(node->getType()); };
    if (node->getOp() == qglslang::EOpInterpolateAtCentroid)
        invertedType = getInvertedSwizzleType(*node->getOperand());

    builder.clearAccessChain();
    TIntermNode *operandNode;
    if (invertedType != spv::NoType)
        operandNode = node->getOperand()->getAsBinaryNode()->getLeft();
    else
        operandNode = node->getOperand();
    
    operandNode->traverse(this);

    spv::Id operand = spv::NoResult;

    spv::Builder::AccessChain::CoherentFlags lvalueCoherentFlags;

#ifndef GLSLANG_WEB
    if (node->getOp() == qglslang::EOpAtomicCounterIncrement ||
        node->getOp() == qglslang::EOpAtomicCounterDecrement ||
        node->getOp() == qglslang::EOpAtomicCounter          ||
        (node->getOp() == qglslang::EOpInterpolateAtCentroid &&
          glslangIntermediate->getSource() != qglslang::EShSourceHlsl)  ||
        node->getOp() == qglslang::EOpRayQueryProceed        ||
        node->getOp() == qglslang::EOpRayQueryGetRayTMin     ||
        node->getOp() == qglslang::EOpRayQueryGetRayFlags    ||
        node->getOp() == qglslang::EOpRayQueryGetWorldRayOrigin ||
        node->getOp() == qglslang::EOpRayQueryGetWorldRayDirection ||
        node->getOp() == qglslang::EOpRayQueryGetIntersectionCandidateAABBOpaque ||
        node->getOp() == qglslang::EOpRayQueryTerminate ||
        node->getOp() == qglslang::EOpRayQueryConfirmIntersection ||
        (node->getOp() == qglslang::EOpSpirvInst && operandNode->getAsTyped()->getQualifier().isSpirvByReference())) {
        operand = builder.accessChainGetLValue(); // Special case l-value operands
        lvalueCoherentFlags = builder.getAccessChain().coherentFlags;
        lvalueCoherentFlags |= TranslateCoherent(operandNode->getAsTyped()->getType());
    } else if (operandNode->getAsTyped()->getQualifier().isSpirvLiteral()) {
        // Will be translated to a literal value, make a placeholder here
        operand = spv::NoResult;
    } else
#endif
    {
        operand = accessChainLoad(node->getOperand()->getType());
    }

    OpDecorations decorations = { TranslatePrecisionDecoration(node->getOperationPrecision()),
                                  TranslateNoContractionDecoration(node->getType().getQualifier()),
                                  TranslateNonUniformDecoration(node->getType().getQualifier()) };

    // it could be a conversion
    if (! result)
        result = createConversion(node->getOp(), decorations, resultType(), operand,
            node->getOperand()->getBasicType());

    // if not, then possibly an operation
    if (! result)
        result = createUnaryOperation(node->getOp(), decorations, resultType(), operand,
            node->getOperand()->getBasicType(), lvalueCoherentFlags);

#ifndef GLSLANG_WEB
    // it could be attached to a SPIR-V intruction
    if (!result) {
        if (node->getOp() == qglslang::EOpSpirvInst) {
            const auto& spirvInst = node->getSpirvInstruction();
            if (spirvInst.set == "") {
                spv::IdImmediate idImmOp = {true, operand};
                if (operandNode->getAsTyped()->getQualifier().isSpirvLiteral()) {
                    // Translate the constant to a literal value
                    std::vector<unsigned> literals;
                    qglslang::TVector<const qglslang::TIntermConstantUnion*> constants;
                    constants.push_back(operandNode->getAsConstantUnion());
                    TranslateLiterals(constants, literals);
                    idImmOp = {false, literals[0]};
                }

                if (node->getBasicType() == qglslang::EbtVoid)
                    builder.createNoResultOp(static_cast<spv::Op>(spirvInst.id), {idImmOp});
                else
                    result = builder.createOp(static_cast<spv::Op>(spirvInst.id), resultType(), {idImmOp});
            } else {
                result = builder.createBuiltinCall(
                    resultType(), spirvInst.set == "GLSL.std.450" ? stdBuiltins : getExtBuiltins(spirvInst.set.c_str()),
                    spirvInst.id, {operand});
            }

            if (node->getBasicType() == qglslang::EbtVoid)
                return false; // done with this node
        }
    }
#endif

    if (result) {
        if (invertedType) {
            result = createInvertedSwizzle(decorations.precision, *node->getOperand(), result);
            decorations.addNonUniform(builder, result);
        }

        builder.clearAccessChain();
        builder.setAccessChainRValue(result);

        return false; // done with this node
    }

    // it must be a special case, check...
    switch (node->getOp()) {
    case qglslang::EOpPostIncrement:
    case qglslang::EOpPostDecrement:
    case qglslang::EOpPreIncrement:
    case qglslang::EOpPreDecrement:
        {
            // we need the integer value "1" or the floating point "1.0" to add/subtract
            spv::Id one = 0;
            if (node->getBasicType() == qglslang::EbtFloat)
                one = builder.makeFloatConstant(1.0F);
#ifndef GLSLANG_WEB
            else if (node->getBasicType() == qglslang::EbtDouble)
                one = builder.makeDoubleConstant(1.0);
            else if (node->getBasicType() == qglslang::EbtFloat16)
                one = builder.makeFloat16Constant(1.0F);
            else if (node->getBasicType() == qglslang::EbtInt8  || node->getBasicType() == qglslang::EbtUint8)
                one = builder.makeInt8Constant(1);
            else if (node->getBasicType() == qglslang::EbtInt16 || node->getBasicType() == qglslang::EbtUint16)
                one = builder.makeInt16Constant(1);
            else if (node->getBasicType() == qglslang::EbtInt64 || node->getBasicType() == qglslang::EbtUint64)
                one = builder.makeInt64Constant(1);
#endif
            else
                one = builder.makeIntConstant(1);
            qglslang::TOperator op;
            if (node->getOp() == qglslang::EOpPreIncrement ||
                node->getOp() == qglslang::EOpPostIncrement)
                op = qglslang::EOpAdd;
            else
                op = qglslang::EOpSub;

            spv::Id result = createBinaryOperation(op, decorations,
                                                   convertGlslangToSpvType(node->getType()), operand, one,
                                                   node->getType().getBasicType());
            assert(result != spv::NoResult);

            // The result of operation is always stored, but conditionally the
            // consumed result.  The consumed result is always an r-value.
            builder.accessChainStore(result,
                                     TranslateNonUniformDecoration(builder.getAccessChain().coherentFlags));
            builder.clearAccessChain();
            if (node->getOp() == qglslang::EOpPreIncrement ||
                node->getOp() == qglslang::EOpPreDecrement)
                builder.setAccessChainRValue(result);
            else
                builder.setAccessChainRValue(operand);
        }

        return false;

#ifndef GLSLANG_WEB
    case qglslang::EOpEmitStreamVertex:
        builder.createNoResultOp(spv::OpEmitStreamVertex, operand);
        return false;
    case qglslang::EOpEndStreamPrimitive:
        builder.createNoResultOp(spv::OpEndStreamPrimitive, operand);
        return false;
    case qglslang::EOpRayQueryTerminate:
        builder.createNoResultOp(spv::OpRayQueryTerminateKHR, operand);
        return false;
    case qglslang::EOpRayQueryConfirmIntersection:
        builder.createNoResultOp(spv::OpRayQueryConfirmIntersectionKHR, operand);
        return false;
#endif

    default:
        logger->missingFunctionality("unknown glslang unary");
        return true;  // pick up operand as placeholder result
    }
}

// Construct a composite object, recursively copying members if their types don't match
spv::Id TGlslangToSpvTraverser::createCompositeConstruct(spv::Id resultTypeId, std::vector<spv::Id> constituents)
{
    for (int c = 0; c < (int)constituents.size(); ++c) {
        spv::Id& constituent = constituents[c];
        spv::Id lType = builder.getContainedTypeId(resultTypeId, c);
        spv::Id rType = builder.getTypeId(constituent);
        if (lType != rType) {
            if (glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_4) {
                constituent = builder.createUnaryOp(spv::OpCopyLogical, lType, constituent);
            } else if (builder.isStructType(rType)) {
                std::vector<spv::Id> rTypeConstituents;
                int numrTypeConstituents = builder.getNumTypeConstituents(rType);
                for (int i = 0; i < numrTypeConstituents; ++i) {
                    rTypeConstituents.push_back(builder.createCompositeExtract(constituent,
                        builder.getContainedTypeId(rType, i), i));
                }
                constituents[c] = createCompositeConstruct(lType, rTypeConstituents);
            } else {
                assert(builder.isArrayType(rType));
                std::vector<spv::Id> rTypeConstituents;
                int numrTypeConstituents = builder.getNumTypeConstituents(rType);

                spv::Id elementRType = builder.getContainedTypeId(rType);
                for (int i = 0; i < numrTypeConstituents; ++i) {
                    rTypeConstituents.push_back(builder.createCompositeExtract(constituent, elementRType, i));
                }
                constituents[c] = createCompositeConstruct(lType, rTypeConstituents);
            }
        }
    }
    return builder.createCompositeConstruct(resultTypeId, constituents);
}

bool TGlslangToSpvTraverser::visitAggregate(qglslang::TVisit visit, qglslang::TIntermAggregate* node)
{
    SpecConstantOpModeGuard spec_constant_op_mode_setter(&builder);
    if (node->getType().getQualifier().isSpecConstant())
        spec_constant_op_mode_setter.turnOnSpecConstantOpMode();

    spv::Id result = spv::NoResult;
    spv::Id invertedType = spv::NoType;                     // to use to override the natural type of the node
    std::vector<spv::Builder::AccessChain> complexLvalues;  // for holding swizzling l-values too complex for
                                                            // SPIR-V, for an out parameter
    std::vector<spv::Id> temporaryLvalues;                  // temporaries to pass, as proxies for complexLValues

    auto resultType = [&invertedType, &node, this](){ return invertedType != spv::NoType ?
        invertedType :
        convertGlslangToSpvType(node->getType()); };

    // try texturing
    result = createImageTextureFunctionCall(node);
    if (result != spv::NoResult) {
        builder.clearAccessChain();
        builder.setAccessChainRValue(result);

        return false;
    }
#ifndef GLSLANG_WEB
    else if (node->getOp() == qglslang::EOpImageStore ||
        node->getOp() == qglslang::EOpImageStoreLod ||
        node->getOp() == qglslang::EOpImageAtomicStore) {
        // "imageStore" is a special case, which has no result
        return false;
    }
#endif

    qglslang::TOperator binOp = qglslang::EOpNull;
    bool reduceComparison = true;
    bool isMatrix = false;
    bool noReturnValue = false;
    bool atomic = false;

    spv::Builder::AccessChain::CoherentFlags lvalueCoherentFlags;

    assert(node->getOp());

    spv::Decoration precision = TranslatePrecisionDecoration(node->getOperationPrecision());

    switch (node->getOp()) {
    case qglslang::EOpSequence:
    {
        if (preVisit)
            ++sequenceDepth;
        else
            --sequenceDepth;

        if (sequenceDepth == 1) {
            // If this is the parent node of all the functions, we want to see them
            // early, so all call points have actual SPIR-V functions to reference.
            // In all cases, still let the traverser visit the children for us.
            makeFunctions(node->getAsAggregate()->getSequence());

            // Also, we want all globals initializers to go into the beginning of the entry point, before
            // anything else gets there, so visit out of order, doing them all now.
            makeGlobalInitializers(node->getAsAggregate()->getSequence());

            //Pre process linker objects for ray tracing stages
            if (glslangIntermediate->isRayTracingStage())
                collectRayTracingLinkerObjects();

            // Initializers are done, don't want to visit again, but functions and link objects need to be processed,
            // so do them manually.
            visitFunctions(node->getAsAggregate()->getSequence());

            return false;
        }

        return true;
    }
    case qglslang::EOpLinkerObjects:
    {
        if (visit == qglslang::EvPreVisit)
            linkageOnly = true;
        else
            linkageOnly = false;

        return true;
    }
    case qglslang::EOpComma:
    {
        // processing from left to right naturally leaves the right-most
        // lying around in the access chain
        qglslang::TIntermSequence& glslangOperands = node->getSequence();
        for (int i = 0; i < (int)glslangOperands.size(); ++i)
            glslangOperands[i]->traverse(this);

        return false;
    }
    case qglslang::EOpFunction:
        if (visit == qglslang::EvPreVisit) {
            if (isShaderEntryPoint(node)) {
                inEntryPoint = true;
                builder.setBuildPoint(shaderEntry->getLastBlock());
                currentFunction = shaderEntry;
            } else {
                handleFunctionEntry(node);
            }
            if (options.generateDebugInfo) {
                const auto& loc = node->getLoc();
                currentFunction->setDebugLineInfo(builder.getSourceFile(), loc.line, loc.column);
            }
        } else {
            if (inEntryPoint)
                entryPointTerminated = true;
            builder.leaveFunction();
            inEntryPoint = false;
        }

        return true;
    case qglslang::EOpParameters:
        // Parameters will have been consumed by EOpFunction processing, but not
        // the body, so we still visited the function node's children, making this
        // child redundant.
        return false;
    case qglslang::EOpFunctionCall:
    {
        builder.setLine(node->getLoc().line, node->getLoc().getFilename());
        if (node->isUserDefined())
            result = handleUserFunctionCall(node);
        if (result) {
            builder.clearAccessChain();
            builder.setAccessChainRValue(result);
        } else
            logger->missingFunctionality("missing user function; linker needs to catch that");

        return false;
    }
    case qglslang::EOpConstructMat2x2:
    case qglslang::EOpConstructMat2x3:
    case qglslang::EOpConstructMat2x4:
    case qglslang::EOpConstructMat3x2:
    case qglslang::EOpConstructMat3x3:
    case qglslang::EOpConstructMat3x4:
    case qglslang::EOpConstructMat4x2:
    case qglslang::EOpConstructMat4x3:
    case qglslang::EOpConstructMat4x4:
    case qglslang::EOpConstructDMat2x2:
    case qglslang::EOpConstructDMat2x3:
    case qglslang::EOpConstructDMat2x4:
    case qglslang::EOpConstructDMat3x2:
    case qglslang::EOpConstructDMat3x3:
    case qglslang::EOpConstructDMat3x4:
    case qglslang::EOpConstructDMat4x2:
    case qglslang::EOpConstructDMat4x3:
    case qglslang::EOpConstructDMat4x4:
    case qglslang::EOpConstructIMat2x2:
    case qglslang::EOpConstructIMat2x3:
    case qglslang::EOpConstructIMat2x4:
    case qglslang::EOpConstructIMat3x2:
    case qglslang::EOpConstructIMat3x3:
    case qglslang::EOpConstructIMat3x4:
    case qglslang::EOpConstructIMat4x2:
    case qglslang::EOpConstructIMat4x3:
    case qglslang::EOpConstructIMat4x4:
    case qglslang::EOpConstructUMat2x2:
    case qglslang::EOpConstructUMat2x3:
    case qglslang::EOpConstructUMat2x4:
    case qglslang::EOpConstructUMat3x2:
    case qglslang::EOpConstructUMat3x3:
    case qglslang::EOpConstructUMat3x4:
    case qglslang::EOpConstructUMat4x2:
    case qglslang::EOpConstructUMat4x3:
    case qglslang::EOpConstructUMat4x4:
    case qglslang::EOpConstructBMat2x2:
    case qglslang::EOpConstructBMat2x3:
    case qglslang::EOpConstructBMat2x4:
    case qglslang::EOpConstructBMat3x2:
    case qglslang::EOpConstructBMat3x3:
    case qglslang::EOpConstructBMat3x4:
    case qglslang::EOpConstructBMat4x2:
    case qglslang::EOpConstructBMat4x3:
    case qglslang::EOpConstructBMat4x4:
    case qglslang::EOpConstructF16Mat2x2:
    case qglslang::EOpConstructF16Mat2x3:
    case qglslang::EOpConstructF16Mat2x4:
    case qglslang::EOpConstructF16Mat3x2:
    case qglslang::EOpConstructF16Mat3x3:
    case qglslang::EOpConstructF16Mat3x4:
    case qglslang::EOpConstructF16Mat4x2:
    case qglslang::EOpConstructF16Mat4x3:
    case qglslang::EOpConstructF16Mat4x4:
        isMatrix = true;
        // fall through
    case qglslang::EOpConstructFloat:
    case qglslang::EOpConstructVec2:
    case qglslang::EOpConstructVec3:
    case qglslang::EOpConstructVec4:
    case qglslang::EOpConstructDouble:
    case qglslang::EOpConstructDVec2:
    case qglslang::EOpConstructDVec3:
    case qglslang::EOpConstructDVec4:
    case qglslang::EOpConstructFloat16:
    case qglslang::EOpConstructF16Vec2:
    case qglslang::EOpConstructF16Vec3:
    case qglslang::EOpConstructF16Vec4:
    case qglslang::EOpConstructBool:
    case qglslang::EOpConstructBVec2:
    case qglslang::EOpConstructBVec3:
    case qglslang::EOpConstructBVec4:
    case qglslang::EOpConstructInt8:
    case qglslang::EOpConstructI8Vec2:
    case qglslang::EOpConstructI8Vec3:
    case qglslang::EOpConstructI8Vec4:
    case qglslang::EOpConstructUint8:
    case qglslang::EOpConstructU8Vec2:
    case qglslang::EOpConstructU8Vec3:
    case qglslang::EOpConstructU8Vec4:
    case qglslang::EOpConstructInt16:
    case qglslang::EOpConstructI16Vec2:
    case qglslang::EOpConstructI16Vec3:
    case qglslang::EOpConstructI16Vec4:
    case qglslang::EOpConstructUint16:
    case qglslang::EOpConstructU16Vec2:
    case qglslang::EOpConstructU16Vec3:
    case qglslang::EOpConstructU16Vec4:
    case qglslang::EOpConstructInt:
    case qglslang::EOpConstructIVec2:
    case qglslang::EOpConstructIVec3:
    case qglslang::EOpConstructIVec4:
    case qglslang::EOpConstructUint:
    case qglslang::EOpConstructUVec2:
    case qglslang::EOpConstructUVec3:
    case qglslang::EOpConstructUVec4:
    case qglslang::EOpConstructInt64:
    case qglslang::EOpConstructI64Vec2:
    case qglslang::EOpConstructI64Vec3:
    case qglslang::EOpConstructI64Vec4:
    case qglslang::EOpConstructUint64:
    case qglslang::EOpConstructU64Vec2:
    case qglslang::EOpConstructU64Vec3:
    case qglslang::EOpConstructU64Vec4:
    case qglslang::EOpConstructStruct:
    case qglslang::EOpConstructTextureSampler:
    case qglslang::EOpConstructReference:
    case qglslang::EOpConstructCooperativeMatrix:
    {
        builder.setLine(node->getLoc().line, node->getLoc().getFilename());
        std::vector<spv::Id> arguments;
        translateArguments(*node, arguments, lvalueCoherentFlags);
        spv::Id constructed;
        if (node->getOp() == qglslang::EOpConstructTextureSampler)
            constructed = builder.createOp(spv::OpSampledImage, resultType(), arguments);
        else if (node->getOp() == qglslang::EOpConstructStruct ||
                 node->getOp() == qglslang::EOpConstructCooperativeMatrix ||
                 node->getType().isArray()) {
            std::vector<spv::Id> constituents;
            for (int c = 0; c < (int)arguments.size(); ++c)
                constituents.push_back(arguments[c]);
            constructed = createCompositeConstruct(resultType(), constituents);
        } else if (isMatrix)
            constructed = builder.createMatrixConstructor(precision, arguments, resultType());
        else
            constructed = builder.createConstructor(precision, arguments, resultType());

        if (node->getType().getQualifier().isNonUniform()) {
            builder.addDecoration(constructed, spv::DecorationNonUniformEXT);
        }

        builder.clearAccessChain();
        builder.setAccessChainRValue(constructed);

        return false;
    }

    // These six are component-wise compares with component-wise results.
    // Forward on to createBinaryOperation(), requesting a vector result.
    case qglslang::EOpLessThan:
    case qglslang::EOpGreaterThan:
    case qglslang::EOpLessThanEqual:
    case qglslang::EOpGreaterThanEqual:
    case qglslang::EOpVectorEqual:
    case qglslang::EOpVectorNotEqual:
    {
        // Map the operation to a binary
        binOp = node->getOp();
        reduceComparison = false;
        switch (node->getOp()) {
        case qglslang::EOpVectorEqual:     binOp = qglslang::EOpVectorEqual;      break;
        case qglslang::EOpVectorNotEqual:  binOp = qglslang::EOpVectorNotEqual;   break;
        default:                          binOp = node->getOp();                break;
        }

        break;
    }
    case qglslang::EOpMul:
        // component-wise matrix multiply
        binOp = qglslang::EOpMul;
        break;
    case qglslang::EOpOuterProduct:
        // two vectors multiplied to make a matrix
        binOp = qglslang::EOpOuterProduct;
        break;
    case qglslang::EOpDot:
    {
        // for scalar dot product, use multiply
        qglslang::TIntermSequence& glslangOperands = node->getSequence();
        if (glslangOperands[0]->getAsTyped()->getVectorSize() == 1)
            binOp = qglslang::EOpMul;
        break;
    }
    case qglslang::EOpMod:
        // when an aggregate, this is the floating-point mod built-in function,
        // which can be emitted by the one in createBinaryOperation()
        binOp = qglslang::EOpMod;
        break;

    case qglslang::EOpEmitVertex:
    case qglslang::EOpEndPrimitive:
    case qglslang::EOpBarrier:
    case qglslang::EOpMemoryBarrier:
    case qglslang::EOpMemoryBarrierAtomicCounter:
    case qglslang::EOpMemoryBarrierBuffer:
    case qglslang::EOpMemoryBarrierImage:
    case qglslang::EOpMemoryBarrierShared:
    case qglslang::EOpGroupMemoryBarrier:
    case qglslang::EOpDeviceMemoryBarrier:
    case qglslang::EOpAllMemoryBarrierWithGroupSync:
    case qglslang::EOpDeviceMemoryBarrierWithGroupSync:
    case qglslang::EOpWorkgroupMemoryBarrier:
    case qglslang::EOpWorkgroupMemoryBarrierWithGroupSync:
    case qglslang::EOpSubgroupBarrier:
    case qglslang::EOpSubgroupMemoryBarrier:
    case qglslang::EOpSubgroupMemoryBarrierBuffer:
    case qglslang::EOpSubgroupMemoryBarrierImage:
    case qglslang::EOpSubgroupMemoryBarrierShared:
        noReturnValue = true;
        // These all have 0 operands and will naturally finish up in the code below for 0 operands
        break;

    case qglslang::EOpAtomicAdd:
    case qglslang::EOpAtomicSubtract:
    case qglslang::EOpAtomicMin:
    case qglslang::EOpAtomicMax:
    case qglslang::EOpAtomicAnd:
    case qglslang::EOpAtomicOr:
    case qglslang::EOpAtomicXor:
    case qglslang::EOpAtomicExchange:
    case qglslang::EOpAtomicCompSwap:
        atomic = true;
        break;

#ifndef GLSLANG_WEB
    case qglslang::EOpAtomicStore:
        noReturnValue = true;
        // fallthrough
    case qglslang::EOpAtomicLoad:
        atomic = true;
        break;

    case qglslang::EOpAtomicCounterAdd:
    case qglslang::EOpAtomicCounterSubtract:
    case qglslang::EOpAtomicCounterMin:
    case qglslang::EOpAtomicCounterMax:
    case qglslang::EOpAtomicCounterAnd:
    case qglslang::EOpAtomicCounterOr:
    case qglslang::EOpAtomicCounterXor:
    case qglslang::EOpAtomicCounterExchange:
    case qglslang::EOpAtomicCounterCompSwap:
        builder.addExtension("SPV_KHR_shader_atomic_counter_ops");
        builder.addCapability(spv::CapabilityAtomicStorageOps);
        atomic = true;
        break;

    case qglslang::EOpAbsDifference:
    case qglslang::EOpAddSaturate:
    case qglslang::EOpSubSaturate:
    case qglslang::EOpAverage:
    case qglslang::EOpAverageRounded:
    case qglslang::EOpMul32x16:
        builder.addCapability(spv::CapabilityIntegerFunctions2INTEL);
        builder.addExtension("SPV_INTEL_shader_integer_functions2");
        binOp = node->getOp();
        break;

    case qglslang::EOpIgnoreIntersectionNV:
    case qglslang::EOpTerminateRayNV:
    case qglslang::EOpTraceNV:
    case qglslang::EOpTraceRayMotionNV:
    case qglslang::EOpTraceKHR:
    case qglslang::EOpExecuteCallableNV:
    case qglslang::EOpExecuteCallableKHR:
    case qglslang::EOpWritePackedPrimitiveIndices4x8NV:
        noReturnValue = true;
        break;
    case qglslang::EOpRayQueryInitialize:
    case qglslang::EOpRayQueryTerminate:
    case qglslang::EOpRayQueryGenerateIntersection:
    case qglslang::EOpRayQueryConfirmIntersection:
        builder.addExtension("SPV_KHR_ray_query");
        builder.addCapability(spv::CapabilityRayQueryKHR);
        noReturnValue = true;
        break;
    case qglslang::EOpRayQueryProceed:
    case qglslang::EOpRayQueryGetIntersectionType:
    case qglslang::EOpRayQueryGetRayTMin:
    case qglslang::EOpRayQueryGetRayFlags:
    case qglslang::EOpRayQueryGetIntersectionT:
    case qglslang::EOpRayQueryGetIntersectionInstanceCustomIndex:
    case qglslang::EOpRayQueryGetIntersectionInstanceId:
    case qglslang::EOpRayQueryGetIntersectionInstanceShaderBindingTableRecordOffset:
    case qglslang::EOpRayQueryGetIntersectionGeometryIndex:
    case qglslang::EOpRayQueryGetIntersectionPrimitiveIndex:
    case qglslang::EOpRayQueryGetIntersectionBarycentrics:
    case qglslang::EOpRayQueryGetIntersectionFrontFace:
    case qglslang::EOpRayQueryGetIntersectionCandidateAABBOpaque:
    case qglslang::EOpRayQueryGetIntersectionObjectRayDirection:
    case qglslang::EOpRayQueryGetIntersectionObjectRayOrigin:
    case qglslang::EOpRayQueryGetWorldRayDirection:
    case qglslang::EOpRayQueryGetWorldRayOrigin:
    case qglslang::EOpRayQueryGetIntersectionObjectToWorld:
    case qglslang::EOpRayQueryGetIntersectionWorldToObject:
        builder.addExtension("SPV_KHR_ray_query");
        builder.addCapability(spv::CapabilityRayQueryKHR);
        break;
    case qglslang::EOpCooperativeMatrixLoad:
    case qglslang::EOpCooperativeMatrixStore:
        noReturnValue = true;
        break;
    case qglslang::EOpBeginInvocationInterlock:
    case qglslang::EOpEndInvocationInterlock:
        builder.addExtension(spv::E_SPV_EXT_fragment_shader_interlock);
        noReturnValue = true;
        break;
#endif

    case qglslang::EOpDebugPrintf:
        noReturnValue = true;
        break;

    default:
        break;
    }

    //
    // See if it maps to a regular operation.
    //
    if (binOp != qglslang::EOpNull) {
        qglslang::TIntermTyped* left = node->getSequence()[0]->getAsTyped();
        qglslang::TIntermTyped* right = node->getSequence()[1]->getAsTyped();
        assert(left && right);

        builder.clearAccessChain();
        left->traverse(this);
        spv::Id leftId = accessChainLoad(left->getType());

        builder.clearAccessChain();
        right->traverse(this);
        spv::Id rightId = accessChainLoad(right->getType());

        builder.setLine(node->getLoc().line, node->getLoc().getFilename());
        OpDecorations decorations = { precision,
                                      TranslateNoContractionDecoration(node->getType().getQualifier()),
                                      TranslateNonUniformDecoration(node->getType().getQualifier()) };
        result = createBinaryOperation(binOp, decorations,
                                       resultType(), leftId, rightId,
                                       left->getType().getBasicType(), reduceComparison);

        // code above should only make binOp that exists in createBinaryOperation
        assert(result != spv::NoResult);
        builder.clearAccessChain();
        builder.setAccessChainRValue(result);

        return false;
    }

    //
    // Create the list of operands.
    //
    qglslang::TIntermSequence& glslangOperands = node->getSequence();
    std::vector<spv::Id> operands;
    std::vector<spv::IdImmediate> memoryAccessOperands;
    for (int arg = 0; arg < (int)glslangOperands.size(); ++arg) {
        // special case l-value operands; there are just a few
        bool lvalue = false;
        switch (node->getOp()) {
        case qglslang::EOpModf:
            if (arg == 1)
                lvalue = true;
            break;

        case qglslang::EOpRayQueryInitialize:
        case qglslang::EOpRayQueryTerminate:
        case qglslang::EOpRayQueryConfirmIntersection:
        case qglslang::EOpRayQueryProceed:
        case qglslang::EOpRayQueryGenerateIntersection:
        case qglslang::EOpRayQueryGetIntersectionType:
        case qglslang::EOpRayQueryGetIntersectionT:
        case qglslang::EOpRayQueryGetIntersectionInstanceCustomIndex:
        case qglslang::EOpRayQueryGetIntersectionInstanceId:
        case qglslang::EOpRayQueryGetIntersectionInstanceShaderBindingTableRecordOffset:
        case qglslang::EOpRayQueryGetIntersectionGeometryIndex:
        case qglslang::EOpRayQueryGetIntersectionPrimitiveIndex:
        case qglslang::EOpRayQueryGetIntersectionBarycentrics:
        case qglslang::EOpRayQueryGetIntersectionFrontFace:
        case qglslang::EOpRayQueryGetIntersectionObjectRayDirection:
        case qglslang::EOpRayQueryGetIntersectionObjectRayOrigin:
        case qglslang::EOpRayQueryGetIntersectionObjectToWorld:
        case qglslang::EOpRayQueryGetIntersectionWorldToObject:
            if (arg == 0)
                lvalue = true;
            break;

        case qglslang::EOpAtomicAdd:
        case qglslang::EOpAtomicSubtract:
        case qglslang::EOpAtomicMin:
        case qglslang::EOpAtomicMax:
        case qglslang::EOpAtomicAnd:
        case qglslang::EOpAtomicOr:
        case qglslang::EOpAtomicXor:
        case qglslang::EOpAtomicExchange:
        case qglslang::EOpAtomicCompSwap:
            if (arg == 0)
                lvalue = true;
            break;

#ifndef GLSLANG_WEB
        case qglslang::EOpFrexp:
            if (arg == 1)
                lvalue = true;
            break;
        case qglslang::EOpInterpolateAtSample:
        case qglslang::EOpInterpolateAtOffset:
        case qglslang::EOpInterpolateAtVertex:
            if (arg == 0) {
                // If GLSL, use the address of the interpolant argument.
                // If HLSL, use an internal version of OpInterolates that takes
                // the rvalue of the interpolant. A fixup pass in spirv-opt
                // legalization will remove the OpLoad and convert to an lvalue.
                // Had to do this because legalization will only propagate a
                // builtin into an rvalue.
                lvalue = glslangIntermediate->getSource() != qglslang::EShSourceHlsl;

                // Does it need a swizzle inversion?  If so, evaluation is inverted;
                // operate first on the swizzle base, then apply the swizzle.
                // That is, we transform
                //
                //    interpolate(v.zy)  ->  interpolate(v).zy
                //
                if (glslangOperands[0]->getAsOperator() &&
                    glslangOperands[0]->getAsOperator()->getOp() == qglslang::EOpVectorSwizzle)
                    invertedType = convertGlslangToSpvType(
                        glslangOperands[0]->getAsBinaryNode()->getLeft()->getType());
            }
            break;
        case qglslang::EOpAtomicLoad:
        case qglslang::EOpAtomicStore:
        case qglslang::EOpAtomicCounterAdd:
        case qglslang::EOpAtomicCounterSubtract:
        case qglslang::EOpAtomicCounterMin:
        case qglslang::EOpAtomicCounterMax:
        case qglslang::EOpAtomicCounterAnd:
        case qglslang::EOpAtomicCounterOr:
        case qglslang::EOpAtomicCounterXor:
        case qglslang::EOpAtomicCounterExchange:
        case qglslang::EOpAtomicCounterCompSwap:
            if (arg == 0)
                lvalue = true;
            break;
        case qglslang::EOpAddCarry:
        case qglslang::EOpSubBorrow:
            if (arg == 2)
                lvalue = true;
            break;
        case qglslang::EOpUMulExtended:
        case qglslang::EOpIMulExtended:
            if (arg >= 2)
                lvalue = true;
            break;
        case qglslang::EOpCooperativeMatrixLoad:
            if (arg == 0 || arg == 1)
                lvalue = true;
            break;
        case qglslang::EOpCooperativeMatrixStore:
            if (arg == 1)
                lvalue = true;
            break;
        case qglslang::EOpSpirvInst:
            if (glslangOperands[arg]->getAsTyped()->getQualifier().isSpirvByReference())
                lvalue = true;
            break;
#endif
        default:
            break;
        }
        builder.clearAccessChain();
        if (invertedType != spv::NoType && arg == 0)
            glslangOperands[0]->getAsBinaryNode()->getLeft()->traverse(this);
        else
            glslangOperands[arg]->traverse(this);

#ifndef GLSLANG_WEB
        if (node->getOp() == qglslang::EOpCooperativeMatrixLoad ||
            node->getOp() == qglslang::EOpCooperativeMatrixStore) {

            if (arg == 1) {
                // fold "element" parameter into the access chain
                spv::Builder::AccessChain save = builder.getAccessChain();
                builder.clearAccessChain();
                glslangOperands[2]->traverse(this);

                spv::Id elementId = accessChainLoad(glslangOperands[2]->getAsTyped()->getType());

                builder.setAccessChain(save);

                // Point to the first element of the array.
                builder.accessChainPush(elementId,
                    TranslateCoherent(glslangOperands[arg]->getAsTyped()->getType()),
                                      glslangOperands[arg]->getAsTyped()->getType().getBufferReferenceAlignment());

                spv::Builder::AccessChain::CoherentFlags coherentFlags = builder.getAccessChain().coherentFlags;
                unsigned int alignment = builder.getAccessChain().alignment;

                int memoryAccess = TranslateMemoryAccess(coherentFlags);
                if (node->getOp() == qglslang::EOpCooperativeMatrixLoad)
                    memoryAccess &= ~spv::MemoryAccessMakePointerAvailableKHRMask;
                if (node->getOp() == qglslang::EOpCooperativeMatrixStore)
                    memoryAccess &= ~spv::MemoryAccessMakePointerVisibleKHRMask;
                if (builder.getStorageClass(builder.getAccessChain().base) ==
                    spv::StorageClassPhysicalStorageBufferEXT) {
                    memoryAccess = (spv::MemoryAccessMask)(memoryAccess | spv::MemoryAccessAlignedMask);
                }

                memoryAccessOperands.push_back(spv::IdImmediate(false, memoryAccess));

                if (memoryAccess & spv::MemoryAccessAlignedMask) {
                    memoryAccessOperands.push_back(spv::IdImmediate(false, alignment));
                }

                if (memoryAccess &
                    (spv::MemoryAccessMakePointerAvailableKHRMask | spv::MemoryAccessMakePointerVisibleKHRMask)) {
                    memoryAccessOperands.push_back(spv::IdImmediate(true,
                        builder.makeUintConstant(TranslateMemoryScope(coherentFlags))));
                }
            } else if (arg == 2) {
                continue;
            }
        }
#endif

        // for l-values, pass the address, for r-values, pass the value
        if (lvalue) {
            if (invertedType == spv::NoType && !builder.isSpvLvalue()) {
                // SPIR-V cannot represent an l-value containing a swizzle that doesn't
                // reduce to a simple access chain.  So, we need a temporary vector to
                // receive the result, and must later swizzle that into the original
                // l-value.
                complexLvalues.push_back(builder.getAccessChain());
                temporaryLvalues.push_back(builder.createVariable(
                    spv::NoPrecision, spv::StorageClassFunction,
                    builder.accessChainGetInferredType(), "swizzleTemp"));
                operands.push_back(temporaryLvalues.back());
            } else {
                operands.push_back(builder.accessChainGetLValue());
            }
            lvalueCoherentFlags = builder.getAccessChain().coherentFlags;
            lvalueCoherentFlags |= TranslateCoherent(glslangOperands[arg]->getAsTyped()->getType());
        } else {
            builder.setLine(node->getLoc().line, node->getLoc().getFilename());
             qglslang::TOperator glslangOp = node->getOp();
             if (arg == 1 &&
                (glslangOp == qglslang::EOpRayQueryGetIntersectionType ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionT ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionInstanceCustomIndex ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionInstanceId ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionInstanceShaderBindingTableRecordOffset ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionGeometryIndex ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionPrimitiveIndex ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionBarycentrics ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionFrontFace ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionObjectRayDirection ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionObjectRayOrigin ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionObjectToWorld ||
                 glslangOp == qglslang::EOpRayQueryGetIntersectionWorldToObject
                    )) {
                bool cond = glslangOperands[arg]->getAsConstantUnion()->getConstArray()[0].getBConst();
                operands.push_back(builder.makeIntConstant(cond ? 1 : 0));
             } else if ((arg == 10 && glslangOp == qglslang::EOpTraceKHR) ||
                        (arg == 11 && glslangOp == qglslang::EOpTraceRayMotionNV) ||
                        (arg == 1  && glslangOp == qglslang::EOpExecuteCallableKHR)) {
                 const int opdNum = glslangOp == qglslang::EOpTraceKHR ? 10 : (glslangOp == qglslang::EOpTraceRayMotionNV ? 11 : 1);
                 const int set = glslangOp == qglslang::EOpExecuteCallableKHR ? 1 : 0;

                 const int location = glslangOperands[opdNum]->getAsConstantUnion()->getConstArray()[0].getUConst();
                 auto itNode = locationToSymbol[set].find(location);
                 visitSymbol(itNode->second);
                 spv::Id symId = getSymbolId(itNode->second);
                 operands.push_back(symId);
#ifndef GLSLANG_WEB
             } else if (glslangOperands[arg]->getAsTyped()->getQualifier().isSpirvLiteral()) {
                 // Will be translated to a literal value, make a placeholder here
                 operands.push_back(spv::NoResult);
#endif
             } else  {
                operands.push_back(accessChainLoad(glslangOperands[arg]->getAsTyped()->getType()));
             }
        }
    }

    builder.setLine(node->getLoc().line, node->getLoc().getFilename());
#ifndef GLSLANG_WEB
    if (node->getOp() == qglslang::EOpCooperativeMatrixLoad) {
        std::vector<spv::IdImmediate> idImmOps;

        idImmOps.push_back(spv::IdImmediate(true, operands[1])); // buf
        idImmOps.push_back(spv::IdImmediate(true, operands[2])); // stride
        idImmOps.push_back(spv::IdImmediate(true, operands[3])); // colMajor
        idImmOps.insert(idImmOps.end(), memoryAccessOperands.begin(), memoryAccessOperands.end());
        // get the pointee type
        spv::Id typeId = builder.getContainedTypeId(builder.getTypeId(operands[0]));
        assert(builder.isCooperativeMatrixType(typeId));
        // do the op
        spv::Id result = builder.createOp(spv::OpCooperativeMatrixLoadNV, typeId, idImmOps);
        // store the result to the pointer (out param 'm')
        builder.createStore(result, operands[0]);
        result = 0;
    } else if (node->getOp() == qglslang::EOpCooperativeMatrixStore) {
        std::vector<spv::IdImmediate> idImmOps;

        idImmOps.push_back(spv::IdImmediate(true, operands[1])); // buf
        idImmOps.push_back(spv::IdImmediate(true, operands[0])); // object
        idImmOps.push_back(spv::IdImmediate(true, operands[2])); // stride
        idImmOps.push_back(spv::IdImmediate(true, operands[3])); // colMajor
        idImmOps.insert(idImmOps.end(), memoryAccessOperands.begin(), memoryAccessOperands.end());

        builder.createNoResultOp(spv::OpCooperativeMatrixStoreNV, idImmOps);
        result = 0;
    } else
#endif
    if (atomic) {
        // Handle all atomics
        qglslang::TBasicType typeProxy = (node->getOp() == qglslang::EOpAtomicStore)
            ? node->getSequence()[0]->getAsTyped()->getBasicType() : node->getBasicType();
        result = createAtomicOperation(node->getOp(), precision, resultType(), operands, typeProxy,
            lvalueCoherentFlags);
#ifndef GLSLANG_WEB
    } else if (node->getOp() == qglslang::EOpSpirvInst) {
        const auto& spirvInst = node->getSpirvInstruction();
        if (spirvInst.set == "") {
            std::vector<spv::IdImmediate> idImmOps;
            for (unsigned int i = 0; i < glslangOperands.size(); ++i) {
                if (glslangOperands[i]->getAsTyped()->getQualifier().isSpirvLiteral()) {
                    // Translate the constant to a literal value
                    std::vector<unsigned> literals;
                    qglslang::TVector<const qglslang::TIntermConstantUnion*> constants;
                    constants.push_back(glslangOperands[i]->getAsConstantUnion());
                    TranslateLiterals(constants, literals);
                    idImmOps.push_back({false, literals[0]});
                } else
                    idImmOps.push_back({true, operands[i]});
            }

            if (node->getBasicType() == qglslang::EbtVoid)
                builder.createNoResultOp(static_cast<spv::Op>(spirvInst.id), idImmOps);
            else
                result = builder.createOp(static_cast<spv::Op>(spirvInst.id), resultType(), idImmOps);
        } else {
            result = builder.createBuiltinCall(
                resultType(), spirvInst.set == "GLSL.std.450" ? stdBuiltins : getExtBuiltins(spirvInst.set.c_str()),
                spirvInst.id, operands);
        }
        noReturnValue = node->getBasicType() == qglslang::EbtVoid;
#endif
    } else if (node->getOp() == qglslang::EOpDebugPrintf) {
        if (!nonSemanticDebugPrintf) {
            nonSemanticDebugPrintf = builder.import("NonSemantic.DebugPrintf");
        }
        result = builder.createBuiltinCall(builder.makeVoidType(), nonSemanticDebugPrintf, spv::NonSemanticDebugPrintfDebugPrintf, operands);
        builder.addExtension(spv::E_SPV_KHR_non_semantic_info);
    } else {
        // Pass through to generic operations.
        switch (glslangOperands.size()) {
        case 0:
            result = createNoArgOperation(node->getOp(), precision, resultType());
            break;
        case 1:
            {
                OpDecorations decorations = { precision, 
                                              TranslateNoContractionDecoration(node->getType().getQualifier()),
                                              TranslateNonUniformDecoration(node->getType().getQualifier()) };
                result = createUnaryOperation(
                    node->getOp(), decorations,
                    resultType(), operands.front(),
                    glslangOperands[0]->getAsTyped()->getBasicType(), lvalueCoherentFlags);
            }
            break;
        default:
            result = createMiscOperation(node->getOp(), precision, resultType(), operands, node->getBasicType());
            break;
        }

        if (invertedType != spv::NoResult)
            result = createInvertedSwizzle(precision, *glslangOperands[0]->getAsBinaryNode(), result);

        for (unsigned int i = 0; i < temporaryLvalues.size(); ++i) {
            builder.setAccessChain(complexLvalues[i]);
            builder.accessChainStore(builder.createLoad(temporaryLvalues[i], spv::NoPrecision),
                TranslateNonUniformDecoration(complexLvalues[i].coherentFlags));
        }
    }

    if (noReturnValue)
        return false;

    if (! result) {
        logger->missingFunctionality("unknown glslang aggregate");
        return true;  // pick up a child as a placeholder operand
    } else {
        builder.clearAccessChain();
        builder.setAccessChainRValue(result);
        return false;
    }
}

// This path handles both if-then-else and ?:
// The if-then-else has a node type of void, while
// ?: has either a void or a non-void node type
//
// Leaving the result, when not void:
// GLSL only has r-values as the result of a :?, but
// if we have an l-value, that can be more efficient if it will
// become the base of a complex r-value expression, because the
// next layer copies r-values into memory to use the access-chain mechanism
bool TGlslangToSpvTraverser::visitSelection(qglslang::TVisit /* visit */, qglslang::TIntermSelection* node)
{
    // see if OpSelect can handle it
    const auto isOpSelectable = [&]() {
        if (node->getBasicType() == qglslang::EbtVoid)
            return false;
        // OpSelect can do all other types starting with SPV 1.4
        if (glslangIntermediate->getSpv().spv < qglslang::EShTargetSpv_1_4) {
            // pre-1.4, only scalars and vectors can be handled
            if ((!node->getType().isScalar() && !node->getType().isVector()))
                return false;
        }
        return true;
    };

    // See if it simple and safe, or required, to execute both sides.
    // Crucially, side effects must be either semantically required or avoided,
    // and there are performance trade-offs.
    // Return true if required or a good idea (and safe) to execute both sides,
    // false otherwise.
    const auto bothSidesPolicy = [&]() -> bool {
        // do we have both sides?
        if (node->getTrueBlock()  == nullptr ||
            node->getFalseBlock() == nullptr)
            return false;

        // required? (unless we write additional code to look for side effects
        // and make performance trade-offs if none are present)
        if (!node->getShortCircuit())
            return true;

        // if not required to execute both, decide based on performance/practicality...

        if (!isOpSelectable())
            return false;

        assert(node->getType() == node->getTrueBlock() ->getAsTyped()->getType() &&
               node->getType() == node->getFalseBlock()->getAsTyped()->getType());

        // return true if a single operand to ? : is okay for OpSelect
        const auto operandOkay = [](qglslang::TIntermTyped* node) {
            return node->getAsSymbolNode() || node->getType().getQualifier().isConstant();
        };

        return operandOkay(node->getTrueBlock() ->getAsTyped()) &&
               operandOkay(node->getFalseBlock()->getAsTyped());
    };

    spv::Id result = spv::NoResult; // upcoming result selecting between trueValue and falseValue
    // emit the condition before doing anything with selection
    node->getCondition()->traverse(this);
    spv::Id condition = accessChainLoad(node->getCondition()->getType());

    // Find a way of executing both sides and selecting the right result.
    const auto executeBothSides = [&]() -> void {
        // execute both sides
        node->getTrueBlock()->traverse(this);
        spv::Id trueValue = accessChainLoad(node->getTrueBlock()->getAsTyped()->getType());
        node->getFalseBlock()->traverse(this);
        spv::Id falseValue = accessChainLoad(node->getTrueBlock()->getAsTyped()->getType());

        builder.setLine(node->getLoc().line, node->getLoc().getFilename());

        // done if void
        if (node->getBasicType() == qglslang::EbtVoid)
            return;

        // emit code to select between trueValue and falseValue

        // see if OpSelect can handle it
        if (isOpSelectable()) {
            // Emit OpSelect for this selection.

            // smear condition to vector, if necessary (AST is always scalar)
            // Before 1.4, smear like for mix(), starting with 1.4, keep it scalar
            if (glslangIntermediate->getSpv().spv < qglslang::EShTargetSpv_1_4 && builder.isVector(trueValue)) {
                condition = builder.smearScalar(spv::NoPrecision, condition, 
                                                builder.makeVectorType(builder.makeBoolType(),
                                                                       builder.getNumComponents(trueValue)));
            }

            // OpSelect
            result = builder.createTriOp(spv::OpSelect,
                                         convertGlslangToSpvType(node->getType()), condition,
                                                                 trueValue, falseValue);

            builder.clearAccessChain();
            builder.setAccessChainRValue(result);
        } else {
            // We need control flow to select the result.
            // TODO: Once SPIR-V OpSelect allows arbitrary types, eliminate this path.
            result = builder.createVariable(TranslatePrecisionDecoration(node->getType()),
                spv::StorageClassFunction, convertGlslangToSpvType(node->getType()));

            // Selection control:
            const spv::SelectionControlMask control = TranslateSelectionControl(*node);

            // make an "if" based on the value created by the condition
            spv::Builder::If ifBuilder(condition, control, builder);

            // emit the "then" statement
            builder.createStore(trueValue, result);
            ifBuilder.makeBeginElse();
            // emit the "else" statement
            builder.createStore(falseValue, result);

            // finish off the control flow
            ifBuilder.makeEndIf();

            builder.clearAccessChain();
            builder.setAccessChainLValue(result);
        }
    };

    // Execute the one side needed, as per the condition
    const auto executeOneSide = [&]() {
        // Always emit control flow.
        if (node->getBasicType() != qglslang::EbtVoid) {
            result = builder.createVariable(TranslatePrecisionDecoration(node->getType()), spv::StorageClassFunction,
                convertGlslangToSpvType(node->getType()));
        }

        // Selection control:
        const spv::SelectionControlMask control = TranslateSelectionControl(*node);

        // make an "if" based on the value created by the condition
        spv::Builder::If ifBuilder(condition, control, builder);

        // emit the "then" statement
        if (node->getTrueBlock() != nullptr) {
            node->getTrueBlock()->traverse(this);
            if (result != spv::NoResult)
                builder.createStore(accessChainLoad(node->getTrueBlock()->getAsTyped()->getType()), result);
        }

        if (node->getFalseBlock() != nullptr) {
            ifBuilder.makeBeginElse();
            // emit the "else" statement
            node->getFalseBlock()->traverse(this);
            if (result != spv::NoResult)
                builder.createStore(accessChainLoad(node->getFalseBlock()->getAsTyped()->getType()), result);
        }

        // finish off the control flow
        ifBuilder.makeEndIf();

        if (result != spv::NoResult) {
            builder.clearAccessChain();
            builder.setAccessChainLValue(result);
        }
    };

    // Try for OpSelect (or a requirement to execute both sides)
    if (bothSidesPolicy()) {
        SpecConstantOpModeGuard spec_constant_op_mode_setter(&builder);
        if (node->getType().getQualifier().isSpecConstant())
            spec_constant_op_mode_setter.turnOnSpecConstantOpMode();
        executeBothSides();
    } else
        executeOneSide();

    return false;
}

bool TGlslangToSpvTraverser::visitSwitch(qglslang::TVisit /* visit */, qglslang::TIntermSwitch* node)
{
    // emit and get the condition before doing anything with switch
    node->getCondition()->traverse(this);
    spv::Id selector = accessChainLoad(node->getCondition()->getAsTyped()->getType());

    // Selection control:
    const spv::SelectionControlMask control = TranslateSwitchControl(*node);

    // browse the children to sort out code segments
    int defaultSegment = -1;
    std::vector<TIntermNode*> codeSegments;
    qglslang::TIntermSequence& sequence = node->getBody()->getSequence();
    std::vector<int> caseValues;
    std::vector<int> valueIndexToSegment(sequence.size());  // note: probably not all are used, it is an overestimate
    for (qglslang::TIntermSequence::iterator c = sequence.begin(); c != sequence.end(); ++c) {
        TIntermNode* child = *c;
        if (child->getAsBranchNode() && child->getAsBranchNode()->getFlowOp() == qglslang::EOpDefault)
            defaultSegment = (int)codeSegments.size();
        else if (child->getAsBranchNode() && child->getAsBranchNode()->getFlowOp() == qglslang::EOpCase) {
            valueIndexToSegment[caseValues.size()] = (int)codeSegments.size();
            caseValues.push_back(child->getAsBranchNode()->getExpression()->getAsConstantUnion()
                ->getConstArray()[0].getIConst());
        } else
            codeSegments.push_back(child);
    }

    // handle the case where the last code segment is missing, due to no code
    // statements between the last case and the end of the switch statement
    if ((caseValues.size() && (int)codeSegments.size() == valueIndexToSegment[caseValues.size() - 1]) ||
        (int)codeSegments.size() == defaultSegment)
        codeSegments.push_back(nullptr);

    // make the switch statement
    std::vector<spv::Block*> segmentBlocks; // returned, as the blocks allocated in the call
    builder.makeSwitch(selector, control, (int)codeSegments.size(), caseValues, valueIndexToSegment, defaultSegment,
        segmentBlocks);

    // emit all the code in the segments
    breakForLoop.push(false);
    for (unsigned int s = 0; s < codeSegments.size(); ++s) {
        builder.nextSwitchSegment(segmentBlocks, s);
        if (codeSegments[s])
            codeSegments[s]->traverse(this);
        else
            builder.addSwitchBreak();
    }
    breakForLoop.pop();

    builder.endSwitch(segmentBlocks);

    return false;
}

void TGlslangToSpvTraverser::visitConstantUnion(qglslang::TIntermConstantUnion* node)
{
#ifndef GLSLANG_WEB
    if (node->getQualifier().isSpirvLiteral())
        return; // Translated to a literal value, skip further processing
#endif

    int nextConst = 0;
    spv::Id constant = createSpvConstantFromConstUnionArray(node->getType(), node->getConstArray(), nextConst, false);

    builder.clearAccessChain();
    builder.setAccessChainRValue(constant);
}

bool TGlslangToSpvTraverser::visitLoop(qglslang::TVisit /* visit */, qglslang::TIntermLoop* node)
{
    auto blocks = builder.makeNewLoop();
    builder.createBranch(&blocks.head);

    // Loop control:
    std::vector<unsigned int> operands;
    const spv::LoopControlMask control = TranslateLoopControl(*node, operands);

    // Spec requires back edges to target header blocks, and every header block
    // must dominate its merge block.  Make a header block first to ensure these
    // conditions are met.  By definition, it will contain OpLoopMerge, followed
    // by a block-ending branch.  But we don't want to put any other body/test
    // instructions in it, since the body/test may have arbitrary instructions,
    // including merges of its own.
    builder.setLine(node->getLoc().line, node->getLoc().getFilename());
    builder.setBuildPoint(&blocks.head);
    builder.createLoopMerge(&blocks.merge, &blocks.continue_target, control, operands);
    if (node->testFirst() && node->getTest()) {
        spv::Block& test = builder.makeNewBlock();
        builder.createBranch(&test);

        builder.setBuildPoint(&test);
        node->getTest()->traverse(this);
        spv::Id condition = accessChainLoad(node->getTest()->getType());
        builder.createConditionalBranch(condition, &blocks.body, &blocks.merge);

        builder.setBuildPoint(&blocks.body);
        breakForLoop.push(true);
        if (node->getBody())
            node->getBody()->traverse(this);
        builder.createBranch(&blocks.continue_target);
        breakForLoop.pop();

        builder.setBuildPoint(&blocks.continue_target);
        if (node->getTerminal())
            node->getTerminal()->traverse(this);
        builder.createBranch(&blocks.head);
    } else {
        builder.setLine(node->getLoc().line, node->getLoc().getFilename());
        builder.createBranch(&blocks.body);

        breakForLoop.push(true);
        builder.setBuildPoint(&blocks.body);
        if (node->getBody())
            node->getBody()->traverse(this);
        builder.createBranch(&blocks.continue_target);
        breakForLoop.pop();

        builder.setBuildPoint(&blocks.continue_target);
        if (node->getTerminal())
            node->getTerminal()->traverse(this);
        if (node->getTest()) {
            node->getTest()->traverse(this);
            spv::Id condition =
                accessChainLoad(node->getTest()->getType());
            builder.createConditionalBranch(condition, &blocks.head, &blocks.merge);
        } else {
            // TODO: unless there was a break/return/discard instruction
            // somewhere in the body, this is an infinite loop, so we should
            // issue a warning.
            builder.createBranch(&blocks.head);
        }
    }
    builder.setBuildPoint(&blocks.merge);
    builder.closeLoop();
    return false;
}

bool TGlslangToSpvTraverser::visitBranch(qglslang::TVisit /* visit */, qglslang::TIntermBranch* node)
{
    if (node->getExpression())
        node->getExpression()->traverse(this);

    builder.setLine(node->getLoc().line, node->getLoc().getFilename());

    switch (node->getFlowOp()) {
    case qglslang::EOpKill:
        if (glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_6) {
            if (glslangIntermediate->getSource() == qglslang::EShSourceHlsl) {
              builder.addCapability(spv::CapabilityDemoteToHelperInvocation);
              builder.createNoResultOp(spv::OpDemoteToHelperInvocationEXT);
            } else {
                builder.makeStatementTerminator(spv::OpTerminateInvocation, "post-terminate-invocation");
            }
        } else {
            builder.makeStatementTerminator(spv::OpKill, "post-discard");
        }
        break;
    case qglslang::EOpTerminateInvocation:
        builder.addExtension(spv::E_SPV_KHR_terminate_invocation);
        builder.makeStatementTerminator(spv::OpTerminateInvocation, "post-terminate-invocation");
        break;
    case qglslang::EOpBreak:
        if (breakForLoop.top())
            builder.createLoopExit();
        else
            builder.addSwitchBreak();
        break;
    case qglslang::EOpContinue:
        builder.createLoopContinue();
        break;
    case qglslang::EOpReturn:
        if (node->getExpression() != nullptr) {
            const qglslang::TType& glslangReturnType = node->getExpression()->getType();
            spv::Id returnId = accessChainLoad(glslangReturnType);
            if (builder.getTypeId(returnId) != currentFunction->getReturnType() ||
                TranslatePrecisionDecoration(glslangReturnType) != currentFunction->getReturnPrecision()) {
                builder.clearAccessChain();
                spv::Id copyId = builder.createVariable(currentFunction->getReturnPrecision(),
                    spv::StorageClassFunction, currentFunction->getReturnType());
                builder.setAccessChainLValue(copyId);
                multiTypeStore(glslangReturnType, returnId);
                returnId = builder.createLoad(copyId, currentFunction->getReturnPrecision());
            }
            builder.makeReturn(false, returnId);
        } else
            builder.makeReturn(false);

        builder.clearAccessChain();
        break;

#ifndef GLSLANG_WEB
    case qglslang::EOpDemote:
        builder.createNoResultOp(spv::OpDemoteToHelperInvocationEXT);
        builder.addExtension(spv::E_SPV_EXT_demote_to_helper_invocation);
        builder.addCapability(spv::CapabilityDemoteToHelperInvocationEXT);
        break;
    case qglslang::EOpTerminateRayKHR:
        builder.makeStatementTerminator(spv::OpTerminateRayKHR, "post-terminateRayKHR");
        break;
    case qglslang::EOpIgnoreIntersectionKHR:
        builder.makeStatementTerminator(spv::OpIgnoreIntersectionKHR, "post-ignoreIntersectionKHR");
        break;
#endif

    default:
        assert(0);
        break;
    }

    return false;
}

spv::Id TGlslangToSpvTraverser::createSpvVariable(const qglslang::TIntermSymbol* node, spv::Id forcedType)
{
    // First, steer off constants, which are not SPIR-V variables, but
    // can still have a mapping to a SPIR-V Id.
    // This includes specialization constants.
    if (node->getQualifier().isConstant()) {
        spv::Id result = createSpvConstant(*node);
        if (result != spv::NoResult)
            return result;
    }

    // Now, handle actual variables
    spv::StorageClass storageClass = TranslateStorageClass(node->getType());
    spv::Id spvType = forcedType == spv::NoType ? convertGlslangToSpvType(node->getType())
                                                : forcedType;

    const bool contains16BitType = node->getType().contains16BitFloat() ||
                                   node->getType().contains16BitInt();
    if (contains16BitType) {
        switch (storageClass) {
        case spv::StorageClassInput:
        case spv::StorageClassOutput:
            builder.addIncorporatedExtension(spv::E_SPV_KHR_16bit_storage, spv::Spv_1_3);
            builder.addCapability(spv::CapabilityStorageInputOutput16);
            break;
        case spv::StorageClassUniform:
            builder.addIncorporatedExtension(spv::E_SPV_KHR_16bit_storage, spv::Spv_1_3);
            if (node->getType().getQualifier().storage == qglslang::EvqBuffer)
                builder.addCapability(spv::CapabilityStorageUniformBufferBlock16);
            else
                builder.addCapability(spv::CapabilityStorageUniform16);
            break;
#ifndef GLSLANG_WEB
        case spv::StorageClassPushConstant:
            builder.addIncorporatedExtension(spv::E_SPV_KHR_16bit_storage, spv::Spv_1_3);
            builder.addCapability(spv::CapabilityStoragePushConstant16);
            break;
        case spv::StorageClassStorageBuffer:
        case spv::StorageClassPhysicalStorageBufferEXT:
            builder.addIncorporatedExtension(spv::E_SPV_KHR_16bit_storage, spv::Spv_1_3);
            builder.addCapability(spv::CapabilityStorageUniformBufferBlock16);
            break;
#endif
        default:
            if (storageClass == spv::StorageClassWorkgroup &&
                node->getType().getBasicType() == qglslang::EbtBlock) {
                builder.addCapability(spv::CapabilityWorkgroupMemoryExplicitLayout16BitAccessKHR);
                break;
            }
            if (node->getType().contains16BitFloat())
                builder.addCapability(spv::CapabilityFloat16);
            if (node->getType().contains16BitInt())
                builder.addCapability(spv::CapabilityInt16);
            break;
        }
    }

    if (node->getType().contains8BitInt()) {
        if (storageClass == spv::StorageClassPushConstant) {
            builder.addIncorporatedExtension(spv::E_SPV_KHR_8bit_storage, spv::Spv_1_5);
            builder.addCapability(spv::CapabilityStoragePushConstant8);
        } else if (storageClass == spv::StorageClassUniform) {
            builder.addIncorporatedExtension(spv::E_SPV_KHR_8bit_storage, spv::Spv_1_5);
            builder.addCapability(spv::CapabilityUniformAndStorageBuffer8BitAccess);
        } else if (storageClass == spv::StorageClassStorageBuffer) {
            builder.addIncorporatedExtension(spv::E_SPV_KHR_8bit_storage, spv::Spv_1_5);
            builder.addCapability(spv::CapabilityStorageBuffer8BitAccess);
        } else if (storageClass == spv::StorageClassWorkgroup &&
                   node->getType().getBasicType() == qglslang::EbtBlock) {
            builder.addCapability(spv::CapabilityWorkgroupMemoryExplicitLayout8BitAccessKHR);
        } else {
            builder.addCapability(spv::CapabilityInt8);
        }
    }

    const char* name = node->getName().c_str();
    if (qglslang::IsAnonymous(name))
        name = "";

    spv::Id initializer = spv::NoResult;

    if (node->getType().getQualifier().storage == qglslang::EvqUniform && !node->getConstArray().empty()) {
        int nextConst = 0;
        initializer = createSpvConstantFromConstUnionArray(node->getType(),
                                                           node->getConstArray(),
                                                           nextConst,
                                                           false /* specConst */);
    } else if (node->getType().getQualifier().isNullInit()) {
        initializer = builder.makeNullConstant(spvType);
    }

    return builder.createVariable(spv::NoPrecision, storageClass, spvType, name, initializer);
}

// Return type Id of the sampled type.
spv::Id TGlslangToSpvTraverser::getSampledType(const qglslang::TSampler& sampler)
{
    switch (sampler.type) {
        case qglslang::EbtInt:      return builder.makeIntType(32);
        case qglslang::EbtUint:     return builder.makeUintType(32);
        case qglslang::EbtFloat:    return builder.makeFloatType(32);
#ifndef GLSLANG_WEB
        case qglslang::EbtFloat16:
            builder.addExtension(spv::E_SPV_AMD_gpu_shader_half_float_fetch);
            builder.addCapability(spv::CapabilityFloat16ImageAMD);
            return builder.makeFloatType(16);
        case qglslang::EbtInt64:
            builder.addExtension(spv::E_SPV_EXT_shader_image_int64);
            builder.addCapability(spv::CapabilityInt64ImageEXT);
            return builder.makeIntType(64);
        case qglslang::EbtUint64:
            builder.addExtension(spv::E_SPV_EXT_shader_image_int64);
            builder.addCapability(spv::CapabilityInt64ImageEXT);
            return builder.makeUintType(64);
#endif
        default:
            assert(0);
            return builder.makeFloatType(32);
    }
}

// If node is a swizzle operation, return the type that should be used if
// the swizzle base is first consumed by another operation, before the swizzle
// is applied.
spv::Id TGlslangToSpvTraverser::getInvertedSwizzleType(const qglslang::TIntermTyped& node)
{
    if (node.getAsOperator() &&
        node.getAsOperator()->getOp() == qglslang::EOpVectorSwizzle)
        return convertGlslangToSpvType(node.getAsBinaryNode()->getLeft()->getType());
    else
        return spv::NoType;
}

// When inverting a swizzle with a parent op, this function
// will apply the swizzle operation to a completed parent operation.
spv::Id TGlslangToSpvTraverser::createInvertedSwizzle(spv::Decoration precision, const qglslang::TIntermTyped& node,
    spv::Id parentResult)
{
    std::vector<unsigned> swizzle;
    convertSwizzle(*node.getAsBinaryNode()->getRight()->getAsAggregate(), swizzle);
    return builder.createRvalueSwizzle(precision, convertGlslangToSpvType(node.getType()), parentResult, swizzle);
}

// Convert a glslang AST swizzle node to a swizzle vector for building SPIR-V.
void TGlslangToSpvTraverser::convertSwizzle(const qglslang::TIntermAggregate& node, std::vector<unsigned>& swizzle)
{
    const qglslang::TIntermSequence& swizzleSequence = node.getSequence();
    for (int i = 0; i < (int)swizzleSequence.size(); ++i)
        swizzle.push_back(swizzleSequence[i]->getAsConstantUnion()->getConstArray()[0].getIConst());
}

// Convert from a glslang type to an SPV type, by calling into a
// recursive version of this function. This establishes the inherited
// layout state rooted from the top-level type.
spv::Id TGlslangToSpvTraverser::convertGlslangToSpvType(const qglslang::TType& type, bool forwardReferenceOnly)
{
    return convertGlslangToSpvType(type, getExplicitLayout(type), type.getQualifier(), false, forwardReferenceOnly);
}

// Do full recursive conversion of an arbitrary glslang type to a SPIR-V Id.
// explicitLayout can be kept the same throughout the hierarchical recursive walk.
// Mutually recursive with convertGlslangStructToSpvType().
spv::Id TGlslangToSpvTraverser::convertGlslangToSpvType(const qglslang::TType& type,
    qglslang::TLayoutPacking explicitLayout, const qglslang::TQualifier& qualifier,
    bool lastBufferBlockMember, bool forwardReferenceOnly)
{
    spv::Id spvType = spv::NoResult;

    switch (type.getBasicType()) {
    case qglslang::EbtVoid:
        spvType = builder.makeVoidType();
        assert (! type.isArray());
        break;
    case qglslang::EbtBool:
        // "transparent" bool doesn't exist in SPIR-V.  The GLSL convention is
        // a 32-bit int where non-0 means true.
        if (explicitLayout != qglslang::ElpNone)
            spvType = builder.makeUintType(32);
        else
            spvType = builder.makeBoolType();
        break;
    case qglslang::EbtInt:
        spvType = builder.makeIntType(32);
        break;
    case qglslang::EbtUint:
        spvType = builder.makeUintType(32);
        break;
    case qglslang::EbtFloat:
        spvType = builder.makeFloatType(32);
        break;
#ifndef GLSLANG_WEB
    case qglslang::EbtDouble:
        spvType = builder.makeFloatType(64);
        break;
    case qglslang::EbtFloat16:
        spvType = builder.makeFloatType(16);
        break;
    case qglslang::EbtInt8:
        spvType = builder.makeIntType(8);
        break;
    case qglslang::EbtUint8:
        spvType = builder.makeUintType(8);
        break;
    case qglslang::EbtInt16:
        spvType = builder.makeIntType(16);
        break;
    case qglslang::EbtUint16:
        spvType = builder.makeUintType(16);
        break;
    case qglslang::EbtInt64:
        spvType = builder.makeIntType(64);
        break;
    case qglslang::EbtUint64:
        spvType = builder.makeUintType(64);
        break;
    case qglslang::EbtAtomicUint:
        builder.addCapability(spv::CapabilityAtomicStorage);
        spvType = builder.makeUintType(32);
        break;
    case qglslang::EbtAccStruct:
        switch (glslangIntermediate->getStage()) {
        case EShLangRayGen:
        case EShLangIntersect:
        case EShLangAnyHit:
        case EShLangClosestHit:
        case EShLangMiss:
        case EShLangCallable:
            // these all should have the RayTracingNV/KHR capability already
            break;
        default:
            {
                auto& extensions = glslangIntermediate->getRequestedExtensions();
                if (extensions.find("GL_EXT_ray_query") != extensions.end()) {
                    builder.addExtension(spv::E_SPV_KHR_ray_query);
                    builder.addCapability(spv::CapabilityRayQueryKHR);
                }
            }
            break;
        }
        spvType = builder.makeAccelerationStructureType();
        break;
    case qglslang::EbtRayQuery:
        {
            auto& extensions = glslangIntermediate->getRequestedExtensions();
            if (extensions.find("GL_EXT_ray_query") != extensions.end()) {
                builder.addExtension(spv::E_SPV_KHR_ray_query);
                builder.addCapability(spv::CapabilityRayQueryKHR);
            }
            spvType = builder.makeRayQueryType();
        }
        break;
    case qglslang::EbtReference:
        {
            // Make the forward pointer, then recurse to convert the structure type, then
            // patch up the forward pointer with a real pointer type.
            if (forwardPointers.find(type.getReferentType()) == forwardPointers.end()) {
                spv::Id forwardId = builder.makeForwardPointer(spv::StorageClassPhysicalStorageBufferEXT);
                forwardPointers[type.getReferentType()] = forwardId;
            }
            spvType = forwardPointers[type.getReferentType()];
            if (!forwardReferenceOnly) {
                spv::Id referentType = convertGlslangToSpvType(*type.getReferentType());
                builder.makePointerFromForwardPointer(spv::StorageClassPhysicalStorageBufferEXT,
                                                      forwardPointers[type.getReferentType()],
                                                      referentType);
            }
        }
        break;
#endif
    case qglslang::EbtSampler:
        {
            const qglslang::TSampler& sampler = type.getSampler();
            if (sampler.isPureSampler()) {
                spvType = builder.makeSamplerType();
            } else {
                // an image is present, make its type
                spvType = builder.makeImageType(getSampledType(sampler), TranslateDimensionality(sampler),
                                                sampler.isShadow(), sampler.isArrayed(), sampler.isMultiSample(),
                                                sampler.isImageClass() ? 2 : 1, TranslateImageFormat(type));
                if (sampler.isCombined()) {
                    // already has both image and sampler, make the combined type
                    spvType = builder.makeSampledImageType(spvType);
                }
            }
        }
        break;
    case qglslang::EbtStruct:
    case qglslang::EbtBlock:
        {
            // If we've seen this struct type, return it
            const qglslang::TTypeList* glslangMembers = type.getStruct();

            // Try to share structs for different layouts, but not yet for other
            // kinds of qualification (primarily not yet including interpolant qualification).
            if (! HasNonLayoutQualifiers(type, qualifier))
                spvType = structMap[explicitLayout][qualifier.layoutMatrix][glslangMembers];
            if (spvType != spv::NoResult)
                break;

            // else, we haven't seen it...
            if (type.getBasicType() == qglslang::EbtBlock)
                memberRemapper[glslangTypeToIdMap[glslangMembers]].resize(glslangMembers->size());
            spvType = convertGlslangStructToSpvType(type, glslangMembers, explicitLayout, qualifier);
        }
        break;
    case qglslang::EbtString:
        // no type used for OpString
        return 0;
#ifndef GLSLANG_WEB
    case qglslang::EbtSpirvType: {
        // GL_EXT_spirv_intrinsics
        const auto& spirvType = type.getSpirvType();
        const auto& spirvInst = spirvType.spirvInst;

        std::vector<spv::IdImmediate> operands;
        for (const auto& typeParam : spirvType.typeParams) {
            // Constant expression
            if (typeParam.constant->isLiteral()) {
                if (typeParam.constant->getBasicType() == qglslang::EbtFloat) {
                    float floatValue = static_cast<float>(typeParam.constant->getConstArray()[0].getDConst());
                    unsigned literal = *reinterpret_cast<unsigned*>(&floatValue);
                    operands.push_back({false, literal});
                } else if (typeParam.constant->getBasicType() == qglslang::EbtInt) {
                    unsigned literal = typeParam.constant->getConstArray()[0].getIConst();
                    operands.push_back({false, literal});
                } else if (typeParam.constant->getBasicType() == qglslang::EbtUint) {
                    unsigned literal = typeParam.constant->getConstArray()[0].getUConst();
                    operands.push_back({false, literal});
                } else if (typeParam.constant->getBasicType() == qglslang::EbtBool) {
                    unsigned literal = typeParam.constant->getConstArray()[0].getBConst();
                    operands.push_back({false, literal});
                } else if (typeParam.constant->getBasicType() == qglslang::EbtString) {
                    auto str = typeParam.constant->getConstArray()[0].getSConst()->c_str();
                    unsigned literal = 0;
                    char* literalPtr = reinterpret_cast<char*>(&literal);
                    unsigned charCount = 0;
                    char ch = 0;
                    do {
                        ch = *(str++);
                        *(literalPtr++) = ch;
                        ++charCount;
                        if (charCount == 4) {
                            operands.push_back({false, literal});
                            literalPtr = reinterpret_cast<char*>(&literal);
                            charCount = 0;
                        }
                    } while (ch != 0);

                    // Partial literal is padded with 0
                    if (charCount > 0) {
                        for (; charCount < 4; ++charCount)
                            *(literalPtr++) = 0;
                        operands.push_back({false, literal});
                    }
                } else
                    assert(0); // Unexpected type
            } else
                operands.push_back({true, createSpvConstant(*typeParam.constant)});
        }

        assert(spirvInst.set == ""); // Currently, couldn't be extended instructions.
        spvType = builder.makeGenericType(static_cast<spv::Op>(spirvInst.id), operands);

        break;
    }
#endif
    default:
        assert(0);
        break;
    }

    if (type.isMatrix())
        spvType = builder.makeMatrixType(spvType, type.getMatrixCols(), type.getMatrixRows());
    else {
        // If this variable has a vector element count greater than 1, create a SPIR-V vector
        if (type.getVectorSize() > 1)
            spvType = builder.makeVectorType(spvType, type.getVectorSize());
    }

    if (type.isCoopMat()) {
        builder.addCapability(spv::CapabilityCooperativeMatrixNV);
        builder.addExtension(spv::E_SPV_NV_cooperative_matrix);
        if (type.getBasicType() == qglslang::EbtFloat16)
            builder.addCapability(spv::CapabilityFloat16);
        if (type.getBasicType() == qglslang::EbtUint8 ||
            type.getBasicType() == qglslang::EbtInt8) {
            builder.addCapability(spv::CapabilityInt8);
        }

        spv::Id scope = makeArraySizeId(*type.getTypeParameters(), 1);
        spv::Id rows = makeArraySizeId(*type.getTypeParameters(), 2);
        spv::Id cols = makeArraySizeId(*type.getTypeParameters(), 3);

        spvType = builder.makeCooperativeMatrixType(spvType, scope, rows, cols);
    }

    if (type.isArray()) {
        int stride = 0;  // keep this 0 unless doing an explicit layout; 0 will mean no decoration, no stride

        // Do all but the outer dimension
        if (type.getArraySizes()->getNumDims() > 1) {
            // We need to decorate array strides for types needing explicit layout, except blocks.
            if (explicitLayout != qglslang::ElpNone && type.getBasicType() != qglslang::EbtBlock) {
                // Use a dummy glslang type for querying internal strides of
                // arrays of arrays, but using just a one-dimensional array.
                qglslang::TType simpleArrayType(type, 0); // deference type of the array
                while (simpleArrayType.getArraySizes()->getNumDims() > 1)
                    simpleArrayType.getArraySizes()->dereference();

                // Will compute the higher-order strides here, rather than making a whole
                // pile of types and doing repetitive recursion on their contents.
                stride = getArrayStride(simpleArrayType, explicitLayout, qualifier.layoutMatrix);
            }

            // make the arrays
            for (int dim = type.getArraySizes()->getNumDims() - 1; dim > 0; --dim) {
                spvType = builder.makeArrayType(spvType, makeArraySizeId(*type.getArraySizes(), dim), stride);
                if (stride > 0)
                    builder.addDecoration(spvType, spv::DecorationArrayStride, stride);
                stride *= type.getArraySizes()->getDimSize(dim);
            }
        } else {
            // single-dimensional array, and don't yet have stride

            // We need to decorate array strides for types needing explicit layout, except blocks.
            if (explicitLayout != qglslang::ElpNone && type.getBasicType() != qglslang::EbtBlock)
                stride = getArrayStride(type, explicitLayout, qualifier.layoutMatrix);
        }

        // Do the outer dimension, which might not be known for a runtime-sized array.
        // (Unsized arrays that survive through linking will be runtime-sized arrays)
        if (type.isSizedArray())
            spvType = builder.makeArrayType(spvType, makeArraySizeId(*type.getArraySizes(), 0), stride);
        else {
#ifndef GLSLANG_WEB
            if (!lastBufferBlockMember) {
                builder.addIncorporatedExtension("SPV_EXT_descriptor_indexing", spv::Spv_1_5);
                builder.addCapability(spv::CapabilityRuntimeDescriptorArrayEXT);
            }
#endif
            spvType = builder.makeRuntimeArray(spvType);
        }
        if (stride > 0)
            builder.addDecoration(spvType, spv::DecorationArrayStride, stride);
    }

    return spvType;
}

// TODO: this functionality should exist at a higher level, in creating the AST
//
// Identify interface members that don't have their required extension turned on.
//
bool TGlslangToSpvTraverser::filterMember(const qglslang::TType& member)
{
#ifndef GLSLANG_WEB
    auto& extensions = glslangIntermediate->getRequestedExtensions();

    if (member.getFieldName() == "gl_SecondaryViewportMaskNV" &&
        extensions.find("GL_NV_stereo_view_rendering") == extensions.end())
        return true;
    if (member.getFieldName() == "gl_SecondaryPositionNV" &&
        extensions.find("GL_NV_stereo_view_rendering") == extensions.end())
        return true;

    if (glslangIntermediate->getStage() != EShLangMeshNV) {
        if (member.getFieldName() == "gl_ViewportMask" &&
            extensions.find("GL_NV_viewport_array2") == extensions.end())
            return true;
        if (member.getFieldName() == "gl_PositionPerViewNV" &&
            extensions.find("GL_NVX_multiview_per_view_attributes") == extensions.end())
            return true;
        if (member.getFieldName() == "gl_ViewportMaskPerViewNV" &&
            extensions.find("GL_NVX_multiview_per_view_attributes") == extensions.end())
            return true;
    }
#endif

    return false;
};

// Do full recursive conversion of a glslang structure (or block) type to a SPIR-V Id.
// explicitLayout can be kept the same throughout the hierarchical recursive walk.
// Mutually recursive with convertGlslangToSpvType().
spv::Id TGlslangToSpvTraverser::convertGlslangStructToSpvType(const qglslang::TType& type,
                                                              const qglslang::TTypeList* glslangMembers,
                                                              qglslang::TLayoutPacking explicitLayout,
                                                              const qglslang::TQualifier& qualifier)
{
    // Create a vector of struct types for SPIR-V to consume
    std::vector<spv::Id> spvMembers;
    int memberDelta = 0;  // how much the member's index changes from glslang to SPIR-V, normally 0,
                          // except sometimes for blocks
    std::vector<std::pair<qglslang::TType*, qglslang::TQualifier> > deferredForwardPointers;
    for (int i = 0; i < (int)glslangMembers->size(); i++) {
        qglslang::TType& glslangMember = *(*glslangMembers)[i].type;
        if (glslangMember.hiddenMember()) {
            ++memberDelta;
            if (type.getBasicType() == qglslang::EbtBlock)
                memberRemapper[glslangTypeToIdMap[glslangMembers]][i] = -1;
        } else {
            if (type.getBasicType() == qglslang::EbtBlock) {
                if (filterMember(glslangMember)) {
                    memberDelta++;
                    memberRemapper[glslangTypeToIdMap[glslangMembers]][i] = -1;
                    continue;
                }
                memberRemapper[glslangTypeToIdMap[glslangMembers]][i] = i - memberDelta;
            }
            // modify just this child's view of the qualifier
            qglslang::TQualifier memberQualifier = glslangMember.getQualifier();
            InheritQualifiers(memberQualifier, qualifier);

            // manually inherit location
            if (! memberQualifier.hasLocation() && qualifier.hasLocation())
                memberQualifier.layoutLocation = qualifier.layoutLocation;

            // recurse
            bool lastBufferBlockMember = qualifier.storage == qglslang::EvqBuffer &&
                                         i == (int)glslangMembers->size() - 1;

            // Make forward pointers for any pointer members, and create a list of members to
            // convert to spirv types after creating the struct.
            if (glslangMember.isReference()) {
                if (forwardPointers.find(glslangMember.getReferentType()) == forwardPointers.end()) {
                    deferredForwardPointers.push_back(std::make_pair(&glslangMember, memberQualifier));
                }
                spvMembers.push_back(
                    convertGlslangToSpvType(glslangMember, explicitLayout, memberQualifier, lastBufferBlockMember,
                        true));
            } else {
                spvMembers.push_back(
                    convertGlslangToSpvType(glslangMember, explicitLayout, memberQualifier, lastBufferBlockMember,
                        false));
            }
        }
    }

    // Make the SPIR-V type
    spv::Id spvType = builder.makeStructType(spvMembers, type.getTypeName().c_str());
    if (! HasNonLayoutQualifiers(type, qualifier))
        structMap[explicitLayout][qualifier.layoutMatrix][glslangMembers] = spvType;

    // Decorate it
    decorateStructType(type, glslangMembers, explicitLayout, qualifier, spvType);

    for (int i = 0; i < (int)deferredForwardPointers.size(); ++i) {
        auto it = deferredForwardPointers[i];
        convertGlslangToSpvType(*it.first, explicitLayout, it.second, false);
    }

    return spvType;
}

void TGlslangToSpvTraverser::decorateStructType(const qglslang::TType& type,
                                                const qglslang::TTypeList* glslangMembers,
                                                qglslang::TLayoutPacking explicitLayout,
                                                const qglslang::TQualifier& qualifier,
                                                spv::Id spvType)
{
    // Name and decorate the non-hidden members
    int offset = -1;
    bool memberLocationInvalid = type.isArrayOfArrays() ||
        (type.isArray() && (type.getQualifier().isArrayedIo(glslangIntermediate->getStage()) == false));
    for (int i = 0; i < (int)glslangMembers->size(); i++) {
        qglslang::TType& glslangMember = *(*glslangMembers)[i].type;
        int member = i;
        if (type.getBasicType() == qglslang::EbtBlock) {
            member = memberRemapper[glslangTypeToIdMap[glslangMembers]][i];
            if (filterMember(glslangMember))
                continue;
        }

        // modify just this child's view of the qualifier
        qglslang::TQualifier memberQualifier = glslangMember.getQualifier();
        InheritQualifiers(memberQualifier, qualifier);

        // using -1 above to indicate a hidden member
        if (member < 0)
            continue;

        builder.addMemberName(spvType, member, glslangMember.getFieldName().c_str());
        builder.addMemberDecoration(spvType, member,
                                    TranslateLayoutDecoration(glslangMember, memberQualifier.layoutMatrix));
        builder.addMemberDecoration(spvType, member, TranslatePrecisionDecoration(glslangMember));
        // Add interpolation and auxiliary storage decorations only to
        // top-level members of Input and Output storage classes
        if (type.getQualifier().storage == qglslang::EvqVaryingIn ||
            type.getQualifier().storage == qglslang::EvqVaryingOut) {
            if (type.getBasicType() == qglslang::EbtBlock ||
                glslangIntermediate->getSource() == qglslang::EShSourceHlsl) {
                builder.addMemberDecoration(spvType, member, TranslateInterpolationDecoration(memberQualifier));
                builder.addMemberDecoration(spvType, member, TranslateAuxiliaryStorageDecoration(memberQualifier));
#ifndef GLSLANG_WEB
                addMeshNVDecoration(spvType, member, memberQualifier);
#endif
            }
        }
        builder.addMemberDecoration(spvType, member, TranslateInvariantDecoration(memberQualifier));

#ifndef GLSLANG_WEB
        if (type.getBasicType() == qglslang::EbtBlock &&
            qualifier.storage == qglslang::EvqBuffer) {
            // Add memory decorations only to top-level members of shader storage block
            std::vector<spv::Decoration> memory;
            TranslateMemoryDecoration(memberQualifier, memory, glslangIntermediate->usingVulkanMemoryModel());
            for (unsigned int i = 0; i < memory.size(); ++i)
                builder.addMemberDecoration(spvType, member, memory[i]);
        }

#endif

        // Location assignment was already completed correctly by the front end,
        // just track whether a member needs to be decorated.
        // Ignore member locations if the container is an array, as that's
        // ill-specified and decisions have been made to not allow this.
        if (!memberLocationInvalid && memberQualifier.hasLocation())
            builder.addMemberDecoration(spvType, member, spv::DecorationLocation, memberQualifier.layoutLocation);

        // component, XFB, others
        if (glslangMember.getQualifier().hasComponent())
            builder.addMemberDecoration(spvType, member, spv::DecorationComponent,
                                        glslangMember.getQualifier().layoutComponent);
        if (glslangMember.getQualifier().hasXfbOffset())
            builder.addMemberDecoration(spvType, member, spv::DecorationOffset,
                                        glslangMember.getQualifier().layoutXfbOffset);
        else if (explicitLayout != qglslang::ElpNone) {
            // figure out what to do with offset, which is accumulating
            int nextOffset;
            updateMemberOffset(type, glslangMember, offset, nextOffset, explicitLayout, memberQualifier.layoutMatrix);
            if (offset >= 0)
                builder.addMemberDecoration(spvType, member, spv::DecorationOffset, offset);
            offset = nextOffset;
        }

        if (glslangMember.isMatrix() && explicitLayout != qglslang::ElpNone)
            builder.addMemberDecoration(spvType, member, spv::DecorationMatrixStride,
                                        getMatrixStride(glslangMember, explicitLayout, memberQualifier.layoutMatrix));

        // built-in variable decorations
        spv::BuiltIn builtIn = TranslateBuiltInDecoration(glslangMember.getQualifier().builtIn, true);
        if (builtIn != spv::BuiltInMax)
            builder.addMemberDecoration(spvType, member, spv::DecorationBuiltIn, (int)builtIn);

#ifndef GLSLANG_WEB
        // nonuniform
        builder.addMemberDecoration(spvType, member, TranslateNonUniformDecoration(glslangMember.getQualifier()));

        if (glslangIntermediate->getHlslFunctionality1() && memberQualifier.semanticName != nullptr) {
            builder.addExtension("SPV_GOOGLE_hlsl_functionality1");
            builder.addMemberDecoration(spvType, member, (spv::Decoration)spv::DecorationHlslSemanticGOOGLE,
                                        memberQualifier.semanticName);
        }

        if (builtIn == spv::BuiltInLayer) {
            // SPV_NV_viewport_array2 extension
            if (glslangMember.getQualifier().layoutViewportRelative){
                builder.addMemberDecoration(spvType, member, (spv::Decoration)spv::DecorationViewportRelativeNV);
                builder.addCapability(spv::CapabilityShaderViewportMaskNV);
                builder.addExtension(spv::E_SPV_NV_viewport_array2);
            }
            if (glslangMember.getQualifier().layoutSecondaryViewportRelativeOffset != -2048){
                builder.addMemberDecoration(spvType, member,
                                            (spv::Decoration)spv::DecorationSecondaryViewportRelativeNV,
                                            glslangMember.getQualifier().layoutSecondaryViewportRelativeOffset);
                builder.addCapability(spv::CapabilityShaderStereoViewNV);
                builder.addExtension(spv::E_SPV_NV_stereo_view_rendering);
            }
        }
        if (glslangMember.getQualifier().layoutPassthrough) {
            builder.addMemberDecoration(spvType, member, (spv::Decoration)spv::DecorationPassthroughNV);
            builder.addCapability(spv::CapabilityGeometryShaderPassthroughNV);
            builder.addExtension(spv::E_SPV_NV_geometry_shader_passthrough);
        }

        //
        // Add SPIR-V decorations for members (GL_EXT_spirv_intrinsics)
        //
        if (glslangMember.getQualifier().hasSprivDecorate()) {
            const qglslang::TSpirvDecorate& spirvDecorate = glslangMember.getQualifier().getSpirvDecorate();

            // Add spirv_decorate
            for (auto& decorate : spirvDecorate.decorates) {
                if (!decorate.second.empty()) {
                    std::vector<unsigned> literals;
                    TranslateLiterals(decorate.second, literals);
                    builder.addMemberDecoration(spvType, member, static_cast<spv::Decoration>(decorate.first), literals);
                }
                else
                    builder.addMemberDecoration(spvType, member, static_cast<spv::Decoration>(decorate.first));
            }

            // spirv_decorate_id not applied to members
            assert(spirvDecorate.decorateIds.empty());

            // Add spirv_decorate_string
            for (auto& decorateString : spirvDecorate.decorateStrings) {
                std::vector<const char*> strings;
                assert(!decorateString.second.empty());
                for (auto extraOperand : decorateString.second) {
                    const char* string = extraOperand->getConstArray()[0].getSConst()->c_str();
                    strings.push_back(string);
                }
                builder.addDecoration(spvType, static_cast<spv::Decoration>(decorateString.first), strings);
            }
        }
#endif
    }

    // Decorate the structure
    builder.addDecoration(spvType, TranslateLayoutDecoration(type, qualifier.layoutMatrix));
    builder.addDecoration(spvType, TranslateBlockDecoration(type, glslangIntermediate->usingStorageBuffer()));
}

// Turn the expression forming the array size into an id.
// This is not quite trivial, because of specialization constants.
// Sometimes, a raw constant is turned into an Id, and sometimes
// a specialization constant expression is.
spv::Id TGlslangToSpvTraverser::makeArraySizeId(const qglslang::TArraySizes& arraySizes, int dim)
{
    // First, see if this is sized with a node, meaning a specialization constant:
    qglslang::TIntermTyped* specNode = arraySizes.getDimNode(dim);
    if (specNode != nullptr) {
        builder.clearAccessChain();
        SpecConstantOpModeGuard spec_constant_op_mode_setter(&builder);
        spec_constant_op_mode_setter.turnOnSpecConstantOpMode();
        specNode->traverse(this);
        return accessChainLoad(specNode->getAsTyped()->getType());
    }

    // Otherwise, need a compile-time (front end) size, get it:
    int size = arraySizes.getDimSize(dim);
    assert(size > 0);
    return builder.makeUintConstant(size);
}

// Wrap the builder's accessChainLoad to:
//  - localize handling of RelaxedPrecision
//  - use the SPIR-V inferred type instead of another conversion of the glslang type
//    (avoids unnecessary work and possible type punning for structures)
//  - do conversion of concrete to abstract type
spv::Id TGlslangToSpvTraverser::accessChainLoad(const qglslang::TType& type)
{
    spv::Id nominalTypeId = builder.accessChainGetInferredType();

    spv::Builder::AccessChain::CoherentFlags coherentFlags = builder.getAccessChain().coherentFlags;
    coherentFlags |= TranslateCoherent(type);

    unsigned int alignment = builder.getAccessChain().alignment;
    alignment |= type.getBufferReferenceAlignment();

    spv::Id loadedId = builder.accessChainLoad(TranslatePrecisionDecoration(type),
        TranslateNonUniformDecoration(builder.getAccessChain().coherentFlags),
        TranslateNonUniformDecoration(type.getQualifier()),
        nominalTypeId,
        spv::MemoryAccessMask(TranslateMemoryAccess(coherentFlags) & ~spv::MemoryAccessMakePointerAvailableKHRMask),
        TranslateMemoryScope(coherentFlags),
        alignment);

    // Need to convert to abstract types when necessary
    if (type.getBasicType() == qglslang::EbtBool) {
        loadedId = convertLoadedBoolInUniformToUint(type, nominalTypeId, loadedId);
    }

    return loadedId;
}

// Wrap the builder's accessChainStore to:
//  - do conversion of concrete to abstract type
//
// Implicitly uses the existing builder.accessChain as the storage target.
void TGlslangToSpvTraverser::accessChainStore(const qglslang::TType& type, spv::Id rvalue)
{
    // Need to convert to abstract types when necessary
    if (type.getBasicType() == qglslang::EbtBool) {
        spv::Id nominalTypeId = builder.accessChainGetInferredType();

        if (builder.isScalarType(nominalTypeId)) {
            // Conversion for bool
            spv::Id boolType = builder.makeBoolType();
            if (nominalTypeId != boolType) {
                // keep these outside arguments, for determinant order-of-evaluation
                spv::Id one = builder.makeUintConstant(1);
                spv::Id zero = builder.makeUintConstant(0);
                rvalue = builder.createTriOp(spv::OpSelect, nominalTypeId, rvalue, one, zero);
            } else if (builder.getTypeId(rvalue) != boolType)
                rvalue = builder.createBinOp(spv::OpINotEqual, boolType, rvalue, builder.makeUintConstant(0));
        } else if (builder.isVectorType(nominalTypeId)) {
            // Conversion for bvec
            int vecSize = builder.getNumTypeComponents(nominalTypeId);
            spv::Id bvecType = builder.makeVectorType(builder.makeBoolType(), vecSize);
            if (nominalTypeId != bvecType) {
                // keep these outside arguments, for determinant order-of-evaluation
                spv::Id one = makeSmearedConstant(builder.makeUintConstant(1), vecSize);
                spv::Id zero = makeSmearedConstant(builder.makeUintConstant(0), vecSize);
                rvalue = builder.createTriOp(spv::OpSelect, nominalTypeId, rvalue, one, zero);
            } else if (builder.getTypeId(rvalue) != bvecType)
                rvalue = builder.createBinOp(spv::OpINotEqual, bvecType, rvalue,
                                             makeSmearedConstant(builder.makeUintConstant(0), vecSize));
        }
    }

    spv::Builder::AccessChain::CoherentFlags coherentFlags = builder.getAccessChain().coherentFlags;
    coherentFlags |= TranslateCoherent(type);

    unsigned int alignment = builder.getAccessChain().alignment;
    alignment |= type.getBufferReferenceAlignment();

    builder.accessChainStore(rvalue, TranslateNonUniformDecoration(builder.getAccessChain().coherentFlags),
                             spv::MemoryAccessMask(TranslateMemoryAccess(coherentFlags) &
                                ~spv::MemoryAccessMakePointerVisibleKHRMask),
                             TranslateMemoryScope(coherentFlags), alignment);
}

// For storing when types match at the glslang level, but not might match at the
// SPIR-V level.
//
// This especially happens when a single glslang type expands to multiple
// SPIR-V types, like a struct that is used in a member-undecorated way as well
// as in a member-decorated way.
//
// NOTE: This function can handle any store request; if it's not special it
// simplifies to a simple OpStore.
//
// Implicitly uses the existing builder.accessChain as the storage target.
void TGlslangToSpvTraverser::multiTypeStore(const qglslang::TType& type, spv::Id rValue)
{
    // we only do the complex path here if it's an aggregate
    if (! type.isStruct() && ! type.isArray()) {
        accessChainStore(type, rValue);
        return;
    }

    // and, it has to be a case of type aliasing
    spv::Id rType = builder.getTypeId(rValue);
    spv::Id lValue = builder.accessChainGetLValue();
    spv::Id lType = builder.getContainedTypeId(builder.getTypeId(lValue));
    if (lType == rType) {
        accessChainStore(type, rValue);
        return;
    }

    // Recursively (as needed) copy an aggregate type to a different aggregate type,
    // where the two types were the same type in GLSL. This requires member
    // by member copy, recursively.

    // SPIR-V 1.4 added an instruction to do help do this.
    if (glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_4) {
        // However, bool in uniform space is changed to int, so
        // OpCopyLogical does not work for that.
        // TODO: It would be more robust to do a full recursive verification of the types satisfying SPIR-V rules.
        bool rBool = builder.containsType(builder.getTypeId(rValue), spv::OpTypeBool, 0);
        bool lBool = builder.containsType(lType, spv::OpTypeBool, 0);
        if (lBool == rBool) {
            spv::Id logicalCopy = builder.createUnaryOp(spv::OpCopyLogical, lType, rValue);
            accessChainStore(type, logicalCopy);
            return;
        }
    }

    // If an array, copy element by element.
    if (type.isArray()) {
        qglslang::TType glslangElementType(type, 0);
        spv::Id elementRType = builder.getContainedTypeId(rType);
        for (int index = 0; index < type.getOuterArraySize(); ++index) {
            // get the source member
            spv::Id elementRValue = builder.createCompositeExtract(rValue, elementRType, index);

            // set up the target storage
            builder.clearAccessChain();
            builder.setAccessChainLValue(lValue);
            builder.accessChainPush(builder.makeIntConstant(index), TranslateCoherent(type),
                type.getBufferReferenceAlignment());

            // store the member
            multiTypeStore(glslangElementType, elementRValue);
        }
    } else {
        assert(type.isStruct());

        // loop over structure members
        const qglslang::TTypeList& members = *type.getStruct();
        for (int m = 0; m < (int)members.size(); ++m) {
            const qglslang::TType& glslangMemberType = *members[m].type;

            // get the source member
            spv::Id memberRType = builder.getContainedTypeId(rType, m);
            spv::Id memberRValue = builder.createCompositeExtract(rValue, memberRType, m);

            // set up the target storage
            builder.clearAccessChain();
            builder.setAccessChainLValue(lValue);
            builder.accessChainPush(builder.makeIntConstant(m), TranslateCoherent(type),
                type.getBufferReferenceAlignment());

            // store the member
            multiTypeStore(glslangMemberType, memberRValue);
        }
    }
}

// Decide whether or not this type should be
// decorated with offsets and strides, and if so
// whether std140 or std430 rules should be applied.
qglslang::TLayoutPacking TGlslangToSpvTraverser::getExplicitLayout(const qglslang::TType& type) const
{
    // has to be a block
    if (type.getBasicType() != qglslang::EbtBlock)
        return qglslang::ElpNone;

    // has to be a uniform or buffer block or task in/out blocks
    if (type.getQualifier().storage != qglslang::EvqUniform &&
        type.getQualifier().storage != qglslang::EvqBuffer &&
        type.getQualifier().storage != qglslang::EvqShared &&
        !type.getQualifier().isTaskMemory())
        return qglslang::ElpNone;

    // return the layout to use
    switch (type.getQualifier().layoutPacking) {
    case qglslang::ElpStd140:
    case qglslang::ElpStd430:
    case qglslang::ElpScalar:
        return type.getQualifier().layoutPacking;
    default:
        return qglslang::ElpNone;
    }
}

// Given an array type, returns the integer stride required for that array
int TGlslangToSpvTraverser::getArrayStride(const qglslang::TType& arrayType, qglslang::TLayoutPacking explicitLayout,
    qglslang::TLayoutMatrix matrixLayout)
{
    int size;
    int stride;
    glslangIntermediate->getMemberAlignment(arrayType, size, stride, explicitLayout,
        matrixLayout == qglslang::ElmRowMajor);

    return stride;
}

// Given a matrix type, or array (of array) of matrixes type, returns the integer stride required for that matrix
// when used as a member of an interface block
int TGlslangToSpvTraverser::getMatrixStride(const qglslang::TType& matrixType, qglslang::TLayoutPacking explicitLayout,
    qglslang::TLayoutMatrix matrixLayout)
{
    qglslang::TType elementType;
    elementType.shallowCopy(matrixType);
    elementType.clearArraySizes();

    int size;
    int stride;
    glslangIntermediate->getMemberAlignment(elementType, size, stride, explicitLayout,
        matrixLayout == qglslang::ElmRowMajor);

    return stride;
}

// Given a member type of a struct, realign the current offset for it, and compute
// the next (not yet aligned) offset for the next member, which will get aligned
// on the next call.
// 'currentOffset' should be passed in already initialized, ready to modify, and reflecting
// the migration of data from nextOffset -> currentOffset.  It should be -1 on the first call.
// -1 means a non-forced member offset (no decoration needed).
void TGlslangToSpvTraverser::updateMemberOffset(const qglslang::TType& structType, const qglslang::TType& memberType,
    int& currentOffset, int& nextOffset, qglslang::TLayoutPacking explicitLayout, qglslang::TLayoutMatrix matrixLayout)
{
    // this will get a positive value when deemed necessary
    nextOffset = -1;

    // override anything in currentOffset with user-set offset
    if (memberType.getQualifier().hasOffset())
        currentOffset = memberType.getQualifier().layoutOffset;

    // It could be that current linker usage in glslang updated all the layoutOffset,
    // in which case the following code does not matter.  But, that's not quite right
    // once cross-compilation unit GLSL validation is done, as the original user
    // settings are needed in layoutOffset, and then the following will come into play.

    if (explicitLayout == qglslang::ElpNone) {
        if (! memberType.getQualifier().hasOffset())
            currentOffset = -1;

        return;
    }

    // Getting this far means we need explicit offsets
    if (currentOffset < 0)
        currentOffset = 0;

    // Now, currentOffset is valid (either 0, or from a previous nextOffset),
    // but possibly not yet correctly aligned.

    int memberSize;
    int dummyStride;
    int memberAlignment = glslangIntermediate->getMemberAlignment(memberType, memberSize, dummyStride, explicitLayout,
        matrixLayout == qglslang::ElmRowMajor);

    // Adjust alignment for HLSL rules
    // TODO: make this consistent in early phases of code:
    //       adjusting this late means inconsistencies with earlier code, which for reflection is an issue
    // Until reflection is brought in sync with these adjustments, don't apply to $Global,
    // which is the most likely to rely on reflection, and least likely to rely implicit layouts
    if (glslangIntermediate->usingHlslOffsets() &&
        ! memberType.isArray() && memberType.isVector() && structType.getTypeName().compare("$Global") != 0) {
        int dummySize;
        int componentAlignment = glslangIntermediate->getBaseAlignmentScalar(memberType, dummySize);
        if (componentAlignment <= 4)
            memberAlignment = componentAlignment;
    }

    // Bump up to member alignment
    qglslang::RoundToPow2(currentOffset, memberAlignment);

    // Bump up to vec4 if there is a bad straddle
    if (explicitLayout != qglslang::ElpScalar && glslangIntermediate->improperStraddle(memberType, memberSize,
        currentOffset))
        qglslang::RoundToPow2(currentOffset, 16);

    nextOffset = currentOffset + memberSize;
}

void TGlslangToSpvTraverser::declareUseOfStructMember(const qglslang::TTypeList& members, int glslangMember)
{
    const qglslang::TBuiltInVariable glslangBuiltIn = members[glslangMember].type->getQualifier().builtIn;
    switch (glslangBuiltIn)
    {
    case qglslang::EbvPointSize:
#ifndef GLSLANG_WEB
    case qglslang::EbvClipDistance:
    case qglslang::EbvCullDistance:
    case qglslang::EbvViewportMaskNV:
    case qglslang::EbvSecondaryPositionNV:
    case qglslang::EbvSecondaryViewportMaskNV:
    case qglslang::EbvPositionPerViewNV:
    case qglslang::EbvViewportMaskPerViewNV:
    case qglslang::EbvTaskCountNV:
    case qglslang::EbvPrimitiveCountNV:
    case qglslang::EbvPrimitiveIndicesNV:
    case qglslang::EbvClipDistancePerViewNV:
    case qglslang::EbvCullDistancePerViewNV:
    case qglslang::EbvLayerPerViewNV:
    case qglslang::EbvMeshViewCountNV:
    case qglslang::EbvMeshViewIndicesNV:
#endif
        // Generate the associated capability.  Delegate to TranslateBuiltInDecoration.
        // Alternately, we could just call this for any glslang built-in, since the
        // capability already guards against duplicates.
        TranslateBuiltInDecoration(glslangBuiltIn, false);
        break;
    default:
        // Capabilities were already generated when the struct was declared.
        break;
    }
}

bool TGlslangToSpvTraverser::isShaderEntryPoint(const qglslang::TIntermAggregate* node)
{
    return node->getName().compare(glslangIntermediate->getEntryPointMangledName().c_str()) == 0;
}

// Does parameter need a place to keep writes, separate from the original?
// Assumes called after originalParam(), which filters out block/buffer/opaque-based
// qualifiers such that we should have only in/out/inout/constreadonly here.
bool TGlslangToSpvTraverser::writableParam(qglslang::TStorageQualifier qualifier) const
{
    assert(qualifier == qglslang::EvqIn ||
           qualifier == qglslang::EvqOut ||
           qualifier == qglslang::EvqInOut ||
           qualifier == qglslang::EvqUniform ||
           qualifier == qglslang::EvqConstReadOnly);
    return qualifier != qglslang::EvqConstReadOnly &&
           qualifier != qglslang::EvqUniform;
}

// Is parameter pass-by-original?
bool TGlslangToSpvTraverser::originalParam(qglslang::TStorageQualifier qualifier, const qglslang::TType& paramType,
                                           bool implicitThisParam)
{
    if (implicitThisParam)                                                                     // implicit this
        return true;
    if (glslangIntermediate->getSource() == qglslang::EShSourceHlsl)
        return paramType.getBasicType() == qglslang::EbtBlock;
    return paramType.containsOpaque() ||                                                       // sampler, etc.
#ifndef GLSLANG_WEB
           paramType.getQualifier().isSpirvByReference() ||                                    // spirv_by_reference
#endif
           (paramType.getBasicType() == qglslang::EbtBlock && qualifier == qglslang::EvqBuffer); // SSBO
}

// Make all the functions, skeletally, without actually visiting their bodies.
void TGlslangToSpvTraverser::makeFunctions(const qglslang::TIntermSequence& glslFunctions)
{
    const auto getParamDecorations = [&](std::vector<spv::Decoration>& decorations, const qglslang::TType& type,
        bool useVulkanMemoryModel) {
        spv::Decoration paramPrecision = TranslatePrecisionDecoration(type);
        if (paramPrecision != spv::NoPrecision)
            decorations.push_back(paramPrecision);
        TranslateMemoryDecoration(type.getQualifier(), decorations, useVulkanMemoryModel);
        if (type.isReference()) {
            // Original and non-writable params pass the pointer directly and
            // use restrict/aliased, others are stored to a pointer in Function
            // memory and use RestrictPointer/AliasedPointer.
            if (originalParam(type.getQualifier().storage, type, false) ||
                !writableParam(type.getQualifier().storage)) {
                decorations.push_back(type.getQualifier().isRestrict() ? spv::DecorationRestrict :
                                                                         spv::DecorationAliased);
            } else {
                decorations.push_back(type.getQualifier().isRestrict() ? spv::DecorationRestrictPointerEXT :
                                                                         spv::DecorationAliasedPointerEXT);
            }
        }
    };

    for (int f = 0; f < (int)glslFunctions.size(); ++f) {
        qglslang::TIntermAggregate* glslFunction = glslFunctions[f]->getAsAggregate();
        if (! glslFunction || glslFunction->getOp() != qglslang::EOpFunction || isShaderEntryPoint(glslFunction))
            continue;

        // We're on a user function.  Set up the basic interface for the function now,
        // so that it's available to call.  Translating the body will happen later.
        //
        // Typically (except for a "const in" parameter), an address will be passed to the
        // function.  What it is an address of varies:
        //
        // - "in" parameters not marked as "const" can be written to without modifying the calling
        //   argument so that write needs to be to a copy, hence the address of a copy works.
        //
        // - "const in" parameters can just be the r-value, as no writes need occur.
        //
        // - "out" and "inout" arguments can't be done as pointers to the calling argument, because
        //   GLSL has copy-in/copy-out semantics.  They can be handled though with a pointer to a copy.

        std::vector<spv::Id> paramTypes;
        std::vector<std::vector<spv::Decoration>> paramDecorations; // list of decorations per parameter
        qglslang::TIntermSequence& parameters = glslFunction->getSequence()[0]->getAsAggregate()->getSequence();

#ifdef ENABLE_HLSL
        bool implicitThis = (int)parameters.size() > 0 && parameters[0]->getAsSymbolNode()->getName() ==
                                                          glslangIntermediate->implicitThisName;
#else
        bool implicitThis = false;
#endif

        paramDecorations.resize(parameters.size());
        for (int p = 0; p < (int)parameters.size(); ++p) {
            const qglslang::TType& paramType = parameters[p]->getAsTyped()->getType();
            spv::Id typeId = convertGlslangToSpvType(paramType);
            if (originalParam(paramType.getQualifier().storage, paramType, implicitThis && p == 0))
                typeId = builder.makePointer(TranslateStorageClass(paramType), typeId);
            else if (writableParam(paramType.getQualifier().storage))
                typeId = builder.makePointer(spv::StorageClassFunction, typeId);
            else
                rValueParameters.insert(parameters[p]->getAsSymbolNode()->getId());
            getParamDecorations(paramDecorations[p], paramType, glslangIntermediate->usingVulkanMemoryModel());
            paramTypes.push_back(typeId);
        }

        spv::Block* functionBlock;
        spv::Function *function = builder.makeFunctionEntry(TranslatePrecisionDecoration(glslFunction->getType()),
                                                            convertGlslangToSpvType(glslFunction->getType()),
                                                            glslFunction->getName().c_str(), paramTypes,
                                                            paramDecorations, &functionBlock);
        if (implicitThis)
            function->setImplicitThis();

        // Track function to emit/call later
        functionMap[glslFunction->getName().c_str()] = function;

        // Set the parameter id's
        for (int p = 0; p < (int)parameters.size(); ++p) {
            symbolValues[parameters[p]->getAsSymbolNode()->getId()] = function->getParamId(p);
            // give a name too
            builder.addName(function->getParamId(p), parameters[p]->getAsSymbolNode()->getName().c_str());

            const qglslang::TType& paramType = parameters[p]->getAsTyped()->getType();
            if (paramType.contains8BitInt())
                builder.addCapability(spv::CapabilityInt8);
            if (paramType.contains16BitInt())
                builder.addCapability(spv::CapabilityInt16);
            if (paramType.contains16BitFloat())
                builder.addCapability(spv::CapabilityFloat16);
        }
    }
}

// Process all the initializers, while skipping the functions and link objects
void TGlslangToSpvTraverser::makeGlobalInitializers(const qglslang::TIntermSequence& initializers)
{
    builder.setBuildPoint(shaderEntry->getLastBlock());
    for (int i = 0; i < (int)initializers.size(); ++i) {
        qglslang::TIntermAggregate* initializer = initializers[i]->getAsAggregate();
        if (initializer && initializer->getOp() != qglslang::EOpFunction && initializer->getOp() !=
            qglslang::EOpLinkerObjects) {

            // We're on a top-level node that's not a function.  Treat as an initializer, whose
            // code goes into the beginning of the entry point.
            initializer->traverse(this);
        }
    }
}
// Walk over all linker objects to create a map for payload and callable data linker objects
// and their location to be used during codegen for OpTraceKHR and OpExecuteCallableKHR
// This is done here since it is possible that these linker objects are not be referenced in the AST
void TGlslangToSpvTraverser::collectRayTracingLinkerObjects()
{
    qglslang::TIntermAggregate* linkerObjects = glslangIntermediate->findLinkerObjects();
    for (auto& objSeq : linkerObjects->getSequence()) {
        auto objNode = objSeq->getAsSymbolNode();
        if (objNode != nullptr) {
            if (objNode->getQualifier().hasLocation()) {
                unsigned int location = objNode->getQualifier().layoutLocation;
                auto st = objNode->getQualifier().storage;
                int set;
                switch (st)
                {
                case qglslang::EvqPayload:
                case qglslang::EvqPayloadIn:
                    set = 0;
                    break;
                case qglslang::EvqCallableData:
                case qglslang::EvqCallableDataIn:
                    set = 1;
                    break;

                default:
                    set = -1;
                }
                if (set != -1)
                    locationToSymbol[set].insert(std::make_pair(location, objNode));
            }
        }
    }
}
// Process all the functions, while skipping initializers.
void TGlslangToSpvTraverser::visitFunctions(const qglslang::TIntermSequence& glslFunctions)
{
    for (int f = 0; f < (int)glslFunctions.size(); ++f) {
        qglslang::TIntermAggregate* node = glslFunctions[f]->getAsAggregate();
        if (node && (node->getOp() == qglslang::EOpFunction || node->getOp() == qglslang::EOpLinkerObjects))
            node->traverse(this);
    }
}

void TGlslangToSpvTraverser::handleFunctionEntry(const qglslang::TIntermAggregate* node)
{
    // SPIR-V functions should already be in the functionMap from the prepass
    // that called makeFunctions().
    currentFunction = functionMap[node->getName().c_str()];
    spv::Block* functionBlock = currentFunction->getEntryBlock();
    builder.setBuildPoint(functionBlock);
}

void TGlslangToSpvTraverser::translateArguments(const qglslang::TIntermAggregate& node, std::vector<spv::Id>& arguments,
    spv::Builder::AccessChain::CoherentFlags &lvalueCoherentFlags)
{
    const qglslang::TIntermSequence& glslangArguments = node.getSequence();

    qglslang::TSampler sampler = {};
    bool cubeCompare = false;
#ifndef GLSLANG_WEB
    bool f16ShadowCompare = false;
#endif
    if (node.isTexture() || node.isImage()) {
        sampler = glslangArguments[0]->getAsTyped()->getType().getSampler();
        cubeCompare = sampler.dim == qglslang::EsdCube && sampler.arrayed && sampler.shadow;
#ifndef GLSLANG_WEB
        f16ShadowCompare = sampler.shadow &&
            glslangArguments[1]->getAsTyped()->getType().getBasicType() == qglslang::EbtFloat16;
#endif
    }

    for (int i = 0; i < (int)glslangArguments.size(); ++i) {
        builder.clearAccessChain();
        glslangArguments[i]->traverse(this);

#ifndef GLSLANG_WEB
        // Special case l-value operands
        bool lvalue = false;
        switch (node.getOp()) {
        case qglslang::EOpImageAtomicAdd:
        case qglslang::EOpImageAtomicMin:
        case qglslang::EOpImageAtomicMax:
        case qglslang::EOpImageAtomicAnd:
        case qglslang::EOpImageAtomicOr:
        case qglslang::EOpImageAtomicXor:
        case qglslang::EOpImageAtomicExchange:
        case qglslang::EOpImageAtomicCompSwap:
        case qglslang::EOpImageAtomicLoad:
        case qglslang::EOpImageAtomicStore:
            if (i == 0)
                lvalue = true;
            break;
        case qglslang::EOpSparseImageLoad:
            if ((sampler.ms && i == 3) || (! sampler.ms && i == 2))
                lvalue = true;
            break;
        case qglslang::EOpSparseTexture:
            if (((cubeCompare || f16ShadowCompare) && i == 3) || (! (cubeCompare || f16ShadowCompare) && i == 2))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureClamp:
            if (((cubeCompare || f16ShadowCompare) && i == 4) || (! (cubeCompare || f16ShadowCompare) && i == 3))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureLod:
        case qglslang::EOpSparseTextureOffset:
            if  ((f16ShadowCompare && i == 4) || (! f16ShadowCompare && i == 3))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureFetch:
            if ((sampler.dim != qglslang::EsdRect && i == 3) || (sampler.dim == qglslang::EsdRect && i == 2))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureFetchOffset:
            if ((sampler.dim != qglslang::EsdRect && i == 4) || (sampler.dim == qglslang::EsdRect && i == 3))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureLodOffset:
        case qglslang::EOpSparseTextureGrad:
        case qglslang::EOpSparseTextureOffsetClamp:
            if ((f16ShadowCompare && i == 5) || (! f16ShadowCompare && i == 4))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureGradOffset:
        case qglslang::EOpSparseTextureGradClamp:
            if ((f16ShadowCompare && i == 6) || (! f16ShadowCompare && i == 5))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureGradOffsetClamp:
            if ((f16ShadowCompare && i == 7) || (! f16ShadowCompare && i == 6))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureGather:
            if ((sampler.shadow && i == 3) || (! sampler.shadow && i == 2))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureGatherOffset:
        case qglslang::EOpSparseTextureGatherOffsets:
            if ((sampler.shadow && i == 4) || (! sampler.shadow && i == 3))
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureGatherLod:
            if (i == 3)
                lvalue = true;
            break;
        case qglslang::EOpSparseTextureGatherLodOffset:
        case qglslang::EOpSparseTextureGatherLodOffsets:
            if (i == 4)
                lvalue = true;
            break;
        case qglslang::EOpSparseImageLoadLod:
            if (i == 3)
                lvalue = true;
            break;
        case qglslang::EOpImageSampleFootprintNV:
            if (i == 4)
                lvalue = true;
            break;
        case qglslang::EOpImageSampleFootprintClampNV:
        case qglslang::EOpImageSampleFootprintLodNV:
            if (i == 5)
                lvalue = true;
            break;
        case qglslang::EOpImageSampleFootprintGradNV:
            if (i == 6)
                lvalue = true;
            break;
        case qglslang::EOpImageSampleFootprintGradClampNV:
            if (i == 7)
                lvalue = true;
            break;
        default:
            break;
        }

        if (lvalue) {
            spv::Id lvalue_id = builder.accessChainGetLValue();
            arguments.push_back(lvalue_id);
            lvalueCoherentFlags = builder.getAccessChain().coherentFlags;
            builder.addDecoration(lvalue_id, TranslateNonUniformDecoration(lvalueCoherentFlags));
            lvalueCoherentFlags |= TranslateCoherent(glslangArguments[i]->getAsTyped()->getType());
        } else
#endif
            arguments.push_back(accessChainLoad(glslangArguments[i]->getAsTyped()->getType()));
    }
}

void TGlslangToSpvTraverser::translateArguments(qglslang::TIntermUnary& node, std::vector<spv::Id>& arguments)
{
    builder.clearAccessChain();
    node.getOperand()->traverse(this);
    arguments.push_back(accessChainLoad(node.getOperand()->getType()));
}

spv::Id TGlslangToSpvTraverser::createImageTextureFunctionCall(qglslang::TIntermOperator* node)
{
    if (! node->isImage() && ! node->isTexture())
        return spv::NoResult;

    builder.setLine(node->getLoc().line, node->getLoc().getFilename());

    // Process a GLSL texturing op (will be SPV image)

    const qglslang::TType &imageType = node->getAsAggregate()
                                        ? node->getAsAggregate()->getSequence()[0]->getAsTyped()->getType()
                                        : node->getAsUnaryNode()->getOperand()->getAsTyped()->getType();
    const qglslang::TSampler sampler = imageType.getSampler();
#ifdef GLSLANG_WEB
    const bool f16ShadowCompare = false;
#else
    bool f16ShadowCompare = (sampler.shadow && node->getAsAggregate())
            ? node->getAsAggregate()->getSequence()[1]->getAsTyped()->getType().getBasicType() == qglslang::EbtFloat16
            : false;
#endif

    const auto signExtensionMask = [&]() {
        if (builder.getSpvVersion() >= spv::Spv_1_4) {
            if (sampler.type == qglslang::EbtUint)
                return spv::ImageOperandsZeroExtendMask;
            else if (sampler.type == qglslang::EbtInt)
                return spv::ImageOperandsSignExtendMask;
        }
        return spv::ImageOperandsMaskNone;
    };

    spv::Builder::AccessChain::CoherentFlags lvalueCoherentFlags;

    std::vector<spv::Id> arguments;
    if (node->getAsAggregate())
        translateArguments(*node->getAsAggregate(), arguments, lvalueCoherentFlags);
    else
        translateArguments(*node->getAsUnaryNode(), arguments);
    spv::Decoration precision = TranslatePrecisionDecoration(node->getType());

    spv::Builder::TextureParameters params = { };
    params.sampler = arguments[0];

    qglslang::TCrackedTextureOp cracked;
    node->crackTexture(sampler, cracked);

    const bool isUnsignedResult = node->getType().getBasicType() == qglslang::EbtUint;

    if (builder.isSampledImage(params.sampler) &&
        ((cracked.query && node->getOp() != qglslang::EOpTextureQueryLod) || cracked.fragMask || cracked.fetch)) {
        params.sampler = builder.createUnaryOp(spv::OpImage, builder.getImageType(params.sampler), params.sampler);
        if (imageType.getQualifier().isNonUniform()) {
            builder.addDecoration(params.sampler, spv::DecorationNonUniformEXT);
        }
    }
    // Check for queries
    if (cracked.query) {
        switch (node->getOp()) {
        case qglslang::EOpImageQuerySize:
        case qglslang::EOpTextureQuerySize:
            if (arguments.size() > 1) {
                params.lod = arguments[1];
                return builder.createTextureQueryCall(spv::OpImageQuerySizeLod, params, isUnsignedResult);
            } else
                return builder.createTextureQueryCall(spv::OpImageQuerySize, params, isUnsignedResult);
#ifndef GLSLANG_WEB
        case qglslang::EOpImageQuerySamples:
        case qglslang::EOpTextureQuerySamples:
            return builder.createTextureQueryCall(spv::OpImageQuerySamples, params, isUnsignedResult);
        case qglslang::EOpTextureQueryLod:
            params.coords = arguments[1];
            return builder.createTextureQueryCall(spv::OpImageQueryLod, params, isUnsignedResult);
        case qglslang::EOpTextureQueryLevels:
            return builder.createTextureQueryCall(spv::OpImageQueryLevels, params, isUnsignedResult);
        case qglslang::EOpSparseTexelsResident:
            return builder.createUnaryOp(spv::OpImageSparseTexelsResident, builder.makeBoolType(), arguments[0]);
#endif
        default:
            assert(0);
            break;
        }
    }

    int components = node->getType().getVectorSize();

    if (node->getOp() == qglslang::EOpImageLoad ||
        node->getOp() == qglslang::EOpImageLoadLod ||
        node->getOp() == qglslang::EOpTextureFetch ||
        node->getOp() == qglslang::EOpTextureFetchOffset) {
        // These must produce 4 components, per SPIR-V spec.  We'll add a conversion constructor if needed.
        // This will only happen through the HLSL path for operator[], so we do not have to handle e.g.
        // the EOpTexture/Proj/Lod/etc family.  It would be harmless to do so, but would need more logic
        // here around e.g. which ones return scalars or other types.
        components = 4;
    }

    qglslang::TType returnType(node->getType().getBasicType(), qglslang::EvqTemporary, components);

    auto resultType = [&returnType,this]{ return convertGlslangToSpvType(returnType); };

    // Check for image functions other than queries
    if (node->isImage()) {
        std::vector<spv::IdImmediate> operands;
        auto opIt = arguments.begin();
        spv::IdImmediate image = { true, *(opIt++) };
        operands.push_back(image);

        // Handle subpass operations
        // TODO: GLSL should change to have the "MS" only on the type rather than the
        // built-in function.
        if (cracked.subpass) {
            // add on the (0,0) coordinate
            spv::Id zero = builder.makeIntConstant(0);
            std::vector<spv::Id> comps;
            comps.push_back(zero);
            comps.push_back(zero);
            spv::IdImmediate coord = { true,
                builder.makeCompositeConstant(builder.makeVectorType(builder.makeIntType(32), 2), comps) };
            operands.push_back(coord);
            spv::IdImmediate imageOperands = { false, spv::ImageOperandsMaskNone };
            imageOperands.word = imageOperands.word | signExtensionMask();
            if (sampler.isMultiSample()) {
                imageOperands.word = imageOperands.word | spv::ImageOperandsSampleMask;
            }
            if (imageOperands.word != spv::ImageOperandsMaskNone) {
                operands.push_back(imageOperands);
                if (sampler.isMultiSample()) {
                    spv::IdImmediate imageOperand = { true, *(opIt++) };
                    operands.push_back(imageOperand);
                }
            }
            spv::Id result = builder.createOp(spv::OpImageRead, resultType(), operands);
            builder.setPrecision(result, precision);
            return result;
        }

        spv::IdImmediate coord = { true, *(opIt++) };
        operands.push_back(coord);
        if (node->getOp() == qglslang::EOpImageLoad || node->getOp() == qglslang::EOpImageLoadLod) {
            spv::ImageOperandsMask mask = spv::ImageOperandsMaskNone;
            if (sampler.isMultiSample()) {
                mask = mask | spv::ImageOperandsSampleMask;
            }
            if (cracked.lod) {
                builder.addExtension(spv::E_SPV_AMD_shader_image_load_store_lod);
                builder.addCapability(spv::CapabilityImageReadWriteLodAMD);
                mask = mask | spv::ImageOperandsLodMask;
            }
            mask = mask | TranslateImageOperands(TranslateCoherent(imageType));
            mask = (spv::ImageOperandsMask)(mask & ~spv::ImageOperandsMakeTexelAvailableKHRMask);
            mask = mask | signExtensionMask();
            if (mask != spv::ImageOperandsMaskNone) {
                spv::IdImmediate imageOperands = { false, (unsigned int)mask };
                operands.push_back(imageOperands);
            }
            if (mask & spv::ImageOperandsSampleMask) {
                spv::IdImmediate imageOperand = { true, *opIt++ };
                operands.push_back(imageOperand);
            }
            if (mask & spv::ImageOperandsLodMask) {
                spv::IdImmediate imageOperand = { true, *opIt++ };
                operands.push_back(imageOperand);
            }
            if (mask & spv::ImageOperandsMakeTexelVisibleKHRMask) {
                spv::IdImmediate imageOperand = { true,
                                    builder.makeUintConstant(TranslateMemoryScope(TranslateCoherent(imageType))) };
                operands.push_back(imageOperand);
            }

            if (builder.getImageTypeFormat(builder.getImageType(operands.front().word)) == spv::ImageFormatUnknown)
                builder.addCapability(spv::CapabilityStorageImageReadWithoutFormat);

            std::vector<spv::Id> result(1, builder.createOp(spv::OpImageRead, resultType(), operands));
            builder.setPrecision(result[0], precision);

            // If needed, add a conversion constructor to the proper size.
            if (components != node->getType().getVectorSize())
                result[0] = builder.createConstructor(precision, result, convertGlslangToSpvType(node->getType()));

            return result[0];
        } else if (node->getOp() == qglslang::EOpImageStore || node->getOp() == qglslang::EOpImageStoreLod) {

            // Push the texel value before the operands
            if (sampler.isMultiSample() || cracked.lod) {
                spv::IdImmediate texel = { true, *(opIt + 1) };
                operands.push_back(texel);
            } else {
                spv::IdImmediate texel = { true, *opIt };
                operands.push_back(texel);
            }

            spv::ImageOperandsMask mask = spv::ImageOperandsMaskNone;
            if (sampler.isMultiSample()) {
                mask = mask | spv::ImageOperandsSampleMask;
            }
            if (cracked.lod) {
                builder.addExtension(spv::E_SPV_AMD_shader_image_load_store_lod);
                builder.addCapability(spv::CapabilityImageReadWriteLodAMD);
                mask = mask | spv::ImageOperandsLodMask;
            }
            mask = mask | TranslateImageOperands(TranslateCoherent(imageType));
            mask = (spv::ImageOperandsMask)(mask & ~spv::ImageOperandsMakeTexelVisibleKHRMask);
            mask = mask | signExtensionMask();
            if (mask != spv::ImageOperandsMaskNone) {
                spv::IdImmediate imageOperands = { false, (unsigned int)mask };
                operands.push_back(imageOperands);
            }
            if (mask & spv::ImageOperandsSampleMask) {
                spv::IdImmediate imageOperand = { true, *opIt++ };
                operands.push_back(imageOperand);
            }
            if (mask & spv::ImageOperandsLodMask) {
                spv::IdImmediate imageOperand = { true, *opIt++ };
                operands.push_back(imageOperand);
            }
            if (mask & spv::ImageOperandsMakeTexelAvailableKHRMask) {
                spv::IdImmediate imageOperand = { true,
                    builder.makeUintConstant(TranslateMemoryScope(TranslateCoherent(imageType))) };
                operands.push_back(imageOperand);
            }

            builder.createNoResultOp(spv::OpImageWrite, operands);
            if (builder.getImageTypeFormat(builder.getImageType(operands.front().word)) == spv::ImageFormatUnknown)
                builder.addCapability(spv::CapabilityStorageImageWriteWithoutFormat);
            return spv::NoResult;
        } else if (node->getOp() == qglslang::EOpSparseImageLoad ||
                   node->getOp() == qglslang::EOpSparseImageLoadLod) {
            builder.addCapability(spv::CapabilitySparseResidency);
            if (builder.getImageTypeFormat(builder.getImageType(operands.front().word)) == spv::ImageFormatUnknown)
                builder.addCapability(spv::CapabilityStorageImageReadWithoutFormat);

            spv::ImageOperandsMask mask = spv::ImageOperandsMaskNone;
            if (sampler.isMultiSample()) {
                mask = mask | spv::ImageOperandsSampleMask;
            }
            if (cracked.lod) {
                builder.addExtension(spv::E_SPV_AMD_shader_image_load_store_lod);
                builder.addCapability(spv::CapabilityImageReadWriteLodAMD);

                mask = mask | spv::ImageOperandsLodMask;
            }
            mask = mask | TranslateImageOperands(TranslateCoherent(imageType));
            mask = (spv::ImageOperandsMask)(mask & ~spv::ImageOperandsMakeTexelAvailableKHRMask);
            mask = mask | signExtensionMask();
            if (mask != spv::ImageOperandsMaskNone) {
                spv::IdImmediate imageOperands = { false, (unsigned int)mask };
                operands.push_back(imageOperands);
            }
            if (mask & spv::ImageOperandsSampleMask) {
                spv::IdImmediate imageOperand = { true, *opIt++ };
                operands.push_back(imageOperand);
            }
            if (mask & spv::ImageOperandsLodMask) {
                spv::IdImmediate imageOperand = { true, *opIt++ };
                operands.push_back(imageOperand);
            }
            if (mask & spv::ImageOperandsMakeTexelVisibleKHRMask) {
                spv::IdImmediate imageOperand = { true, builder.makeUintConstant(TranslateMemoryScope(
                    TranslateCoherent(imageType))) };
                operands.push_back(imageOperand);
            }

            // Create the return type that was a special structure
            spv::Id texelOut = *opIt;
            spv::Id typeId0 = resultType();
            spv::Id typeId1 = builder.getDerefTypeId(texelOut);
            spv::Id resultTypeId = builder.makeStructResultType(typeId0, typeId1);

            spv::Id resultId = builder.createOp(spv::OpImageSparseRead, resultTypeId, operands);

            // Decode the return type
            builder.createStore(builder.createCompositeExtract(resultId, typeId1, 1), texelOut);
            return builder.createCompositeExtract(resultId, typeId0, 0);
        } else {
            // Process image atomic operations

            // GLSL "IMAGE_PARAMS" will involve in constructing an image texel pointer and this pointer,
            // as the first source operand, is required by SPIR-V atomic operations.
            // For non-MS, the sample value should be 0
            spv::IdImmediate sample = { true, sampler.isMultiSample() ? *(opIt++) : builder.makeUintConstant(0) };
            operands.push_back(sample);

            spv::Id resultTypeId;
            // imageAtomicStore has a void return type so base the pointer type on
            // the type of the value operand.
            if (node->getOp() == qglslang::EOpImageAtomicStore) {
                resultTypeId = builder.makePointer(spv::StorageClassImage, builder.getTypeId(*opIt));
            } else {
                resultTypeId = builder.makePointer(spv::StorageClassImage, resultType());
            }
            spv::Id pointer = builder.createOp(spv::OpImageTexelPointer, resultTypeId, operands);
            if (imageType.getQualifier().nonUniform) {
                builder.addDecoration(pointer, spv::DecorationNonUniformEXT);
            }

            std::vector<spv::Id> operands;
            operands.push_back(pointer);
            for (; opIt != arguments.end(); ++opIt)
                operands.push_back(*opIt);

            return createAtomicOperation(node->getOp(), precision, resultType(), operands, node->getBasicType(),
                lvalueCoherentFlags);
        }
    }

#ifndef GLSLANG_WEB
    // Check for fragment mask functions other than queries
    if (cracked.fragMask) {
        assert(sampler.ms);

        auto opIt = arguments.begin();
        std::vector<spv::Id> operands;

        operands.push_back(params.sampler);
        ++opIt;

        if (sampler.isSubpass()) {
            // add on the (0,0) coordinate
            spv::Id zero = builder.makeIntConstant(0);
            std::vector<spv::Id> comps;
            comps.push_back(zero);
            comps.push_back(zero);
            operands.push_back(builder.makeCompositeConstant(
                builder.makeVectorType(builder.makeIntType(32), 2), comps));
        }

        for (; opIt != arguments.end(); ++opIt)
            operands.push_back(*opIt);

        spv::Op fragMaskOp = spv::OpNop;
        if (node->getOp() == qglslang::EOpFragmentMaskFetch)
            fragMaskOp = spv::OpFragmentMaskFetchAMD;
        else if (node->getOp() == qglslang::EOpFragmentFetch)
            fragMaskOp = spv::OpFragmentFetchAMD;

        builder.addExtension(spv::E_SPV_AMD_shader_fragment_mask);
        builder.addCapability(spv::CapabilityFragmentMaskAMD);
        return builder.createOp(fragMaskOp, resultType(), operands);
    }
#endif

    // Check for texture functions other than queries
    bool sparse = node->isSparseTexture();
    bool imageFootprint = node->isImageFootprint();
    bool cubeCompare = sampler.dim == qglslang::EsdCube && sampler.isArrayed() && sampler.isShadow();

    // check for bias argument
    bool bias = false;
    if (! cracked.lod && ! cracked.grad && ! cracked.fetch && ! cubeCompare) {
        int nonBiasArgCount = 2;
        if (cracked.gather)
            ++nonBiasArgCount; // comp argument should be present when bias argument is present

        if (f16ShadowCompare)
            ++nonBiasArgCount;
        if (cracked.offset)
            ++nonBiasArgCount;
        else if (cracked.offsets)
            ++nonBiasArgCount;
        if (cracked.grad)
            nonBiasArgCount += 2;
        if (cracked.lodClamp)
            ++nonBiasArgCount;
        if (sparse)
            ++nonBiasArgCount;
        if (imageFootprint)
            //Following three extra arguments
            // int granularity, bool coarse, out gl_TextureFootprint2DNV footprint
            nonBiasArgCount += 3;
        if ((int)arguments.size() > nonBiasArgCount)
            bias = true;
    }

#ifndef GLSLANG_WEB
    if (cracked.gather) {
        const auto& sourceExtensions = glslangIntermediate->getRequestedExtensions();
        if (bias || cracked.lod ||
            sourceExtensions.find(qglslang::E_GL_AMD_texture_gather_bias_lod) != sourceExtensions.end()) {
            builder.addExtension(spv::E_SPV_AMD_texture_gather_bias_lod);
            builder.addCapability(spv::CapabilityImageGatherBiasLodAMD);
        }
    }
#endif

    // set the rest of the arguments

    params.coords = arguments[1];
    int extraArgs = 0;
    bool noImplicitLod = false;

    // sort out where Dref is coming from
    if (cubeCompare || f16ShadowCompare) {
        params.Dref = arguments[2];
        ++extraArgs;
    } else if (sampler.shadow && cracked.gather) {
        params.Dref = arguments[2];
        ++extraArgs;
    } else if (sampler.shadow) {
        std::vector<spv::Id> indexes;
        int dRefComp;
        if (cracked.proj)
            dRefComp = 2;  // "The resulting 3rd component of P in the shadow forms is used as Dref"
        else
            dRefComp = builder.getNumComponents(params.coords) - 1;
        indexes.push_back(dRefComp);
        params.Dref = builder.createCompositeExtract(params.coords,
            builder.getScalarTypeId(builder.getTypeId(params.coords)), indexes);
    }

    // lod
    if (cracked.lod) {
        params.lod = arguments[2 + extraArgs];
        ++extraArgs;
    } else if (glslangIntermediate->getStage() != EShLangFragment &&
               !(glslangIntermediate->getStage() == EShLangCompute &&
                 glslangIntermediate->hasLayoutDerivativeModeNone())) {
        // we need to invent the default lod for an explicit lod instruction for a non-fragment stage
        noImplicitLod = true;
    }

    // multisample
    if (sampler.isMultiSample()) {
        params.sample = arguments[2 + extraArgs]; // For MS, "sample" should be specified
        ++extraArgs;
    }

    // gradient
    if (cracked.grad) {
        params.gradX = arguments[2 + extraArgs];
        params.gradY = arguments[3 + extraArgs];
        extraArgs += 2;
    }

    // offset and offsets
    if (cracked.offset) {
        params.offset = arguments[2 + extraArgs];
        ++extraArgs;
    } else if (cracked.offsets) {
        params.offsets = arguments[2 + extraArgs];
        ++extraArgs;
    }

#ifndef GLSLANG_WEB
    // lod clamp
    if (cracked.lodClamp) {
        params.lodClamp = arguments[2 + extraArgs];
        ++extraArgs;
    }
    // sparse
    if (sparse) {
        params.texelOut = arguments[2 + extraArgs];
        ++extraArgs;
    }
    // gather component
    if (cracked.gather && ! sampler.shadow) {
        // default component is 0, if missing, otherwise an argument
        if (2 + extraArgs < (int)arguments.size()) {
            params.component = arguments[2 + extraArgs];
            ++extraArgs;
        } else
            params.component = builder.makeIntConstant(0);
    }
    spv::Id  resultStruct = spv::NoResult;
    if (imageFootprint) {
        //Following three extra arguments
        // int granularity, bool coarse, out gl_TextureFootprint2DNV footprint
        params.granularity = arguments[2 + extraArgs];
        params.coarse = arguments[3 + extraArgs];
        resultStruct = arguments[4 + extraArgs];
        extraArgs += 3;
    }
#endif
    // bias
    if (bias) {
        params.bias = arguments[2 + extraArgs];
        ++extraArgs;
    }

#ifndef GLSLANG_WEB
    if (imageFootprint) {
        builder.addExtension(spv::E_SPV_NV_shader_image_footprint);
        builder.addCapability(spv::CapabilityImageFootprintNV);


        //resultStructType(OpenGL type) contains 5 elements:
        //struct gl_TextureFootprint2DNV {
        //    uvec2 anchor;
        //    uvec2 offset;
        //    uvec2 mask;
        //    uint  lod;
        //    uint  granularity;
        //};
        //or
        //struct gl_TextureFootprint3DNV {
        //    uvec3 anchor;
        //    uvec3 offset;
        //    uvec2 mask;
        //    uint  lod;
        //    uint  granularity;
        //};
        spv::Id resultStructType = builder.getContainedTypeId(builder.getTypeId(resultStruct));
        assert(builder.isStructType(resultStructType));

        //resType (SPIR-V type) contains 6 elements:
        //Member 0 must be a Boolean type scalar(LOD), 
        //Member 1 must be a vector of integer type, whose Signedness operand is 0(anchor),  
        //Member 2 must be a vector of integer type, whose Signedness operand is 0(offset), 
        //Member 3 must be a vector of integer type, whose Signedness operand is 0(mask), 
        //Member 4 must be a scalar of integer type, whose Signedness operand is 0(lod),
        //Member 5 must be a scalar of integer type, whose Signedness operand is 0(granularity).
        std::vector<spv::Id> members;
        members.push_back(resultType());
        for (int i = 0; i < 5; i++) {
            members.push_back(builder.getContainedTypeId(resultStructType, i));
        }
        spv::Id resType = builder.makeStructType(members, "ResType");

        //call ImageFootprintNV
        spv::Id res = builder.createTextureCall(precision, resType, sparse, cracked.fetch, cracked.proj,
                                                cracked.gather, noImplicitLod, params, signExtensionMask());
        
        //copy resType (SPIR-V type) to resultStructType(OpenGL type)
        for (int i = 0; i < 5; i++) {
            builder.clearAccessChain();
            builder.setAccessChainLValue(resultStruct);

            //Accessing to a struct we created, no coherent flag is set
            spv::Builder::AccessChain::CoherentFlags flags;
            flags.clear();

            builder.accessChainPush(builder.makeIntConstant(i), flags, 0);
            builder.accessChainStore(builder.createCompositeExtract(res, builder.getContainedTypeId(resType, i+1),
                i+1), TranslateNonUniformDecoration(imageType.getQualifier()));
        }
        return builder.createCompositeExtract(res, resultType(), 0);
    }
#endif

    // projective component (might not to move)
    // GLSL: "The texture coordinates consumed from P, not including the last component of P,
    //       are divided by the last component of P."
    // SPIR-V:  "... (u [, v] [, w], q)... It may be a vector larger than needed, but all
    //          unused components will appear after all used components."
    if (cracked.proj) {
        int projSourceComp = builder.getNumComponents(params.coords) - 1;
        int projTargetComp;
        switch (sampler.dim) {
        case qglslang::Esd1D:   projTargetComp = 1;              break;
        case qglslang::Esd2D:   projTargetComp = 2;              break;
        case qglslang::EsdRect: projTargetComp = 2;              break;
        default:               projTargetComp = projSourceComp; break;
        }
        // copy the projective coordinate if we have to
        if (projTargetComp != projSourceComp) {
            spv::Id projComp = builder.createCompositeExtract(params.coords,
                                    builder.getScalarTypeId(builder.getTypeId(params.coords)), projSourceComp);
            params.coords = builder.createCompositeInsert(projComp, params.coords,
                                    builder.getTypeId(params.coords), projTargetComp);
        }
    }

#ifndef GLSLANG_WEB
    // nonprivate
    if (imageType.getQualifier().nonprivate) {
        params.nonprivate = true;
    }

    // volatile
    if (imageType.getQualifier().volatil) {
        params.volatil = true;
    }
#endif

    std::vector<spv::Id> result( 1, 
        builder.createTextureCall(precision, resultType(), sparse, cracked.fetch, cracked.proj, cracked.gather,
                                  noImplicitLod, params, signExtensionMask())
    );

    if (components != node->getType().getVectorSize())
        result[0] = builder.createConstructor(precision, result, convertGlslangToSpvType(node->getType()));

    return result[0];
}

spv::Id TGlslangToSpvTraverser::handleUserFunctionCall(const qglslang::TIntermAggregate* node)
{
    // Grab the function's pointer from the previously created function
    spv::Function* function = functionMap[node->getName().c_str()];
    if (! function)
        return 0;

    const qglslang::TIntermSequence& glslangArgs = node->getSequence();
    const qglslang::TQualifierList& qualifiers = node->getQualifierList();

    //  See comments in makeFunctions() for details about the semantics for parameter passing.
    //
    // These imply we need a four step process:
    // 1. Evaluate the arguments
    // 2. Allocate and make copies of in, out, and inout arguments
    // 3. Make the call
    // 4. Copy back the results

    // 1. Evaluate the arguments and their types
    std::vector<spv::Builder::AccessChain> lValues;
    std::vector<spv::Id> rValues;
    std::vector<const qglslang::TType*> argTypes;
    for (int a = 0; a < (int)glslangArgs.size(); ++a) {
        argTypes.push_back(&glslangArgs[a]->getAsTyped()->getType());
        // build l-value
        builder.clearAccessChain();
        glslangArgs[a]->traverse(this);
        // keep outputs and pass-by-originals as l-values, evaluate others as r-values
        if (originalParam(qualifiers[a], *argTypes[a], function->hasImplicitThis() && a == 0) ||
            writableParam(qualifiers[a])) {
            // save l-value
            lValues.push_back(builder.getAccessChain());
        } else {
            // process r-value
            rValues.push_back(accessChainLoad(*argTypes.back()));
        }
    }

    // 2. Allocate space for anything needing a copy, and if it's "in" or "inout"
    // copy the original into that space.
    //
    // Also, build up the list of actual arguments to pass in for the call
    int lValueCount = 0;
    int rValueCount = 0;
    std::vector<spv::Id> spvArgs;
    for (int a = 0; a < (int)glslangArgs.size(); ++a) {
        spv::Id arg;
        if (originalParam(qualifiers[a], *argTypes[a], function->hasImplicitThis() && a == 0)) {
            builder.setAccessChain(lValues[lValueCount]);
            arg = builder.accessChainGetLValue();
            ++lValueCount;
        } else if (writableParam(qualifiers[a])) {
            // need space to hold the copy
            arg = builder.createVariable(function->getParamPrecision(a), spv::StorageClassFunction,
                builder.getContainedTypeId(function->getParamType(a)), "param");
            if (qualifiers[a] == qglslang::EvqIn || qualifiers[a] == qglslang::EvqInOut) {
                // need to copy the input into output space
                builder.setAccessChain(lValues[lValueCount]);
                spv::Id copy = accessChainLoad(*argTypes[a]);
                builder.clearAccessChain();
                builder.setAccessChainLValue(arg);
                multiTypeStore(*argTypes[a], copy);
            }
            ++lValueCount;
        } else {
            // process r-value, which involves a copy for a type mismatch
            if (function->getParamType(a) != builder.getTypeId(rValues[rValueCount]) ||
                TranslatePrecisionDecoration(*argTypes[a]) != function->getParamPrecision(a))
            {
                spv::Id argCopy = builder.createVariable(function->getParamPrecision(a), spv::StorageClassFunction, function->getParamType(a), "arg");
                builder.clearAccessChain();
                builder.setAccessChainLValue(argCopy);
                multiTypeStore(*argTypes[a], rValues[rValueCount]);
                arg = builder.createLoad(argCopy, function->getParamPrecision(a));
            } else
                arg = rValues[rValueCount];
            ++rValueCount;
        }
        spvArgs.push_back(arg);
    }

    // 3. Make the call.
    spv::Id result = builder.createFunctionCall(function, spvArgs);
    builder.setPrecision(result, TranslatePrecisionDecoration(node->getType()));
    builder.addDecoration(result, TranslateNonUniformDecoration(node->getType().getQualifier()));

    // 4. Copy back out an "out" arguments.
    lValueCount = 0;
    for (int a = 0; a < (int)glslangArgs.size(); ++a) {
        if (originalParam(qualifiers[a], *argTypes[a], function->hasImplicitThis() && a == 0))
            ++lValueCount;
        else if (writableParam(qualifiers[a])) {
            if (qualifiers[a] == qglslang::EvqOut || qualifiers[a] == qglslang::EvqInOut) {
                spv::Id copy = builder.createLoad(spvArgs[a], spv::NoPrecision);
                builder.addDecoration(copy, TranslateNonUniformDecoration(argTypes[a]->getQualifier()));
                builder.setAccessChain(lValues[lValueCount]);
                multiTypeStore(*argTypes[a], copy);
            }
            ++lValueCount;
        }
    }

    return result;
}

// Translate AST operation to SPV operation, already having SPV-based operands/types.
spv::Id TGlslangToSpvTraverser::createBinaryOperation(qglslang::TOperator op, OpDecorations& decorations,
                                                      spv::Id typeId, spv::Id left, spv::Id right,
                                                      qglslang::TBasicType typeProxy, bool reduceComparison)
{
    bool isUnsigned = isTypeUnsignedInt(typeProxy);
    bool isFloat = isTypeFloat(typeProxy);
    bool isBool = typeProxy == qglslang::EbtBool;

    spv::Op binOp = spv::OpNop;
    bool needMatchingVectors = true;  // for non-matrix ops, would a scalar need to smear to match a vector?
    bool comparison = false;

    switch (op) {
    case qglslang::EOpAdd:
    case qglslang::EOpAddAssign:
        if (isFloat)
            binOp = spv::OpFAdd;
        else
            binOp = spv::OpIAdd;
        break;
    case qglslang::EOpSub:
    case qglslang::EOpSubAssign:
        if (isFloat)
            binOp = spv::OpFSub;
        else
            binOp = spv::OpISub;
        break;
    case qglslang::EOpMul:
    case qglslang::EOpMulAssign:
        if (isFloat)
            binOp = spv::OpFMul;
        else
            binOp = spv::OpIMul;
        break;
    case qglslang::EOpVectorTimesScalar:
    case qglslang::EOpVectorTimesScalarAssign:
        if (isFloat && (builder.isVector(left) || builder.isVector(right))) {
            if (builder.isVector(right))
                std::swap(left, right);
            assert(builder.isScalar(right));
            needMatchingVectors = false;
            binOp = spv::OpVectorTimesScalar;
        } else if (isFloat)
            binOp = spv::OpFMul;
          else
            binOp = spv::OpIMul;
        break;
    case qglslang::EOpVectorTimesMatrix:
    case qglslang::EOpVectorTimesMatrixAssign:
        binOp = spv::OpVectorTimesMatrix;
        break;
    case qglslang::EOpMatrixTimesVector:
        binOp = spv::OpMatrixTimesVector;
        break;
    case qglslang::EOpMatrixTimesScalar:
    case qglslang::EOpMatrixTimesScalarAssign:
        binOp = spv::OpMatrixTimesScalar;
        break;
    case qglslang::EOpMatrixTimesMatrix:
    case qglslang::EOpMatrixTimesMatrixAssign:
        binOp = spv::OpMatrixTimesMatrix;
        break;
    case qglslang::EOpOuterProduct:
        binOp = spv::OpOuterProduct;
        needMatchingVectors = false;
        break;

    case qglslang::EOpDiv:
    case qglslang::EOpDivAssign:
        if (isFloat)
            binOp = spv::OpFDiv;
        else if (isUnsigned)
            binOp = spv::OpUDiv;
        else
            binOp = spv::OpSDiv;
        break;
    case qglslang::EOpMod:
    case qglslang::EOpModAssign:
        if (isFloat)
            binOp = spv::OpFMod;
        else if (isUnsigned)
            binOp = spv::OpUMod;
        else
            binOp = spv::OpSMod;
        break;
    case qglslang::EOpRightShift:
    case qglslang::EOpRightShiftAssign:
        if (isUnsigned)
            binOp = spv::OpShiftRightLogical;
        else
            binOp = spv::OpShiftRightArithmetic;
        break;
    case qglslang::EOpLeftShift:
    case qglslang::EOpLeftShiftAssign:
        binOp = spv::OpShiftLeftLogical;
        break;
    case qglslang::EOpAnd:
    case qglslang::EOpAndAssign:
        binOp = spv::OpBitwiseAnd;
        break;
    case qglslang::EOpLogicalAnd:
        needMatchingVectors = false;
        binOp = spv::OpLogicalAnd;
        break;
    case qglslang::EOpInclusiveOr:
    case qglslang::EOpInclusiveOrAssign:
        binOp = spv::OpBitwiseOr;
        break;
    case qglslang::EOpLogicalOr:
        needMatchingVectors = false;
        binOp = spv::OpLogicalOr;
        break;
    case qglslang::EOpExclusiveOr:
    case qglslang::EOpExclusiveOrAssign:
        binOp = spv::OpBitwiseXor;
        break;
    case qglslang::EOpLogicalXor:
        needMatchingVectors = false;
        binOp = spv::OpLogicalNotEqual;
        break;

    case qglslang::EOpAbsDifference:
        binOp = isUnsigned ? spv::OpAbsUSubINTEL : spv::OpAbsISubINTEL;
        break;

    case qglslang::EOpAddSaturate:
        binOp = isUnsigned ? spv::OpUAddSatINTEL : spv::OpIAddSatINTEL;
        break;

    case qglslang::EOpSubSaturate:
        binOp = isUnsigned ? spv::OpUSubSatINTEL : spv::OpISubSatINTEL;
        break;

    case qglslang::EOpAverage:
        binOp = isUnsigned ? spv::OpUAverageINTEL : spv::OpIAverageINTEL;
        break;

    case qglslang::EOpAverageRounded:
        binOp = isUnsigned ? spv::OpUAverageRoundedINTEL : spv::OpIAverageRoundedINTEL;
        break;

    case qglslang::EOpMul32x16:
        binOp = isUnsigned ? spv::OpUMul32x16INTEL : spv::OpIMul32x16INTEL;
        break;

    case qglslang::EOpLessThan:
    case qglslang::EOpGreaterThan:
    case qglslang::EOpLessThanEqual:
    case qglslang::EOpGreaterThanEqual:
    case qglslang::EOpEqual:
    case qglslang::EOpNotEqual:
    case qglslang::EOpVectorEqual:
    case qglslang::EOpVectorNotEqual:
        comparison = true;
        break;
    default:
        break;
    }

    // handle mapped binary operations (should be non-comparison)
    if (binOp != spv::OpNop) {
        assert(comparison == false);
        if (builder.isMatrix(left) || builder.isMatrix(right) ||
            builder.isCooperativeMatrix(left) || builder.isCooperativeMatrix(right))
            return createBinaryMatrixOperation(binOp, decorations, typeId, left, right);

        // No matrix involved; make both operands be the same number of components, if needed
        if (needMatchingVectors)
            builder.promoteScalar(decorations.precision, left, right);

        spv::Id result = builder.createBinOp(binOp, typeId, left, right);
        decorations.addNoContraction(builder, result);
        decorations.addNonUniform(builder, result);
        return builder.setPrecision(result, decorations.precision);
    }

    if (! comparison)
        return 0;

    // Handle comparison instructions

    if (reduceComparison && (op == qglslang::EOpEqual || op == qglslang::EOpNotEqual)
                         && (builder.isVector(left) || builder.isMatrix(left) || builder.isAggregate(left))) {
        spv::Id result = builder.createCompositeCompare(decorations.precision, left, right, op == qglslang::EOpEqual);
        decorations.addNonUniform(builder, result);
        return result;
    }

    switch (op) {
    case qglslang::EOpLessThan:
        if (isFloat)
            binOp = spv::OpFOrdLessThan;
        else if (isUnsigned)
            binOp = spv::OpULessThan;
        else
            binOp = spv::OpSLessThan;
        break;
    case qglslang::EOpGreaterThan:
        if (isFloat)
            binOp = spv::OpFOrdGreaterThan;
        else if (isUnsigned)
            binOp = spv::OpUGreaterThan;
        else
            binOp = spv::OpSGreaterThan;
        break;
    case qglslang::EOpLessThanEqual:
        if (isFloat)
            binOp = spv::OpFOrdLessThanEqual;
        else if (isUnsigned)
            binOp = spv::OpULessThanEqual;
        else
            binOp = spv::OpSLessThanEqual;
        break;
    case qglslang::EOpGreaterThanEqual:
        if (isFloat)
            binOp = spv::OpFOrdGreaterThanEqual;
        else if (isUnsigned)
            binOp = spv::OpUGreaterThanEqual;
        else
            binOp = spv::OpSGreaterThanEqual;
        break;
    case qglslang::EOpEqual:
    case qglslang::EOpVectorEqual:
        if (isFloat)
            binOp = spv::OpFOrdEqual;
        else if (isBool)
            binOp = spv::OpLogicalEqual;
        else
            binOp = spv::OpIEqual;
        break;
    case qglslang::EOpNotEqual:
    case qglslang::EOpVectorNotEqual:
        if (isFloat)
            binOp = spv::OpFUnordNotEqual;
        else if (isBool)
            binOp = spv::OpLogicalNotEqual;
        else
            binOp = spv::OpINotEqual;
        break;
    default:
        break;
    }

    if (binOp != spv::OpNop) {
        spv::Id result = builder.createBinOp(binOp, typeId, left, right);
        decorations.addNoContraction(builder, result);
        decorations.addNonUniform(builder, result);
        return builder.setPrecision(result, decorations.precision);
    }

    return 0;
}

//
// Translate AST matrix operation to SPV operation, already having SPV-based operands/types.
// These can be any of:
//
//   matrix * scalar
//   scalar * matrix
//   matrix * matrix     linear algebraic
//   matrix * vector
//   vector * matrix
//   matrix * matrix     componentwise
//   matrix op matrix    op in {+, -, /}
//   matrix op scalar    op in {+, -, /}
//   scalar op matrix    op in {+, -, /}
//
spv::Id TGlslangToSpvTraverser::createBinaryMatrixOperation(spv::Op op, OpDecorations& decorations, spv::Id typeId,
                                                            spv::Id left, spv::Id right)
{
    bool firstClass = true;

    // First, handle first-class matrix operations (* and matrix/scalar)
    switch (op) {
    case spv::OpFDiv:
        if (builder.isMatrix(left) && builder.isScalar(right)) {
            // turn matrix / scalar into a multiply...
            spv::Id resultType = builder.getTypeId(right);
            right = builder.createBinOp(spv::OpFDiv, resultType, builder.makeFpConstant(resultType, 1.0), right);
            op = spv::OpMatrixTimesScalar;
        } else
            firstClass = false;
        break;
    case spv::OpMatrixTimesScalar:
        if (builder.isMatrix(right) || builder.isCooperativeMatrix(right))
            std::swap(left, right);
        assert(builder.isScalar(right));
        break;
    case spv::OpVectorTimesMatrix:
        assert(builder.isVector(left));
        assert(builder.isMatrix(right));
        break;
    case spv::OpMatrixTimesVector:
        assert(builder.isMatrix(left));
        assert(builder.isVector(right));
        break;
    case spv::OpMatrixTimesMatrix:
        assert(builder.isMatrix(left));
        assert(builder.isMatrix(right));
        break;
    default:
        firstClass = false;
        break;
    }

    if (builder.isCooperativeMatrix(left) || builder.isCooperativeMatrix(right))
        firstClass = true;

    if (firstClass) {
        spv::Id result = builder.createBinOp(op, typeId, left, right);
        decorations.addNoContraction(builder, result);
        decorations.addNonUniform(builder, result);
        return builder.setPrecision(result, decorations.precision);
    }

    // Handle component-wise +, -, *, %, and / for all combinations of type.
    // The result type of all of them is the same type as the (a) matrix operand.
    // The algorithm is to:
    //   - break the matrix(es) into vectors
    //   - smear any scalar to a vector
    //   - do vector operations
    //   - make a matrix out the vector results
    switch (op) {
    case spv::OpFAdd:
    case spv::OpFSub:
    case spv::OpFDiv:
    case spv::OpFMod:
    case spv::OpFMul:
    {
        // one time set up...
        bool  leftMat = builder.isMatrix(left);
        bool rightMat = builder.isMatrix(right);
        unsigned int numCols = leftMat ? builder.getNumColumns(left) : builder.getNumColumns(right);
        int numRows = leftMat ? builder.getNumRows(left) : builder.getNumRows(right);
        spv::Id scalarType = builder.getScalarTypeId(typeId);
        spv::Id vecType = builder.makeVectorType(scalarType, numRows);
        std::vector<spv::Id> results;
        spv::Id smearVec = spv::NoResult;
        if (builder.isScalar(left))
            smearVec = builder.smearScalar(decorations.precision, left, vecType);
        else if (builder.isScalar(right))
            smearVec = builder.smearScalar(decorations.precision, right, vecType);

        // do each vector op
        for (unsigned int c = 0; c < numCols; ++c) {
            std::vector<unsigned int> indexes;
            indexes.push_back(c);
            spv::Id  leftVec =  leftMat ? builder.createCompositeExtract( left, vecType, indexes) : smearVec;
            spv::Id rightVec = rightMat ? builder.createCompositeExtract(right, vecType, indexes) : smearVec;
            spv::Id result = builder.createBinOp(op, vecType, leftVec, rightVec);
            decorations.addNoContraction(builder, result);
            decorations.addNonUniform(builder, result);
            results.push_back(builder.setPrecision(result, decorations.precision));
        }

        // put the pieces together
        spv::Id result = builder.setPrecision(builder.createCompositeConstruct(typeId, results), decorations.precision);
        decorations.addNonUniform(builder, result);
        return result;
    }
    default:
        assert(0);
        return spv::NoResult;
    }
}

spv::Id TGlslangToSpvTraverser::createUnaryOperation(qglslang::TOperator op, OpDecorations& decorations, spv::Id typeId,
    spv::Id operand, qglslang::TBasicType typeProxy, const spv::Builder::AccessChain::CoherentFlags &lvalueCoherentFlags)
{
    spv::Op unaryOp = spv::OpNop;
    int extBuiltins = -1;
    int libCall = -1;
    bool isUnsigned = isTypeUnsignedInt(typeProxy);
    bool isFloat = isTypeFloat(typeProxy);

    switch (op) {
    case qglslang::EOpNegative:
        if (isFloat) {
            unaryOp = spv::OpFNegate;
            if (builder.isMatrixType(typeId))
                return createUnaryMatrixOperation(unaryOp, decorations, typeId, operand, typeProxy);
        } else
            unaryOp = spv::OpSNegate;
        break;

    case qglslang::EOpLogicalNot:
    case qglslang::EOpVectorLogicalNot:
        unaryOp = spv::OpLogicalNot;
        break;
    case qglslang::EOpBitwiseNot:
        unaryOp = spv::OpNot;
        break;

    case qglslang::EOpDeterminant:
        libCall = spv::GLSLstd450Determinant;
        break;
    case qglslang::EOpMatrixInverse:
        libCall = spv::GLSLstd450MatrixInverse;
        break;
    case qglslang::EOpTranspose:
        unaryOp = spv::OpTranspose;
        break;

    case qglslang::EOpRadians:
        libCall = spv::GLSLstd450Radians;
        break;
    case qglslang::EOpDegrees:
        libCall = spv::GLSLstd450Degrees;
        break;
    case qglslang::EOpSin:
        libCall = spv::GLSLstd450Sin;
        break;
    case qglslang::EOpCos:
        libCall = spv::GLSLstd450Cos;
        break;
    case qglslang::EOpTan:
        libCall = spv::GLSLstd450Tan;
        break;
    case qglslang::EOpAcos:
        libCall = spv::GLSLstd450Acos;
        break;
    case qglslang::EOpAsin:
        libCall = spv::GLSLstd450Asin;
        break;
    case qglslang::EOpAtan:
        libCall = spv::GLSLstd450Atan;
        break;

    case qglslang::EOpAcosh:
        libCall = spv::GLSLstd450Acosh;
        break;
    case qglslang::EOpAsinh:
        libCall = spv::GLSLstd450Asinh;
        break;
    case qglslang::EOpAtanh:
        libCall = spv::GLSLstd450Atanh;
        break;
    case qglslang::EOpTanh:
        libCall = spv::GLSLstd450Tanh;
        break;
    case qglslang::EOpCosh:
        libCall = spv::GLSLstd450Cosh;
        break;
    case qglslang::EOpSinh:
        libCall = spv::GLSLstd450Sinh;
        break;

    case qglslang::EOpLength:
        libCall = spv::GLSLstd450Length;
        break;
    case qglslang::EOpNormalize:
        libCall = spv::GLSLstd450Normalize;
        break;

    case qglslang::EOpExp:
        libCall = spv::GLSLstd450Exp;
        break;
    case qglslang::EOpLog:
        libCall = spv::GLSLstd450Log;
        break;
    case qglslang::EOpExp2:
        libCall = spv::GLSLstd450Exp2;
        break;
    case qglslang::EOpLog2:
        libCall = spv::GLSLstd450Log2;
        break;
    case qglslang::EOpSqrt:
        libCall = spv::GLSLstd450Sqrt;
        break;
    case qglslang::EOpInverseSqrt:
        libCall = spv::GLSLstd450InverseSqrt;
        break;

    case qglslang::EOpFloor:
        libCall = spv::GLSLstd450Floor;
        break;
    case qglslang::EOpTrunc:
        libCall = spv::GLSLstd450Trunc;
        break;
    case qglslang::EOpRound:
        libCall = spv::GLSLstd450Round;
        break;
    case qglslang::EOpRoundEven:
        libCall = spv::GLSLstd450RoundEven;
        break;
    case qglslang::EOpCeil:
        libCall = spv::GLSLstd450Ceil;
        break;
    case qglslang::EOpFract:
        libCall = spv::GLSLstd450Fract;
        break;

    case qglslang::EOpIsNan:
        unaryOp = spv::OpIsNan;
        break;
    case qglslang::EOpIsInf:
        unaryOp = spv::OpIsInf;
        break;
    case qglslang::EOpIsFinite:
        unaryOp = spv::OpIsFinite;
        break;

    case qglslang::EOpFloatBitsToInt:
    case qglslang::EOpFloatBitsToUint:
    case qglslang::EOpIntBitsToFloat:
    case qglslang::EOpUintBitsToFloat:
    case qglslang::EOpDoubleBitsToInt64:
    case qglslang::EOpDoubleBitsToUint64:
    case qglslang::EOpInt64BitsToDouble:
    case qglslang::EOpUint64BitsToDouble:
    case qglslang::EOpFloat16BitsToInt16:
    case qglslang::EOpFloat16BitsToUint16:
    case qglslang::EOpInt16BitsToFloat16:
    case qglslang::EOpUint16BitsToFloat16:
        unaryOp = spv::OpBitcast;
        break;

    case qglslang::EOpPackSnorm2x16:
        libCall = spv::GLSLstd450PackSnorm2x16;
        break;
    case qglslang::EOpUnpackSnorm2x16:
        libCall = spv::GLSLstd450UnpackSnorm2x16;
        break;
    case qglslang::EOpPackUnorm2x16:
        libCall = spv::GLSLstd450PackUnorm2x16;
        break;
    case qglslang::EOpUnpackUnorm2x16:
        libCall = spv::GLSLstd450UnpackUnorm2x16;
        break;
    case qglslang::EOpPackHalf2x16:
        libCall = spv::GLSLstd450PackHalf2x16;
        break;
    case qglslang::EOpUnpackHalf2x16:
        libCall = spv::GLSLstd450UnpackHalf2x16;
        break;
#ifndef GLSLANG_WEB
    case qglslang::EOpPackSnorm4x8:
        libCall = spv::GLSLstd450PackSnorm4x8;
        break;
    case qglslang::EOpUnpackSnorm4x8:
        libCall = spv::GLSLstd450UnpackSnorm4x8;
        break;
    case qglslang::EOpPackUnorm4x8:
        libCall = spv::GLSLstd450PackUnorm4x8;
        break;
    case qglslang::EOpUnpackUnorm4x8:
        libCall = spv::GLSLstd450UnpackUnorm4x8;
        break;
    case qglslang::EOpPackDouble2x32:
        libCall = spv::GLSLstd450PackDouble2x32;
        break;
    case qglslang::EOpUnpackDouble2x32:
        libCall = spv::GLSLstd450UnpackDouble2x32;
        break;
#endif

    case qglslang::EOpPackInt2x32:
    case qglslang::EOpUnpackInt2x32:
    case qglslang::EOpPackUint2x32:
    case qglslang::EOpUnpackUint2x32:
    case qglslang::EOpPack16:
    case qglslang::EOpPack32:
    case qglslang::EOpPack64:
    case qglslang::EOpUnpack32:
    case qglslang::EOpUnpack16:
    case qglslang::EOpUnpack8:
    case qglslang::EOpPackInt2x16:
    case qglslang::EOpUnpackInt2x16:
    case qglslang::EOpPackUint2x16:
    case qglslang::EOpUnpackUint2x16:
    case qglslang::EOpPackInt4x16:
    case qglslang::EOpUnpackInt4x16:
    case qglslang::EOpPackUint4x16:
    case qglslang::EOpUnpackUint4x16:
    case qglslang::EOpPackFloat2x16:
    case qglslang::EOpUnpackFloat2x16:
        unaryOp = spv::OpBitcast;
        break;

    case qglslang::EOpDPdx:
        unaryOp = spv::OpDPdx;
        break;
    case qglslang::EOpDPdy:
        unaryOp = spv::OpDPdy;
        break;
    case qglslang::EOpFwidth:
        unaryOp = spv::OpFwidth;
        break;

    case qglslang::EOpAny:
        unaryOp = spv::OpAny;
        break;
    case qglslang::EOpAll:
        unaryOp = spv::OpAll;
        break;

    case qglslang::EOpAbs:
        if (isFloat)
            libCall = spv::GLSLstd450FAbs;
        else
            libCall = spv::GLSLstd450SAbs;
        break;
    case qglslang::EOpSign:
        if (isFloat)
            libCall = spv::GLSLstd450FSign;
        else
            libCall = spv::GLSLstd450SSign;
        break;

#ifndef GLSLANG_WEB
    case qglslang::EOpDPdxFine:
        unaryOp = spv::OpDPdxFine;
        break;
    case qglslang::EOpDPdyFine:
        unaryOp = spv::OpDPdyFine;
        break;
    case qglslang::EOpFwidthFine:
        unaryOp = spv::OpFwidthFine;
        break;
    case qglslang::EOpDPdxCoarse:
        unaryOp = spv::OpDPdxCoarse;
        break;
    case qglslang::EOpDPdyCoarse:
        unaryOp = spv::OpDPdyCoarse;
        break;
    case qglslang::EOpFwidthCoarse:
        unaryOp = spv::OpFwidthCoarse;
        break;
    case qglslang::EOpRayQueryProceed:
        unaryOp = spv::OpRayQueryProceedKHR;
        break;
    case qglslang::EOpRayQueryGetRayTMin:
        unaryOp = spv::OpRayQueryGetRayTMinKHR;
        break;
    case qglslang::EOpRayQueryGetRayFlags:
        unaryOp = spv::OpRayQueryGetRayFlagsKHR;
        break;
    case qglslang::EOpRayQueryGetWorldRayOrigin:
        unaryOp = spv::OpRayQueryGetWorldRayOriginKHR;
        break;
    case qglslang::EOpRayQueryGetWorldRayDirection:
        unaryOp = spv::OpRayQueryGetWorldRayDirectionKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionCandidateAABBOpaque:
        unaryOp = spv::OpRayQueryGetIntersectionCandidateAABBOpaqueKHR;
        break;
    case qglslang::EOpInterpolateAtCentroid:
        if (typeProxy == qglslang::EbtFloat16)
            builder.addExtension(spv::E_SPV_AMD_gpu_shader_half_float);
        libCall = spv::GLSLstd450InterpolateAtCentroid;
        break;
    case qglslang::EOpAtomicCounterIncrement:
    case qglslang::EOpAtomicCounterDecrement:
    case qglslang::EOpAtomicCounter:
    {
        // Handle all of the atomics in one place, in createAtomicOperation()
        std::vector<spv::Id> operands;
        operands.push_back(operand);
        return createAtomicOperation(op, decorations.precision, typeId, operands, typeProxy, lvalueCoherentFlags);
    }

    case qglslang::EOpBitFieldReverse:
        unaryOp = spv::OpBitReverse;
        break;
    case qglslang::EOpBitCount:
        unaryOp = spv::OpBitCount;
        break;
    case qglslang::EOpFindLSB:
        libCall = spv::GLSLstd450FindILsb;
        break;
    case qglslang::EOpFindMSB:
        if (isUnsigned)
            libCall = spv::GLSLstd450FindUMsb;
        else
            libCall = spv::GLSLstd450FindSMsb;
        break;

    case qglslang::EOpCountLeadingZeros:
        builder.addCapability(spv::CapabilityIntegerFunctions2INTEL);
        builder.addExtension("SPV_INTEL_shader_integer_functions2");
        unaryOp = spv::OpUCountLeadingZerosINTEL;
        break;

    case qglslang::EOpCountTrailingZeros:
        builder.addCapability(spv::CapabilityIntegerFunctions2INTEL);
        builder.addExtension("SPV_INTEL_shader_integer_functions2");
        unaryOp = spv::OpUCountTrailingZerosINTEL;
        break;

    case qglslang::EOpBallot:
    case qglslang::EOpReadFirstInvocation:
    case qglslang::EOpAnyInvocation:
    case qglslang::EOpAllInvocations:
    case qglslang::EOpAllInvocationsEqual:
    case qglslang::EOpMinInvocations:
    case qglslang::EOpMaxInvocations:
    case qglslang::EOpAddInvocations:
    case qglslang::EOpMinInvocationsNonUniform:
    case qglslang::EOpMaxInvocationsNonUniform:
    case qglslang::EOpAddInvocationsNonUniform:
    case qglslang::EOpMinInvocationsInclusiveScan:
    case qglslang::EOpMaxInvocationsInclusiveScan:
    case qglslang::EOpAddInvocationsInclusiveScan:
    case qglslang::EOpMinInvocationsInclusiveScanNonUniform:
    case qglslang::EOpMaxInvocationsInclusiveScanNonUniform:
    case qglslang::EOpAddInvocationsInclusiveScanNonUniform:
    case qglslang::EOpMinInvocationsExclusiveScan:
    case qglslang::EOpMaxInvocationsExclusiveScan:
    case qglslang::EOpAddInvocationsExclusiveScan:
    case qglslang::EOpMinInvocationsExclusiveScanNonUniform:
    case qglslang::EOpMaxInvocationsExclusiveScanNonUniform:
    case qglslang::EOpAddInvocationsExclusiveScanNonUniform:
    {
        std::vector<spv::Id> operands;
        operands.push_back(operand);
        return createInvocationsOperation(op, typeId, operands, typeProxy);
    }
    case qglslang::EOpSubgroupAll:
    case qglslang::EOpSubgroupAny:
    case qglslang::EOpSubgroupAllEqual:
    case qglslang::EOpSubgroupBroadcastFirst:
    case qglslang::EOpSubgroupBallot:
    case qglslang::EOpSubgroupInverseBallot:
    case qglslang::EOpSubgroupBallotBitCount:
    case qglslang::EOpSubgroupBallotInclusiveBitCount:
    case qglslang::EOpSubgroupBallotExclusiveBitCount:
    case qglslang::EOpSubgroupBallotFindLSB:
    case qglslang::EOpSubgroupBallotFindMSB:
    case qglslang::EOpSubgroupAdd:
    case qglslang::EOpSubgroupMul:
    case qglslang::EOpSubgroupMin:
    case qglslang::EOpSubgroupMax:
    case qglslang::EOpSubgroupAnd:
    case qglslang::EOpSubgroupOr:
    case qglslang::EOpSubgroupXor:
    case qglslang::EOpSubgroupInclusiveAdd:
    case qglslang::EOpSubgroupInclusiveMul:
    case qglslang::EOpSubgroupInclusiveMin:
    case qglslang::EOpSubgroupInclusiveMax:
    case qglslang::EOpSubgroupInclusiveAnd:
    case qglslang::EOpSubgroupInclusiveOr:
    case qglslang::EOpSubgroupInclusiveXor:
    case qglslang::EOpSubgroupExclusiveAdd:
    case qglslang::EOpSubgroupExclusiveMul:
    case qglslang::EOpSubgroupExclusiveMin:
    case qglslang::EOpSubgroupExclusiveMax:
    case qglslang::EOpSubgroupExclusiveAnd:
    case qglslang::EOpSubgroupExclusiveOr:
    case qglslang::EOpSubgroupExclusiveXor:
    case qglslang::EOpSubgroupQuadSwapHorizontal:
    case qglslang::EOpSubgroupQuadSwapVertical:
    case qglslang::EOpSubgroupQuadSwapDiagonal: {
        std::vector<spv::Id> operands;
        operands.push_back(operand);
        return createSubgroupOperation(op, typeId, operands, typeProxy);
    }
    case qglslang::EOpMbcnt:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_shader_ballot);
        libCall = spv::MbcntAMD;
        break;

    case qglslang::EOpCubeFaceIndex:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_gcn_shader);
        libCall = spv::CubeFaceIndexAMD;
        break;

    case qglslang::EOpCubeFaceCoord:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_gcn_shader);
        libCall = spv::CubeFaceCoordAMD;
        break;
    case qglslang::EOpSubgroupPartition:
        unaryOp = spv::OpGroupNonUniformPartitionNV;
        break;
    case qglslang::EOpConstructReference:
        unaryOp = spv::OpBitcast;
        break;

    case qglslang::EOpConvUint64ToAccStruct:
    case qglslang::EOpConvUvec2ToAccStruct:
        unaryOp = spv::OpConvertUToAccelerationStructureKHR;
        break;
#endif

    case qglslang::EOpCopyObject:
        unaryOp = spv::OpCopyObject;
        break;

    default:
        return 0;
    }

    spv::Id id;
    if (libCall >= 0) {
        std::vector<spv::Id> args;
        args.push_back(operand);
        id = builder.createBuiltinCall(typeId, extBuiltins >= 0 ? extBuiltins : stdBuiltins, libCall, args);
    } else {
        id = builder.createUnaryOp(unaryOp, typeId, operand);
    }

    decorations.addNoContraction(builder, id);
    decorations.addNonUniform(builder, id);
    return builder.setPrecision(id, decorations.precision);
}

// Create a unary operation on a matrix
spv::Id TGlslangToSpvTraverser::createUnaryMatrixOperation(spv::Op op, OpDecorations& decorations, spv::Id typeId,
                                                           spv::Id operand, qglslang::TBasicType /* typeProxy */)
{
    // Handle unary operations vector by vector.
    // The result type is the same type as the original type.
    // The algorithm is to:
    //   - break the matrix into vectors
    //   - apply the operation to each vector
    //   - make a matrix out the vector results

    // get the types sorted out
    int numCols = builder.getNumColumns(operand);
    int numRows = builder.getNumRows(operand);
    spv::Id srcVecType  = builder.makeVectorType(builder.getScalarTypeId(builder.getTypeId(operand)), numRows);
    spv::Id destVecType = builder.makeVectorType(builder.getScalarTypeId(typeId), numRows);
    std::vector<spv::Id> results;

    // do each vector op
    for (int c = 0; c < numCols; ++c) {
        std::vector<unsigned int> indexes;
        indexes.push_back(c);
        spv::Id srcVec  = builder.createCompositeExtract(operand, srcVecType, indexes);
        spv::Id destVec = builder.createUnaryOp(op, destVecType, srcVec);
        decorations.addNoContraction(builder, destVec);
        decorations.addNonUniform(builder, destVec);
        results.push_back(builder.setPrecision(destVec, decorations.precision));
    }

    // put the pieces together
    spv::Id result = builder.setPrecision(builder.createCompositeConstruct(typeId, results), decorations.precision);
    decorations.addNonUniform(builder, result);
    return result;
}

// For converting integers where both the bitwidth and the signedness could
// change, but only do the width change here. The caller is still responsible
// for the signedness conversion.
spv::Id TGlslangToSpvTraverser::createIntWidthConversion(qglslang::TOperator op, spv::Id operand, int vectorSize)
{
    // Get the result type width, based on the type to convert to.
    int width = 32;
    switch(op) {
    case qglslang::EOpConvInt16ToUint8:
    case qglslang::EOpConvIntToUint8:
    case qglslang::EOpConvInt64ToUint8:
    case qglslang::EOpConvUint16ToInt8:
    case qglslang::EOpConvUintToInt8:
    case qglslang::EOpConvUint64ToInt8:
        width = 8;
        break;
    case qglslang::EOpConvInt8ToUint16:
    case qglslang::EOpConvIntToUint16:
    case qglslang::EOpConvInt64ToUint16:
    case qglslang::EOpConvUint8ToInt16:
    case qglslang::EOpConvUintToInt16:
    case qglslang::EOpConvUint64ToInt16:
        width = 16;
        break;
    case qglslang::EOpConvInt8ToUint:
    case qglslang::EOpConvInt16ToUint:
    case qglslang::EOpConvInt64ToUint:
    case qglslang::EOpConvUint8ToInt:
    case qglslang::EOpConvUint16ToInt:
    case qglslang::EOpConvUint64ToInt:
        width = 32;
        break;
    case qglslang::EOpConvInt8ToUint64:
    case qglslang::EOpConvInt16ToUint64:
    case qglslang::EOpConvIntToUint64:
    case qglslang::EOpConvUint8ToInt64:
    case qglslang::EOpConvUint16ToInt64:
    case qglslang::EOpConvUintToInt64:
        width = 64;
        break;

    default:
        assert(false && "Default missing");
        break;
    }

    // Get the conversion operation and result type,
    // based on the target width, but the source type.
    spv::Id type = spv::NoType;
    spv::Op convOp = spv::OpNop;
    switch(op) {
    case qglslang::EOpConvInt8ToUint16:
    case qglslang::EOpConvInt8ToUint:
    case qglslang::EOpConvInt8ToUint64:
    case qglslang::EOpConvInt16ToUint8:
    case qglslang::EOpConvInt16ToUint:
    case qglslang::EOpConvInt16ToUint64:
    case qglslang::EOpConvIntToUint8:
    case qglslang::EOpConvIntToUint16:
    case qglslang::EOpConvIntToUint64:
    case qglslang::EOpConvInt64ToUint8:
    case qglslang::EOpConvInt64ToUint16:
    case qglslang::EOpConvInt64ToUint:
        convOp = spv::OpSConvert;
        type = builder.makeIntType(width);
        break;
    default:
        convOp = spv::OpUConvert;
        type = builder.makeUintType(width);
        break;
    }

    if (vectorSize > 0)
        type = builder.makeVectorType(type, vectorSize);

    return builder.createUnaryOp(convOp, type, operand);
}

spv::Id TGlslangToSpvTraverser::createConversion(qglslang::TOperator op, OpDecorations& decorations, spv::Id destType,
                                                 spv::Id operand, qglslang::TBasicType typeProxy)
{
    spv::Op convOp = spv::OpNop;
    spv::Id zero = 0;
    spv::Id one = 0;

    int vectorSize = builder.isVectorType(destType) ? builder.getNumTypeComponents(destType) : 0;

    switch (op) {
    case qglslang::EOpConvIntToBool:
    case qglslang::EOpConvUintToBool:
        zero = builder.makeUintConstant(0);
        zero = makeSmearedConstant(zero, vectorSize);
        return builder.createBinOp(spv::OpINotEqual, destType, operand, zero);
    case qglslang::EOpConvFloatToBool:
        zero = builder.makeFloatConstant(0.0F);
        zero = makeSmearedConstant(zero, vectorSize);
        return builder.createBinOp(spv::OpFUnordNotEqual, destType, operand, zero);
    case qglslang::EOpConvBoolToFloat:
        convOp = spv::OpSelect;
        zero = builder.makeFloatConstant(0.0F);
        one  = builder.makeFloatConstant(1.0F);
        break;

    case qglslang::EOpConvBoolToInt:
    case qglslang::EOpConvBoolToInt64:
#ifndef GLSLANG_WEB
        if (op == qglslang::EOpConvBoolToInt64) {
            zero = builder.makeInt64Constant(0);
            one = builder.makeInt64Constant(1);
        } else
#endif
        {
            zero = builder.makeIntConstant(0);
            one = builder.makeIntConstant(1);
        }

        convOp = spv::OpSelect;
        break;

    case qglslang::EOpConvBoolToUint:
    case qglslang::EOpConvBoolToUint64:
#ifndef GLSLANG_WEB
        if (op == qglslang::EOpConvBoolToUint64) {
            zero = builder.makeUint64Constant(0);
            one = builder.makeUint64Constant(1);
        } else
#endif
        {
            zero = builder.makeUintConstant(0);
            one = builder.makeUintConstant(1);
        }

        convOp = spv::OpSelect;
        break;

    case qglslang::EOpConvInt8ToFloat16:
    case qglslang::EOpConvInt8ToFloat:
    case qglslang::EOpConvInt8ToDouble:
    case qglslang::EOpConvInt16ToFloat16:
    case qglslang::EOpConvInt16ToFloat:
    case qglslang::EOpConvInt16ToDouble:
    case qglslang::EOpConvIntToFloat16:
    case qglslang::EOpConvIntToFloat:
    case qglslang::EOpConvIntToDouble:
    case qglslang::EOpConvInt64ToFloat:
    case qglslang::EOpConvInt64ToDouble:
    case qglslang::EOpConvInt64ToFloat16:
        convOp = spv::OpConvertSToF;
        break;

    case qglslang::EOpConvUint8ToFloat16:
    case qglslang::EOpConvUint8ToFloat:
    case qglslang::EOpConvUint8ToDouble:
    case qglslang::EOpConvUint16ToFloat16:
    case qglslang::EOpConvUint16ToFloat:
    case qglslang::EOpConvUint16ToDouble:
    case qglslang::EOpConvUintToFloat16:
    case qglslang::EOpConvUintToFloat:
    case qglslang::EOpConvUintToDouble:
    case qglslang::EOpConvUint64ToFloat:
    case qglslang::EOpConvUint64ToDouble:
    case qglslang::EOpConvUint64ToFloat16:
        convOp = spv::OpConvertUToF;
        break;

    case qglslang::EOpConvFloat16ToInt8:
    case qglslang::EOpConvFloatToInt8:
    case qglslang::EOpConvDoubleToInt8:
    case qglslang::EOpConvFloat16ToInt16:
    case qglslang::EOpConvFloatToInt16:
    case qglslang::EOpConvDoubleToInt16:
    case qglslang::EOpConvFloat16ToInt:
    case qglslang::EOpConvFloatToInt:
    case qglslang::EOpConvDoubleToInt:
    case qglslang::EOpConvFloat16ToInt64:
    case qglslang::EOpConvFloatToInt64:
    case qglslang::EOpConvDoubleToInt64:
        convOp = spv::OpConvertFToS;
        break;

    case qglslang::EOpConvUint8ToInt8:
    case qglslang::EOpConvInt8ToUint8:
    case qglslang::EOpConvUint16ToInt16:
    case qglslang::EOpConvInt16ToUint16:
    case qglslang::EOpConvUintToInt:
    case qglslang::EOpConvIntToUint:
    case qglslang::EOpConvUint64ToInt64:
    case qglslang::EOpConvInt64ToUint64:
        if (builder.isInSpecConstCodeGenMode()) {
            // Build zero scalar or vector for OpIAdd.
#ifndef GLSLANG_WEB
            if(op == qglslang::EOpConvUint8ToInt8 || op == qglslang::EOpConvInt8ToUint8) {
                zero = builder.makeUint8Constant(0);
            } else if (op == qglslang::EOpConvUint16ToInt16 || op == qglslang::EOpConvInt16ToUint16) {
                zero = builder.makeUint16Constant(0);
            } else if (op == qglslang::EOpConvUint64ToInt64 || op == qglslang::EOpConvInt64ToUint64) {
                zero = builder.makeUint64Constant(0);
            } else
#endif
            {
                zero = builder.makeUintConstant(0);
            }
            zero = makeSmearedConstant(zero, vectorSize);
            // Use OpIAdd, instead of OpBitcast to do the conversion when
            // generating for OpSpecConstantOp instruction.
            return builder.createBinOp(spv::OpIAdd, destType, operand, zero);
        }
        // For normal run-time conversion instruction, use OpBitcast.
        convOp = spv::OpBitcast;
        break;

    case qglslang::EOpConvFloat16ToUint8:
    case qglslang::EOpConvFloatToUint8:
    case qglslang::EOpConvDoubleToUint8:
    case qglslang::EOpConvFloat16ToUint16:
    case qglslang::EOpConvFloatToUint16:
    case qglslang::EOpConvDoubleToUint16:
    case qglslang::EOpConvFloat16ToUint:
    case qglslang::EOpConvFloatToUint:
    case qglslang::EOpConvDoubleToUint:
    case qglslang::EOpConvFloatToUint64:
    case qglslang::EOpConvDoubleToUint64:
    case qglslang::EOpConvFloat16ToUint64:
        convOp = spv::OpConvertFToU;
        break;

#ifndef GLSLANG_WEB
    case qglslang::EOpConvInt8ToBool:
    case qglslang::EOpConvUint8ToBool:
        zero = builder.makeUint8Constant(0);
        zero = makeSmearedConstant(zero, vectorSize);
        return builder.createBinOp(spv::OpINotEqual, destType, operand, zero);
    case qglslang::EOpConvInt16ToBool:
    case qglslang::EOpConvUint16ToBool:
        zero = builder.makeUint16Constant(0);
        zero = makeSmearedConstant(zero, vectorSize);
        return builder.createBinOp(spv::OpINotEqual, destType, operand, zero);
    case qglslang::EOpConvInt64ToBool:
    case qglslang::EOpConvUint64ToBool:
        zero = builder.makeUint64Constant(0);
        zero = makeSmearedConstant(zero, vectorSize);
        return builder.createBinOp(spv::OpINotEqual, destType, operand, zero);
    case qglslang::EOpConvDoubleToBool:
        zero = builder.makeDoubleConstant(0.0);
        zero = makeSmearedConstant(zero, vectorSize);
        return builder.createBinOp(spv::OpFUnordNotEqual, destType, operand, zero);
    case qglslang::EOpConvFloat16ToBool:
        zero = builder.makeFloat16Constant(0.0F);
        zero = makeSmearedConstant(zero, vectorSize);
        return builder.createBinOp(spv::OpFUnordNotEqual, destType, operand, zero);
    case qglslang::EOpConvBoolToDouble:
        convOp = spv::OpSelect;
        zero = builder.makeDoubleConstant(0.0);
        one  = builder.makeDoubleConstant(1.0);
        break;
    case qglslang::EOpConvBoolToFloat16:
        convOp = spv::OpSelect;
        zero = builder.makeFloat16Constant(0.0F);
        one = builder.makeFloat16Constant(1.0F);
        break;
    case qglslang::EOpConvBoolToInt8:
        zero = builder.makeInt8Constant(0);
        one  = builder.makeInt8Constant(1);
        convOp = spv::OpSelect;
        break;
    case qglslang::EOpConvBoolToUint8:
        zero = builder.makeUint8Constant(0);
        one  = builder.makeUint8Constant(1);
        convOp = spv::OpSelect;
        break;
    case qglslang::EOpConvBoolToInt16:
        zero = builder.makeInt16Constant(0);
        one  = builder.makeInt16Constant(1);
        convOp = spv::OpSelect;
        break;
    case qglslang::EOpConvBoolToUint16:
        zero = builder.makeUint16Constant(0);
        one  = builder.makeUint16Constant(1);
        convOp = spv::OpSelect;
        break;
    case qglslang::EOpConvDoubleToFloat:
    case qglslang::EOpConvFloatToDouble:
    case qglslang::EOpConvDoubleToFloat16:
    case qglslang::EOpConvFloat16ToDouble:
    case qglslang::EOpConvFloatToFloat16:
    case qglslang::EOpConvFloat16ToFloat:
        convOp = spv::OpFConvert;
        if (builder.isMatrixType(destType))
            return createUnaryMatrixOperation(convOp, decorations, destType, operand, typeProxy);
        break;

    case qglslang::EOpConvInt8ToInt16:
    case qglslang::EOpConvInt8ToInt:
    case qglslang::EOpConvInt8ToInt64:
    case qglslang::EOpConvInt16ToInt8:
    case qglslang::EOpConvInt16ToInt:
    case qglslang::EOpConvInt16ToInt64:
    case qglslang::EOpConvIntToInt8:
    case qglslang::EOpConvIntToInt16:
    case qglslang::EOpConvIntToInt64:
    case qglslang::EOpConvInt64ToInt8:
    case qglslang::EOpConvInt64ToInt16:
    case qglslang::EOpConvInt64ToInt:
        convOp = spv::OpSConvert;
        break;

    case qglslang::EOpConvUint8ToUint16:
    case qglslang::EOpConvUint8ToUint:
    case qglslang::EOpConvUint8ToUint64:
    case qglslang::EOpConvUint16ToUint8:
    case qglslang::EOpConvUint16ToUint:
    case qglslang::EOpConvUint16ToUint64:
    case qglslang::EOpConvUintToUint8:
    case qglslang::EOpConvUintToUint16:
    case qglslang::EOpConvUintToUint64:
    case qglslang::EOpConvUint64ToUint8:
    case qglslang::EOpConvUint64ToUint16:
    case qglslang::EOpConvUint64ToUint:
        convOp = spv::OpUConvert;
        break;

    case qglslang::EOpConvInt8ToUint16:
    case qglslang::EOpConvInt8ToUint:
    case qglslang::EOpConvInt8ToUint64:
    case qglslang::EOpConvInt16ToUint8:
    case qglslang::EOpConvInt16ToUint:
    case qglslang::EOpConvInt16ToUint64:
    case qglslang::EOpConvIntToUint8:
    case qglslang::EOpConvIntToUint16:
    case qglslang::EOpConvIntToUint64:
    case qglslang::EOpConvInt64ToUint8:
    case qglslang::EOpConvInt64ToUint16:
    case qglslang::EOpConvInt64ToUint:
    case qglslang::EOpConvUint8ToInt16:
    case qglslang::EOpConvUint8ToInt:
    case qglslang::EOpConvUint8ToInt64:
    case qglslang::EOpConvUint16ToInt8:
    case qglslang::EOpConvUint16ToInt:
    case qglslang::EOpConvUint16ToInt64:
    case qglslang::EOpConvUintToInt8:
    case qglslang::EOpConvUintToInt16:
    case qglslang::EOpConvUintToInt64:
    case qglslang::EOpConvUint64ToInt8:
    case qglslang::EOpConvUint64ToInt16:
    case qglslang::EOpConvUint64ToInt:
        // OpSConvert/OpUConvert + OpBitCast
        operand = createIntWidthConversion(op, operand, vectorSize);

        if (builder.isInSpecConstCodeGenMode()) {
            // Build zero scalar or vector for OpIAdd.
            switch(op) {
            case qglslang::EOpConvInt16ToUint8:
            case qglslang::EOpConvIntToUint8:
            case qglslang::EOpConvInt64ToUint8:
            case qglslang::EOpConvUint16ToInt8:
            case qglslang::EOpConvUintToInt8:
            case qglslang::EOpConvUint64ToInt8:
                zero = builder.makeUint8Constant(0);
                break;
            case qglslang::EOpConvInt8ToUint16:
            case qglslang::EOpConvIntToUint16:
            case qglslang::EOpConvInt64ToUint16:
            case qglslang::EOpConvUint8ToInt16:
            case qglslang::EOpConvUintToInt16:
            case qglslang::EOpConvUint64ToInt16:
                zero = builder.makeUint16Constant(0);
                break;
            case qglslang::EOpConvInt8ToUint:
            case qglslang::EOpConvInt16ToUint:
            case qglslang::EOpConvInt64ToUint:
            case qglslang::EOpConvUint8ToInt:
            case qglslang::EOpConvUint16ToInt:
            case qglslang::EOpConvUint64ToInt:
                zero = builder.makeUintConstant(0);
                break;
            case qglslang::EOpConvInt8ToUint64:
            case qglslang::EOpConvInt16ToUint64:
            case qglslang::EOpConvIntToUint64:
            case qglslang::EOpConvUint8ToInt64:
            case qglslang::EOpConvUint16ToInt64:
            case qglslang::EOpConvUintToInt64:
                zero = builder.makeUint64Constant(0);
                break;
            default:
                assert(false && "Default missing");
                break;
            }
            zero = makeSmearedConstant(zero, vectorSize);
            // Use OpIAdd, instead of OpBitcast to do the conversion when
            // generating for OpSpecConstantOp instruction.
            return builder.createBinOp(spv::OpIAdd, destType, operand, zero);
        }
        // For normal run-time conversion instruction, use OpBitcast.
        convOp = spv::OpBitcast;
        break;
    case qglslang::EOpConvUint64ToPtr:
        convOp = spv::OpConvertUToPtr;
        break;
    case qglslang::EOpConvPtrToUint64:
        convOp = spv::OpConvertPtrToU;
        break;
    case qglslang::EOpConvPtrToUvec2:
    case qglslang::EOpConvUvec2ToPtr:
        convOp = spv::OpBitcast;
        break;
#endif

    default:
        break;
    }

    spv::Id result = 0;
    if (convOp == spv::OpNop)
        return result;

    if (convOp == spv::OpSelect) {
        zero = makeSmearedConstant(zero, vectorSize);
        one  = makeSmearedConstant(one, vectorSize);
        result = builder.createTriOp(convOp, destType, operand, one, zero);
    } else
        result = builder.createUnaryOp(convOp, destType, operand);

    result = builder.setPrecision(result, decorations.precision);
    decorations.addNonUniform(builder, result);
    return result;
}

spv::Id TGlslangToSpvTraverser::makeSmearedConstant(spv::Id constant, int vectorSize)
{
    if (vectorSize == 0)
        return constant;

    spv::Id vectorTypeId = builder.makeVectorType(builder.getTypeId(constant), vectorSize);
    std::vector<spv::Id> components;
    for (int c = 0; c < vectorSize; ++c)
        components.push_back(constant);
    return builder.makeCompositeConstant(vectorTypeId, components);
}

// For glslang ops that map to SPV atomic opCodes
spv::Id TGlslangToSpvTraverser::createAtomicOperation(qglslang::TOperator op, spv::Decoration /*precision*/,
    spv::Id typeId, std::vector<spv::Id>& operands, qglslang::TBasicType typeProxy,
    const spv::Builder::AccessChain::CoherentFlags &lvalueCoherentFlags)
{
    spv::Op opCode = spv::OpNop;

    switch (op) {
    case qglslang::EOpAtomicAdd:
    case qglslang::EOpImageAtomicAdd:
    case qglslang::EOpAtomicCounterAdd:
        opCode = spv::OpAtomicIAdd;
        if (typeProxy == qglslang::EbtFloat16 || typeProxy == qglslang::EbtFloat || typeProxy == qglslang::EbtDouble) {
            opCode = spv::OpAtomicFAddEXT;
            builder.addExtension(spv::E_SPV_EXT_shader_atomic_float_add);
            if (typeProxy == qglslang::EbtFloat16) {
                builder.addExtension(spv::E_SPV_EXT_shader_atomic_float16_add);
                builder.addCapability(spv::CapabilityAtomicFloat16AddEXT);
            } else if (typeProxy == qglslang::EbtFloat) {
                builder.addCapability(spv::CapabilityAtomicFloat32AddEXT);
            } else {
                builder.addCapability(spv::CapabilityAtomicFloat64AddEXT);
            }
        }
        break;
    case qglslang::EOpAtomicSubtract:
    case qglslang::EOpAtomicCounterSubtract:
        opCode = spv::OpAtomicISub;
        break;
    case qglslang::EOpAtomicMin:
    case qglslang::EOpImageAtomicMin:
    case qglslang::EOpAtomicCounterMin:
        if (typeProxy == qglslang::EbtFloat16 || typeProxy == qglslang::EbtFloat || typeProxy == qglslang::EbtDouble) {
            opCode = spv::OpAtomicFMinEXT;
            builder.addExtension(spv::E_SPV_EXT_shader_atomic_float_min_max);
            if (typeProxy == qglslang::EbtFloat16)
                builder.addCapability(spv::CapabilityAtomicFloat16MinMaxEXT);
            else if (typeProxy == qglslang::EbtFloat)
                builder.addCapability(spv::CapabilityAtomicFloat32MinMaxEXT);
            else
                builder.addCapability(spv::CapabilityAtomicFloat64MinMaxEXT);
        } else if (typeProxy == qglslang::EbtUint || typeProxy == qglslang::EbtUint64) {
            opCode = spv::OpAtomicUMin;
        } else {
            opCode = spv::OpAtomicSMin;
        }
        break;
    case qglslang::EOpAtomicMax:
    case qglslang::EOpImageAtomicMax:
    case qglslang::EOpAtomicCounterMax:
        if (typeProxy == qglslang::EbtFloat16 || typeProxy == qglslang::EbtFloat || typeProxy == qglslang::EbtDouble) {
            opCode = spv::OpAtomicFMaxEXT;
            builder.addExtension(spv::E_SPV_EXT_shader_atomic_float_min_max);
            if (typeProxy == qglslang::EbtFloat16)
                builder.addCapability(spv::CapabilityAtomicFloat16MinMaxEXT);
            else if (typeProxy == qglslang::EbtFloat)
                builder.addCapability(spv::CapabilityAtomicFloat32MinMaxEXT);
            else
                builder.addCapability(spv::CapabilityAtomicFloat64MinMaxEXT);
        } else if (typeProxy == qglslang::EbtUint || typeProxy == qglslang::EbtUint64) {
            opCode = spv::OpAtomicUMax;
        } else {
            opCode = spv::OpAtomicSMax;
        }
        break;
    case qglslang::EOpAtomicAnd:
    case qglslang::EOpImageAtomicAnd:
    case qglslang::EOpAtomicCounterAnd:
        opCode = spv::OpAtomicAnd;
        break;
    case qglslang::EOpAtomicOr:
    case qglslang::EOpImageAtomicOr:
    case qglslang::EOpAtomicCounterOr:
        opCode = spv::OpAtomicOr;
        break;
    case qglslang::EOpAtomicXor:
    case qglslang::EOpImageAtomicXor:
    case qglslang::EOpAtomicCounterXor:
        opCode = spv::OpAtomicXor;
        break;
    case qglslang::EOpAtomicExchange:
    case qglslang::EOpImageAtomicExchange:
    case qglslang::EOpAtomicCounterExchange:
        opCode = spv::OpAtomicExchange;
        break;
    case qglslang::EOpAtomicCompSwap:
    case qglslang::EOpImageAtomicCompSwap:
    case qglslang::EOpAtomicCounterCompSwap:
        opCode = spv::OpAtomicCompareExchange;
        break;
    case qglslang::EOpAtomicCounterIncrement:
        opCode = spv::OpAtomicIIncrement;
        break;
    case qglslang::EOpAtomicCounterDecrement:
        opCode = spv::OpAtomicIDecrement;
        break;
    case qglslang::EOpAtomicCounter:
    case qglslang::EOpImageAtomicLoad:
    case qglslang::EOpAtomicLoad:
        opCode = spv::OpAtomicLoad;
        break;
    case qglslang::EOpAtomicStore:
    case qglslang::EOpImageAtomicStore:
        opCode = spv::OpAtomicStore;
        break;
    default:
        assert(0);
        break;
    }

    if (typeProxy == qglslang::EbtInt64 || typeProxy == qglslang::EbtUint64)
        builder.addCapability(spv::CapabilityInt64Atomics);

    // Sort out the operands
    //  - mapping from glslang -> SPV
    //  - there are extra SPV operands that are optional in glslang
    //  - compare-exchange swaps the value and comparator
    //  - compare-exchange has an extra memory semantics
    //  - EOpAtomicCounterDecrement needs a post decrement
    spv::Id pointerId = 0, compareId = 0, valueId = 0;
    // scope defaults to Device in the old model, QueueFamilyKHR in the new model
    spv::Id scopeId;
    if (glslangIntermediate->usingVulkanMemoryModel()) {
        scopeId = builder.makeUintConstant(spv::ScopeQueueFamilyKHR);
    } else {
        scopeId = builder.makeUintConstant(spv::ScopeDevice);
    }
    // semantics default to relaxed 
    spv::Id semanticsId = builder.makeUintConstant(lvalueCoherentFlags.isVolatile() &&
        glslangIntermediate->usingVulkanMemoryModel() ?
                                                    spv::MemorySemanticsVolatileMask :
                                                    spv::MemorySemanticsMaskNone);
    spv::Id semanticsId2 = semanticsId;

    pointerId = operands[0];
    if (opCode == spv::OpAtomicIIncrement || opCode == spv::OpAtomicIDecrement) {
        // no additional operands
    } else if (opCode == spv::OpAtomicCompareExchange) {
        compareId = operands[1];
        valueId = operands[2];
        if (operands.size() > 3) {
            scopeId = operands[3];
            semanticsId = builder.makeUintConstant(
                builder.getConstantScalar(operands[4]) | builder.getConstantScalar(operands[5]));
            semanticsId2 = builder.makeUintConstant(
                builder.getConstantScalar(operands[6]) | builder.getConstantScalar(operands[7]));
        }
    } else if (opCode == spv::OpAtomicLoad) {
        if (operands.size() > 1) {
            scopeId = operands[1];
            semanticsId = builder.makeUintConstant(
                builder.getConstantScalar(operands[2]) | builder.getConstantScalar(operands[3]));
        }
    } else {
        // atomic store or RMW
        valueId = operands[1];
        if (operands.size() > 2) {
            scopeId = operands[2];
            semanticsId = builder.makeUintConstant
                (builder.getConstantScalar(operands[3]) | builder.getConstantScalar(operands[4]));
        }
    }

    // Check for capabilities
    unsigned semanticsImmediate = builder.getConstantScalar(semanticsId) | builder.getConstantScalar(semanticsId2);
    if (semanticsImmediate & (spv::MemorySemanticsMakeAvailableKHRMask |
                              spv::MemorySemanticsMakeVisibleKHRMask |
                              spv::MemorySemanticsOutputMemoryKHRMask |
                              spv::MemorySemanticsVolatileMask)) {
        builder.addCapability(spv::CapabilityVulkanMemoryModelKHR);
    }

    if (builder.getConstantScalar(scopeId) == spv::ScopeQueueFamily) {
        builder.addCapability(spv::CapabilityVulkanMemoryModelKHR);
    }

    if (glslangIntermediate->usingVulkanMemoryModel() && builder.getConstantScalar(scopeId) == spv::ScopeDevice) {
        builder.addCapability(spv::CapabilityVulkanMemoryModelDeviceScopeKHR);
    }

    std::vector<spv::Id> spvAtomicOperands;  // hold the spv operands
    spvAtomicOperands.push_back(pointerId);
    spvAtomicOperands.push_back(scopeId);
    spvAtomicOperands.push_back(semanticsId);
    if (opCode == spv::OpAtomicCompareExchange) {
        spvAtomicOperands.push_back(semanticsId2);
        spvAtomicOperands.push_back(valueId);
        spvAtomicOperands.push_back(compareId);
    } else if (opCode != spv::OpAtomicLoad && opCode != spv::OpAtomicIIncrement && opCode != spv::OpAtomicIDecrement) {
        spvAtomicOperands.push_back(valueId);
    }

    if (opCode == spv::OpAtomicStore) {
        builder.createNoResultOp(opCode, spvAtomicOperands);
        return 0;
    } else {
        spv::Id resultId = builder.createOp(opCode, typeId, spvAtomicOperands);

        // GLSL and HLSL atomic-counter decrement return post-decrement value,
        // while SPIR-V returns pre-decrement value. Translate between these semantics.
        if (op == qglslang::EOpAtomicCounterDecrement)
            resultId = builder.createBinOp(spv::OpISub, typeId, resultId, builder.makeIntConstant(1));

        return resultId;
    }
}

// Create group invocation operations.
spv::Id TGlslangToSpvTraverser::createInvocationsOperation(qglslang::TOperator op, spv::Id typeId,
    std::vector<spv::Id>& operands, qglslang::TBasicType typeProxy)
{
    bool isUnsigned = isTypeUnsignedInt(typeProxy);
    bool isFloat = isTypeFloat(typeProxy);

    spv::Op opCode = spv::OpNop;
    std::vector<spv::IdImmediate> spvGroupOperands;
    spv::GroupOperation groupOperation = spv::GroupOperationMax;

    if (op == qglslang::EOpBallot || op == qglslang::EOpReadFirstInvocation ||
        op == qglslang::EOpReadInvocation) {
        builder.addExtension(spv::E_SPV_KHR_shader_ballot);
        builder.addCapability(spv::CapabilitySubgroupBallotKHR);
    } else if (op == qglslang::EOpAnyInvocation ||
        op == qglslang::EOpAllInvocations ||
        op == qglslang::EOpAllInvocationsEqual) {
        builder.addExtension(spv::E_SPV_KHR_subgroup_vote);
        builder.addCapability(spv::CapabilitySubgroupVoteKHR);
    } else {
        builder.addCapability(spv::CapabilityGroups);
        if (op == qglslang::EOpMinInvocationsNonUniform ||
            op == qglslang::EOpMaxInvocationsNonUniform ||
            op == qglslang::EOpAddInvocationsNonUniform ||
            op == qglslang::EOpMinInvocationsInclusiveScanNonUniform ||
            op == qglslang::EOpMaxInvocationsInclusiveScanNonUniform ||
            op == qglslang::EOpAddInvocationsInclusiveScanNonUniform ||
            op == qglslang::EOpMinInvocationsExclusiveScanNonUniform ||
            op == qglslang::EOpMaxInvocationsExclusiveScanNonUniform ||
            op == qglslang::EOpAddInvocationsExclusiveScanNonUniform)
            builder.addExtension(spv::E_SPV_AMD_shader_ballot);

        switch (op) {
        case qglslang::EOpMinInvocations:
        case qglslang::EOpMaxInvocations:
        case qglslang::EOpAddInvocations:
        case qglslang::EOpMinInvocationsNonUniform:
        case qglslang::EOpMaxInvocationsNonUniform:
        case qglslang::EOpAddInvocationsNonUniform:
            groupOperation = spv::GroupOperationReduce;
            break;
        case qglslang::EOpMinInvocationsInclusiveScan:
        case qglslang::EOpMaxInvocationsInclusiveScan:
        case qglslang::EOpAddInvocationsInclusiveScan:
        case qglslang::EOpMinInvocationsInclusiveScanNonUniform:
        case qglslang::EOpMaxInvocationsInclusiveScanNonUniform:
        case qglslang::EOpAddInvocationsInclusiveScanNonUniform:
            groupOperation = spv::GroupOperationInclusiveScan;
            break;
        case qglslang::EOpMinInvocationsExclusiveScan:
        case qglslang::EOpMaxInvocationsExclusiveScan:
        case qglslang::EOpAddInvocationsExclusiveScan:
        case qglslang::EOpMinInvocationsExclusiveScanNonUniform:
        case qglslang::EOpMaxInvocationsExclusiveScanNonUniform:
        case qglslang::EOpAddInvocationsExclusiveScanNonUniform:
            groupOperation = spv::GroupOperationExclusiveScan;
            break;
        default:
            break;
        }
        spv::IdImmediate scope = { true, builder.makeUintConstant(spv::ScopeSubgroup) };
        spvGroupOperands.push_back(scope);
        if (groupOperation != spv::GroupOperationMax) {
            spv::IdImmediate groupOp = { false, (unsigned)groupOperation };
            spvGroupOperands.push_back(groupOp);
        }
    }

    for (auto opIt = operands.begin(); opIt != operands.end(); ++opIt) {
        spv::IdImmediate op = { true, *opIt };
        spvGroupOperands.push_back(op);
    }

    switch (op) {
    case qglslang::EOpAnyInvocation:
        opCode = spv::OpSubgroupAnyKHR;
        break;
    case qglslang::EOpAllInvocations:
        opCode = spv::OpSubgroupAllKHR;
        break;
    case qglslang::EOpAllInvocationsEqual:
        opCode = spv::OpSubgroupAllEqualKHR;
        break;
    case qglslang::EOpReadInvocation:
        opCode = spv::OpSubgroupReadInvocationKHR;
        if (builder.isVectorType(typeId))
            return CreateInvocationsVectorOperation(opCode, groupOperation, typeId, operands);
        break;
    case qglslang::EOpReadFirstInvocation:
        opCode = spv::OpSubgroupFirstInvocationKHR;
        if (builder.isVectorType(typeId))
            return CreateInvocationsVectorOperation(opCode, groupOperation, typeId, operands);
        break;
    case qglslang::EOpBallot:
    {
        // NOTE: According to the spec, the result type of "OpSubgroupBallotKHR" must be a 4 component vector of 32
        // bit integer types. The GLSL built-in function "ballotARB()" assumes the maximum number of invocations in
        // a subgroup is 64. Thus, we have to convert uvec4.xy to uint64_t as follow:
        //
        //     result = Bitcast(SubgroupBallotKHR(Predicate).xy)
        //
        spv::Id uintType  = builder.makeUintType(32);
        spv::Id uvec4Type = builder.makeVectorType(uintType, 4);
        spv::Id result = builder.createOp(spv::OpSubgroupBallotKHR, uvec4Type, spvGroupOperands);

        std::vector<spv::Id> components;
        components.push_back(builder.createCompositeExtract(result, uintType, 0));
        components.push_back(builder.createCompositeExtract(result, uintType, 1));

        spv::Id uvec2Type = builder.makeVectorType(uintType, 2);
        return builder.createUnaryOp(spv::OpBitcast, typeId,
                                     builder.createCompositeConstruct(uvec2Type, components));
    }

    case qglslang::EOpMinInvocations:
    case qglslang::EOpMaxInvocations:
    case qglslang::EOpAddInvocations:
    case qglslang::EOpMinInvocationsInclusiveScan:
    case qglslang::EOpMaxInvocationsInclusiveScan:
    case qglslang::EOpAddInvocationsInclusiveScan:
    case qglslang::EOpMinInvocationsExclusiveScan:
    case qglslang::EOpMaxInvocationsExclusiveScan:
    case qglslang::EOpAddInvocationsExclusiveScan:
        if (op == qglslang::EOpMinInvocations ||
            op == qglslang::EOpMinInvocationsInclusiveScan ||
            op == qglslang::EOpMinInvocationsExclusiveScan) {
            if (isFloat)
                opCode = spv::OpGroupFMin;
            else {
                if (isUnsigned)
                    opCode = spv::OpGroupUMin;
                else
                    opCode = spv::OpGroupSMin;
            }
        } else if (op == qglslang::EOpMaxInvocations ||
                   op == qglslang::EOpMaxInvocationsInclusiveScan ||
                   op == qglslang::EOpMaxInvocationsExclusiveScan) {
            if (isFloat)
                opCode = spv::OpGroupFMax;
            else {
                if (isUnsigned)
                    opCode = spv::OpGroupUMax;
                else
                    opCode = spv::OpGroupSMax;
            }
        } else {
            if (isFloat)
                opCode = spv::OpGroupFAdd;
            else
                opCode = spv::OpGroupIAdd;
        }

        if (builder.isVectorType(typeId))
            return CreateInvocationsVectorOperation(opCode, groupOperation, typeId, operands);

        break;
    case qglslang::EOpMinInvocationsNonUniform:
    case qglslang::EOpMaxInvocationsNonUniform:
    case qglslang::EOpAddInvocationsNonUniform:
    case qglslang::EOpMinInvocationsInclusiveScanNonUniform:
    case qglslang::EOpMaxInvocationsInclusiveScanNonUniform:
    case qglslang::EOpAddInvocationsInclusiveScanNonUniform:
    case qglslang::EOpMinInvocationsExclusiveScanNonUniform:
    case qglslang::EOpMaxInvocationsExclusiveScanNonUniform:
    case qglslang::EOpAddInvocationsExclusiveScanNonUniform:
        if (op == qglslang::EOpMinInvocationsNonUniform ||
            op == qglslang::EOpMinInvocationsInclusiveScanNonUniform ||
            op == qglslang::EOpMinInvocationsExclusiveScanNonUniform) {
            if (isFloat)
                opCode = spv::OpGroupFMinNonUniformAMD;
            else {
                if (isUnsigned)
                    opCode = spv::OpGroupUMinNonUniformAMD;
                else
                    opCode = spv::OpGroupSMinNonUniformAMD;
            }
        }
        else if (op == qglslang::EOpMaxInvocationsNonUniform ||
                 op == qglslang::EOpMaxInvocationsInclusiveScanNonUniform ||
                 op == qglslang::EOpMaxInvocationsExclusiveScanNonUniform) {
            if (isFloat)
                opCode = spv::OpGroupFMaxNonUniformAMD;
            else {
                if (isUnsigned)
                    opCode = spv::OpGroupUMaxNonUniformAMD;
                else
                    opCode = spv::OpGroupSMaxNonUniformAMD;
            }
        }
        else {
            if (isFloat)
                opCode = spv::OpGroupFAddNonUniformAMD;
            else
                opCode = spv::OpGroupIAddNonUniformAMD;
        }

        if (builder.isVectorType(typeId))
            return CreateInvocationsVectorOperation(opCode, groupOperation, typeId, operands);

        break;
    default:
        logger->missingFunctionality("invocation operation");
        return spv::NoResult;
    }

    assert(opCode != spv::OpNop);
    return builder.createOp(opCode, typeId, spvGroupOperands);
}

// Create group invocation operations on a vector
spv::Id TGlslangToSpvTraverser::CreateInvocationsVectorOperation(spv::Op op, spv::GroupOperation groupOperation,
    spv::Id typeId, std::vector<spv::Id>& operands)
{
    assert(op == spv::OpGroupFMin || op == spv::OpGroupUMin || op == spv::OpGroupSMin ||
           op == spv::OpGroupFMax || op == spv::OpGroupUMax || op == spv::OpGroupSMax ||
           op == spv::OpGroupFAdd || op == spv::OpGroupIAdd || op == spv::OpGroupBroadcast ||
           op == spv::OpSubgroupReadInvocationKHR || op == spv::OpSubgroupFirstInvocationKHR ||
           op == spv::OpGroupFMinNonUniformAMD || op == spv::OpGroupUMinNonUniformAMD ||
           op == spv::OpGroupSMinNonUniformAMD ||
           op == spv::OpGroupFMaxNonUniformAMD || op == spv::OpGroupUMaxNonUniformAMD ||
           op == spv::OpGroupSMaxNonUniformAMD ||
           op == spv::OpGroupFAddNonUniformAMD || op == spv::OpGroupIAddNonUniformAMD);

    // Handle group invocation operations scalar by scalar.
    // The result type is the same type as the original type.
    // The algorithm is to:
    //   - break the vector into scalars
    //   - apply the operation to each scalar
    //   - make a vector out the scalar results

    // get the types sorted out
    int numComponents = builder.getNumComponents(operands[0]);
    spv::Id scalarType = builder.getScalarTypeId(builder.getTypeId(operands[0]));
    std::vector<spv::Id> results;

    // do each scalar op
    for (int comp = 0; comp < numComponents; ++comp) {
        std::vector<unsigned int> indexes;
        indexes.push_back(comp);
        spv::IdImmediate scalar = { true, builder.createCompositeExtract(operands[0], scalarType, indexes) };
        std::vector<spv::IdImmediate> spvGroupOperands;
        if (op == spv::OpSubgroupReadInvocationKHR) {
            spvGroupOperands.push_back(scalar);
            spv::IdImmediate operand = { true, operands[1] };
            spvGroupOperands.push_back(operand);
        } else if (op == spv::OpSubgroupFirstInvocationKHR) {
            spvGroupOperands.push_back(scalar);
        } else if (op == spv::OpGroupBroadcast) {
            spv::IdImmediate scope = { true, builder.makeUintConstant(spv::ScopeSubgroup) };
            spvGroupOperands.push_back(scope);
            spvGroupOperands.push_back(scalar);
            spv::IdImmediate operand = { true, operands[1] };
            spvGroupOperands.push_back(operand);
        } else {
            spv::IdImmediate scope = { true, builder.makeUintConstant(spv::ScopeSubgroup) };
            spvGroupOperands.push_back(scope);
            spv::IdImmediate groupOp = { false, (unsigned)groupOperation };
            spvGroupOperands.push_back(groupOp);
            spvGroupOperands.push_back(scalar);
        }

        results.push_back(builder.createOp(op, scalarType, spvGroupOperands));
    }

    // put the pieces together
    return builder.createCompositeConstruct(typeId, results);
}

// Create subgroup invocation operations.
spv::Id TGlslangToSpvTraverser::createSubgroupOperation(qglslang::TOperator op, spv::Id typeId,
    std::vector<spv::Id>& operands, qglslang::TBasicType typeProxy)
{
    // Add the required capabilities.
    switch (op) {
    case qglslang::EOpSubgroupElect:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        break;
    case qglslang::EOpSubgroupAll:
    case qglslang::EOpSubgroupAny:
    case qglslang::EOpSubgroupAllEqual:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformVote);
        break;
    case qglslang::EOpSubgroupBroadcast:
    case qglslang::EOpSubgroupBroadcastFirst:
    case qglslang::EOpSubgroupBallot:
    case qglslang::EOpSubgroupInverseBallot:
    case qglslang::EOpSubgroupBallotBitExtract:
    case qglslang::EOpSubgroupBallotBitCount:
    case qglslang::EOpSubgroupBallotInclusiveBitCount:
    case qglslang::EOpSubgroupBallotExclusiveBitCount:
    case qglslang::EOpSubgroupBallotFindLSB:
    case qglslang::EOpSubgroupBallotFindMSB:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformBallot);
        break;
    case qglslang::EOpSubgroupShuffle:
    case qglslang::EOpSubgroupShuffleXor:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformShuffle);
        break;
    case qglslang::EOpSubgroupShuffleUp:
    case qglslang::EOpSubgroupShuffleDown:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformShuffleRelative);
        break;
    case qglslang::EOpSubgroupAdd:
    case qglslang::EOpSubgroupMul:
    case qglslang::EOpSubgroupMin:
    case qglslang::EOpSubgroupMax:
    case qglslang::EOpSubgroupAnd:
    case qglslang::EOpSubgroupOr:
    case qglslang::EOpSubgroupXor:
    case qglslang::EOpSubgroupInclusiveAdd:
    case qglslang::EOpSubgroupInclusiveMul:
    case qglslang::EOpSubgroupInclusiveMin:
    case qglslang::EOpSubgroupInclusiveMax:
    case qglslang::EOpSubgroupInclusiveAnd:
    case qglslang::EOpSubgroupInclusiveOr:
    case qglslang::EOpSubgroupInclusiveXor:
    case qglslang::EOpSubgroupExclusiveAdd:
    case qglslang::EOpSubgroupExclusiveMul:
    case qglslang::EOpSubgroupExclusiveMin:
    case qglslang::EOpSubgroupExclusiveMax:
    case qglslang::EOpSubgroupExclusiveAnd:
    case qglslang::EOpSubgroupExclusiveOr:
    case qglslang::EOpSubgroupExclusiveXor:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformArithmetic);
        break;
    case qglslang::EOpSubgroupClusteredAdd:
    case qglslang::EOpSubgroupClusteredMul:
    case qglslang::EOpSubgroupClusteredMin:
    case qglslang::EOpSubgroupClusteredMax:
    case qglslang::EOpSubgroupClusteredAnd:
    case qglslang::EOpSubgroupClusteredOr:
    case qglslang::EOpSubgroupClusteredXor:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformClustered);
        break;
    case qglslang::EOpSubgroupQuadBroadcast:
    case qglslang::EOpSubgroupQuadSwapHorizontal:
    case qglslang::EOpSubgroupQuadSwapVertical:
    case qglslang::EOpSubgroupQuadSwapDiagonal:
        builder.addCapability(spv::CapabilityGroupNonUniform);
        builder.addCapability(spv::CapabilityGroupNonUniformQuad);
        break;
    case qglslang::EOpSubgroupPartitionedAdd:
    case qglslang::EOpSubgroupPartitionedMul:
    case qglslang::EOpSubgroupPartitionedMin:
    case qglslang::EOpSubgroupPartitionedMax:
    case qglslang::EOpSubgroupPartitionedAnd:
    case qglslang::EOpSubgroupPartitionedOr:
    case qglslang::EOpSubgroupPartitionedXor:
    case qglslang::EOpSubgroupPartitionedInclusiveAdd:
    case qglslang::EOpSubgroupPartitionedInclusiveMul:
    case qglslang::EOpSubgroupPartitionedInclusiveMin:
    case qglslang::EOpSubgroupPartitionedInclusiveMax:
    case qglslang::EOpSubgroupPartitionedInclusiveAnd:
    case qglslang::EOpSubgroupPartitionedInclusiveOr:
    case qglslang::EOpSubgroupPartitionedInclusiveXor:
    case qglslang::EOpSubgroupPartitionedExclusiveAdd:
    case qglslang::EOpSubgroupPartitionedExclusiveMul:
    case qglslang::EOpSubgroupPartitionedExclusiveMin:
    case qglslang::EOpSubgroupPartitionedExclusiveMax:
    case qglslang::EOpSubgroupPartitionedExclusiveAnd:
    case qglslang::EOpSubgroupPartitionedExclusiveOr:
    case qglslang::EOpSubgroupPartitionedExclusiveXor:
        builder.addExtension(spv::E_SPV_NV_shader_subgroup_partitioned);
        builder.addCapability(spv::CapabilityGroupNonUniformPartitionedNV);
        break;
    default: assert(0 && "Unhandled subgroup operation!");
    }


    const bool isUnsigned = isTypeUnsignedInt(typeProxy);
    const bool isFloat = isTypeFloat(typeProxy);
    const bool isBool = typeProxy == qglslang::EbtBool;

    spv::Op opCode = spv::OpNop;

    // Figure out which opcode to use.
    switch (op) {
    case qglslang::EOpSubgroupElect:                   opCode = spv::OpGroupNonUniformElect; break;
    case qglslang::EOpSubgroupAll:                     opCode = spv::OpGroupNonUniformAll; break;
    case qglslang::EOpSubgroupAny:                     opCode = spv::OpGroupNonUniformAny; break;
    case qglslang::EOpSubgroupAllEqual:                opCode = spv::OpGroupNonUniformAllEqual; break;
    case qglslang::EOpSubgroupBroadcast:               opCode = spv::OpGroupNonUniformBroadcast; break;
    case qglslang::EOpSubgroupBroadcastFirst:          opCode = spv::OpGroupNonUniformBroadcastFirst; break;
    case qglslang::EOpSubgroupBallot:                  opCode = spv::OpGroupNonUniformBallot; break;
    case qglslang::EOpSubgroupInverseBallot:           opCode = spv::OpGroupNonUniformInverseBallot; break;
    case qglslang::EOpSubgroupBallotBitExtract:        opCode = spv::OpGroupNonUniformBallotBitExtract; break;
    case qglslang::EOpSubgroupBallotBitCount:
    case qglslang::EOpSubgroupBallotInclusiveBitCount:
    case qglslang::EOpSubgroupBallotExclusiveBitCount: opCode = spv::OpGroupNonUniformBallotBitCount; break;
    case qglslang::EOpSubgroupBallotFindLSB:           opCode = spv::OpGroupNonUniformBallotFindLSB; break;
    case qglslang::EOpSubgroupBallotFindMSB:           opCode = spv::OpGroupNonUniformBallotFindMSB; break;
    case qglslang::EOpSubgroupShuffle:                 opCode = spv::OpGroupNonUniformShuffle; break;
    case qglslang::EOpSubgroupShuffleXor:              opCode = spv::OpGroupNonUniformShuffleXor; break;
    case qglslang::EOpSubgroupShuffleUp:               opCode = spv::OpGroupNonUniformShuffleUp; break;
    case qglslang::EOpSubgroupShuffleDown:             opCode = spv::OpGroupNonUniformShuffleDown; break;
    case qglslang::EOpSubgroupAdd:
    case qglslang::EOpSubgroupInclusiveAdd:
    case qglslang::EOpSubgroupExclusiveAdd:
    case qglslang::EOpSubgroupClusteredAdd:
    case qglslang::EOpSubgroupPartitionedAdd:
    case qglslang::EOpSubgroupPartitionedInclusiveAdd:
    case qglslang::EOpSubgroupPartitionedExclusiveAdd:
        if (isFloat) {
            opCode = spv::OpGroupNonUniformFAdd;
        } else {
            opCode = spv::OpGroupNonUniformIAdd;
        }
        break;
    case qglslang::EOpSubgroupMul:
    case qglslang::EOpSubgroupInclusiveMul:
    case qglslang::EOpSubgroupExclusiveMul:
    case qglslang::EOpSubgroupClusteredMul:
    case qglslang::EOpSubgroupPartitionedMul:
    case qglslang::EOpSubgroupPartitionedInclusiveMul:
    case qglslang::EOpSubgroupPartitionedExclusiveMul:
        if (isFloat) {
            opCode = spv::OpGroupNonUniformFMul;
        } else {
            opCode = spv::OpGroupNonUniformIMul;
        }
        break;
    case qglslang::EOpSubgroupMin:
    case qglslang::EOpSubgroupInclusiveMin:
    case qglslang::EOpSubgroupExclusiveMin:
    case qglslang::EOpSubgroupClusteredMin:
    case qglslang::EOpSubgroupPartitionedMin:
    case qglslang::EOpSubgroupPartitionedInclusiveMin:
    case qglslang::EOpSubgroupPartitionedExclusiveMin:
        if (isFloat) {
            opCode = spv::OpGroupNonUniformFMin;
        } else if (isUnsigned) {
            opCode = spv::OpGroupNonUniformUMin;
        } else {
            opCode = spv::OpGroupNonUniformSMin;
        }
        break;
    case qglslang::EOpSubgroupMax:
    case qglslang::EOpSubgroupInclusiveMax:
    case qglslang::EOpSubgroupExclusiveMax:
    case qglslang::EOpSubgroupClusteredMax:
    case qglslang::EOpSubgroupPartitionedMax:
    case qglslang::EOpSubgroupPartitionedInclusiveMax:
    case qglslang::EOpSubgroupPartitionedExclusiveMax:
        if (isFloat) {
            opCode = spv::OpGroupNonUniformFMax;
        } else if (isUnsigned) {
            opCode = spv::OpGroupNonUniformUMax;
        } else {
            opCode = spv::OpGroupNonUniformSMax;
        }
        break;
    case qglslang::EOpSubgroupAnd:
    case qglslang::EOpSubgroupInclusiveAnd:
    case qglslang::EOpSubgroupExclusiveAnd:
    case qglslang::EOpSubgroupClusteredAnd:
    case qglslang::EOpSubgroupPartitionedAnd:
    case qglslang::EOpSubgroupPartitionedInclusiveAnd:
    case qglslang::EOpSubgroupPartitionedExclusiveAnd:
        if (isBool) {
            opCode = spv::OpGroupNonUniformLogicalAnd;
        } else {
            opCode = spv::OpGroupNonUniformBitwiseAnd;
        }
        break;
    case qglslang::EOpSubgroupOr:
    case qglslang::EOpSubgroupInclusiveOr:
    case qglslang::EOpSubgroupExclusiveOr:
    case qglslang::EOpSubgroupClusteredOr:
    case qglslang::EOpSubgroupPartitionedOr:
    case qglslang::EOpSubgroupPartitionedInclusiveOr:
    case qglslang::EOpSubgroupPartitionedExclusiveOr:
        if (isBool) {
            opCode = spv::OpGroupNonUniformLogicalOr;
        } else {
            opCode = spv::OpGroupNonUniformBitwiseOr;
        }
        break;
    case qglslang::EOpSubgroupXor:
    case qglslang::EOpSubgroupInclusiveXor:
    case qglslang::EOpSubgroupExclusiveXor:
    case qglslang::EOpSubgroupClusteredXor:
    case qglslang::EOpSubgroupPartitionedXor:
    case qglslang::EOpSubgroupPartitionedInclusiveXor:
    case qglslang::EOpSubgroupPartitionedExclusiveXor:
        if (isBool) {
            opCode = spv::OpGroupNonUniformLogicalXor;
        } else {
            opCode = spv::OpGroupNonUniformBitwiseXor;
        }
        break;
    case qglslang::EOpSubgroupQuadBroadcast:      opCode = spv::OpGroupNonUniformQuadBroadcast; break;
    case qglslang::EOpSubgroupQuadSwapHorizontal:
    case qglslang::EOpSubgroupQuadSwapVertical:
    case qglslang::EOpSubgroupQuadSwapDiagonal:   opCode = spv::OpGroupNonUniformQuadSwap; break;
    default: assert(0 && "Unhandled subgroup operation!");
    }

    // get the right Group Operation
    spv::GroupOperation groupOperation = spv::GroupOperationMax;
    switch (op) {
    default:
        break;
    case qglslang::EOpSubgroupBallotBitCount:
    case qglslang::EOpSubgroupAdd:
    case qglslang::EOpSubgroupMul:
    case qglslang::EOpSubgroupMin:
    case qglslang::EOpSubgroupMax:
    case qglslang::EOpSubgroupAnd:
    case qglslang::EOpSubgroupOr:
    case qglslang::EOpSubgroupXor:
        groupOperation = spv::GroupOperationReduce;
        break;
    case qglslang::EOpSubgroupBallotInclusiveBitCount:
    case qglslang::EOpSubgroupInclusiveAdd:
    case qglslang::EOpSubgroupInclusiveMul:
    case qglslang::EOpSubgroupInclusiveMin:
    case qglslang::EOpSubgroupInclusiveMax:
    case qglslang::EOpSubgroupInclusiveAnd:
    case qglslang::EOpSubgroupInclusiveOr:
    case qglslang::EOpSubgroupInclusiveXor:
        groupOperation = spv::GroupOperationInclusiveScan;
        break;
    case qglslang::EOpSubgroupBallotExclusiveBitCount:
    case qglslang::EOpSubgroupExclusiveAdd:
    case qglslang::EOpSubgroupExclusiveMul:
    case qglslang::EOpSubgroupExclusiveMin:
    case qglslang::EOpSubgroupExclusiveMax:
    case qglslang::EOpSubgroupExclusiveAnd:
    case qglslang::EOpSubgroupExclusiveOr:
    case qglslang::EOpSubgroupExclusiveXor:
        groupOperation = spv::GroupOperationExclusiveScan;
        break;
    case qglslang::EOpSubgroupClusteredAdd:
    case qglslang::EOpSubgroupClusteredMul:
    case qglslang::EOpSubgroupClusteredMin:
    case qglslang::EOpSubgroupClusteredMax:
    case qglslang::EOpSubgroupClusteredAnd:
    case qglslang::EOpSubgroupClusteredOr:
    case qglslang::EOpSubgroupClusteredXor:
        groupOperation = spv::GroupOperationClusteredReduce;
        break;
    case qglslang::EOpSubgroupPartitionedAdd:
    case qglslang::EOpSubgroupPartitionedMul:
    case qglslang::EOpSubgroupPartitionedMin:
    case qglslang::EOpSubgroupPartitionedMax:
    case qglslang::EOpSubgroupPartitionedAnd:
    case qglslang::EOpSubgroupPartitionedOr:
    case qglslang::EOpSubgroupPartitionedXor:
        groupOperation = spv::GroupOperationPartitionedReduceNV;
        break;
    case qglslang::EOpSubgroupPartitionedInclusiveAdd:
    case qglslang::EOpSubgroupPartitionedInclusiveMul:
    case qglslang::EOpSubgroupPartitionedInclusiveMin:
    case qglslang::EOpSubgroupPartitionedInclusiveMax:
    case qglslang::EOpSubgroupPartitionedInclusiveAnd:
    case qglslang::EOpSubgroupPartitionedInclusiveOr:
    case qglslang::EOpSubgroupPartitionedInclusiveXor:
        groupOperation = spv::GroupOperationPartitionedInclusiveScanNV;
        break;
    case qglslang::EOpSubgroupPartitionedExclusiveAdd:
    case qglslang::EOpSubgroupPartitionedExclusiveMul:
    case qglslang::EOpSubgroupPartitionedExclusiveMin:
    case qglslang::EOpSubgroupPartitionedExclusiveMax:
    case qglslang::EOpSubgroupPartitionedExclusiveAnd:
    case qglslang::EOpSubgroupPartitionedExclusiveOr:
    case qglslang::EOpSubgroupPartitionedExclusiveXor:
        groupOperation = spv::GroupOperationPartitionedExclusiveScanNV;
        break;
    }

    // build the instruction
    std::vector<spv::IdImmediate> spvGroupOperands;

    // Every operation begins with the Execution Scope operand.
    spv::IdImmediate executionScope = { true, builder.makeUintConstant(spv::ScopeSubgroup) };
    spvGroupOperands.push_back(executionScope);

    // Next, for all operations that use a Group Operation, push that as an operand.
    if (groupOperation != spv::GroupOperationMax) {
        spv::IdImmediate groupOperand = { false, (unsigned)groupOperation };
        spvGroupOperands.push_back(groupOperand);
    }

    // Push back the operands next.
    for (auto opIt = operands.cbegin(); opIt != operands.cend(); ++opIt) {
        spv::IdImmediate operand = { true, *opIt };
        spvGroupOperands.push_back(operand);
    }

    // Some opcodes have additional operands.
    spv::Id directionId = spv::NoResult;
    switch (op) {
    default: break;
    case qglslang::EOpSubgroupQuadSwapHorizontal: directionId = builder.makeUintConstant(0); break;
    case qglslang::EOpSubgroupQuadSwapVertical:   directionId = builder.makeUintConstant(1); break;
    case qglslang::EOpSubgroupQuadSwapDiagonal:   directionId = builder.makeUintConstant(2); break;
    }
    if (directionId != spv::NoResult) {
        spv::IdImmediate direction = { true, directionId };
        spvGroupOperands.push_back(direction);
    }

    return builder.createOp(opCode, typeId, spvGroupOperands);
}

spv::Id TGlslangToSpvTraverser::createMiscOperation(qglslang::TOperator op, spv::Decoration precision,
    spv::Id typeId, std::vector<spv::Id>& operands, qglslang::TBasicType typeProxy)
{
    bool isUnsigned = isTypeUnsignedInt(typeProxy);
    bool isFloat = isTypeFloat(typeProxy);

    spv::Op opCode = spv::OpNop;
    int extBuiltins = -1;
    int libCall = -1;
    size_t consumedOperands = operands.size();
    spv::Id typeId0 = 0;
    if (consumedOperands > 0)
        typeId0 = builder.getTypeId(operands[0]);
    spv::Id typeId1 = 0;
    if (consumedOperands > 1)
        typeId1 = builder.getTypeId(operands[1]);
    spv::Id frexpIntType = 0;

    switch (op) {
    case qglslang::EOpMin:
        if (isFloat)
            libCall = nanMinMaxClamp ? spv::GLSLstd450NMin : spv::GLSLstd450FMin;
        else if (isUnsigned)
            libCall = spv::GLSLstd450UMin;
        else
            libCall = spv::GLSLstd450SMin;
        builder.promoteScalar(precision, operands.front(), operands.back());
        break;
    case qglslang::EOpModf:
        libCall = spv::GLSLstd450Modf;
        break;
    case qglslang::EOpMax:
        if (isFloat)
            libCall = nanMinMaxClamp ? spv::GLSLstd450NMax : spv::GLSLstd450FMax;
        else if (isUnsigned)
            libCall = spv::GLSLstd450UMax;
        else
            libCall = spv::GLSLstd450SMax;
        builder.promoteScalar(precision, operands.front(), operands.back());
        break;
    case qglslang::EOpPow:
        libCall = spv::GLSLstd450Pow;
        break;
    case qglslang::EOpDot:
        opCode = spv::OpDot;
        break;
    case qglslang::EOpAtan:
        libCall = spv::GLSLstd450Atan2;
        break;

    case qglslang::EOpClamp:
        if (isFloat)
            libCall = nanMinMaxClamp ? spv::GLSLstd450NClamp : spv::GLSLstd450FClamp;
        else if (isUnsigned)
            libCall = spv::GLSLstd450UClamp;
        else
            libCall = spv::GLSLstd450SClamp;
        builder.promoteScalar(precision, operands.front(), operands[1]);
        builder.promoteScalar(precision, operands.front(), operands[2]);
        break;
    case qglslang::EOpMix:
        if (! builder.isBoolType(builder.getScalarTypeId(builder.getTypeId(operands.back())))) {
            assert(isFloat);
            libCall = spv::GLSLstd450FMix;
        } else {
            opCode = spv::OpSelect;
            std::swap(operands.front(), operands.back());
        }
        builder.promoteScalar(precision, operands.front(), operands.back());
        break;
    case qglslang::EOpStep:
        libCall = spv::GLSLstd450Step;
        builder.promoteScalar(precision, operands.front(), operands.back());
        break;
    case qglslang::EOpSmoothStep:
        libCall = spv::GLSLstd450SmoothStep;
        builder.promoteScalar(precision, operands[0], operands[2]);
        builder.promoteScalar(precision, operands[1], operands[2]);
        break;

    case qglslang::EOpDistance:
        libCall = spv::GLSLstd450Distance;
        break;
    case qglslang::EOpCross:
        libCall = spv::GLSLstd450Cross;
        break;
    case qglslang::EOpFaceForward:
        libCall = spv::GLSLstd450FaceForward;
        break;
    case qglslang::EOpReflect:
        libCall = spv::GLSLstd450Reflect;
        break;
    case qglslang::EOpRefract:
        libCall = spv::GLSLstd450Refract;
        break;
    case qglslang::EOpBarrier:
        {
            // This is for the extended controlBarrier function, with four operands.
            // The unextended barrier() goes through createNoArgOperation.
            assert(operands.size() == 4);
            unsigned int executionScope = builder.getConstantScalar(operands[0]);
            unsigned int memoryScope = builder.getConstantScalar(operands[1]);
            unsigned int semantics = builder.getConstantScalar(operands[2]) | builder.getConstantScalar(operands[3]);
            builder.createControlBarrier((spv::Scope)executionScope, (spv::Scope)memoryScope,
                (spv::MemorySemanticsMask)semantics);
            if (semantics & (spv::MemorySemanticsMakeAvailableKHRMask |
                             spv::MemorySemanticsMakeVisibleKHRMask |
                             spv::MemorySemanticsOutputMemoryKHRMask |
                             spv::MemorySemanticsVolatileMask)) {
                builder.addCapability(spv::CapabilityVulkanMemoryModelKHR);
            }
            if (glslangIntermediate->usingVulkanMemoryModel() && (executionScope == spv::ScopeDevice ||
                memoryScope == spv::ScopeDevice)) {
                builder.addCapability(spv::CapabilityVulkanMemoryModelDeviceScopeKHR);
            }
            return 0;
        }
        break;
    case qglslang::EOpMemoryBarrier:
        {
            // This is for the extended memoryBarrier function, with three operands.
            // The unextended memoryBarrier() goes through createNoArgOperation.
            assert(operands.size() == 3);
            unsigned int memoryScope = builder.getConstantScalar(operands[0]);
            unsigned int semantics = builder.getConstantScalar(operands[1]) | builder.getConstantScalar(operands[2]);
            builder.createMemoryBarrier((spv::Scope)memoryScope, (spv::MemorySemanticsMask)semantics);
            if (semantics & (spv::MemorySemanticsMakeAvailableKHRMask |
                             spv::MemorySemanticsMakeVisibleKHRMask |
                             spv::MemorySemanticsOutputMemoryKHRMask |
                             spv::MemorySemanticsVolatileMask)) {
                builder.addCapability(spv::CapabilityVulkanMemoryModelKHR);
            }
            if (glslangIntermediate->usingVulkanMemoryModel() && memoryScope == spv::ScopeDevice) {
                builder.addCapability(spv::CapabilityVulkanMemoryModelDeviceScopeKHR);
            }
            return 0;
        }
        break;

#ifndef GLSLANG_WEB
    case qglslang::EOpInterpolateAtSample:
        if (typeProxy == qglslang::EbtFloat16)
            builder.addExtension(spv::E_SPV_AMD_gpu_shader_half_float);
        libCall = spv::GLSLstd450InterpolateAtSample;
        break;
    case qglslang::EOpInterpolateAtOffset:
        if (typeProxy == qglslang::EbtFloat16)
            builder.addExtension(spv::E_SPV_AMD_gpu_shader_half_float);
        libCall = spv::GLSLstd450InterpolateAtOffset;
        break;
    case qglslang::EOpAddCarry:
        opCode = spv::OpIAddCarry;
        typeId = builder.makeStructResultType(typeId0, typeId0);
        consumedOperands = 2;
        break;
    case qglslang::EOpSubBorrow:
        opCode = spv::OpISubBorrow;
        typeId = builder.makeStructResultType(typeId0, typeId0);
        consumedOperands = 2;
        break;
    case qglslang::EOpUMulExtended:
        opCode = spv::OpUMulExtended;
        typeId = builder.makeStructResultType(typeId0, typeId0);
        consumedOperands = 2;
        break;
    case qglslang::EOpIMulExtended:
        opCode = spv::OpSMulExtended;
        typeId = builder.makeStructResultType(typeId0, typeId0);
        consumedOperands = 2;
        break;
    case qglslang::EOpBitfieldExtract:
        if (isUnsigned)
            opCode = spv::OpBitFieldUExtract;
        else
            opCode = spv::OpBitFieldSExtract;
        break;
    case qglslang::EOpBitfieldInsert:
        opCode = spv::OpBitFieldInsert;
        break;

    case qglslang::EOpFma:
        libCall = spv::GLSLstd450Fma;
        break;
    case qglslang::EOpFrexp:
        {
            libCall = spv::GLSLstd450FrexpStruct;
            assert(builder.isPointerType(typeId1));
            typeId1 = builder.getContainedTypeId(typeId1);
            int width = builder.getScalarTypeWidth(typeId1);
            if (width == 16)
                // Using 16-bit exp operand, enable extension SPV_AMD_gpu_shader_int16
                builder.addExtension(spv::E_SPV_AMD_gpu_shader_int16);
            if (builder.getNumComponents(operands[0]) == 1)
                frexpIntType = builder.makeIntegerType(width, true);
            else
                frexpIntType = builder.makeVectorType(builder.makeIntegerType(width, true),
                    builder.getNumComponents(operands[0]));
            typeId = builder.makeStructResultType(typeId0, frexpIntType);
            consumedOperands = 1;
        }
        break;
    case qglslang::EOpLdexp:
        libCall = spv::GLSLstd450Ldexp;
        break;

    case qglslang::EOpReadInvocation:
        return createInvocationsOperation(op, typeId, operands, typeProxy);

    case qglslang::EOpSubgroupBroadcast:
    case qglslang::EOpSubgroupBallotBitExtract:
    case qglslang::EOpSubgroupShuffle:
    case qglslang::EOpSubgroupShuffleXor:
    case qglslang::EOpSubgroupShuffleUp:
    case qglslang::EOpSubgroupShuffleDown:
    case qglslang::EOpSubgroupClusteredAdd:
    case qglslang::EOpSubgroupClusteredMul:
    case qglslang::EOpSubgroupClusteredMin:
    case qglslang::EOpSubgroupClusteredMax:
    case qglslang::EOpSubgroupClusteredAnd:
    case qglslang::EOpSubgroupClusteredOr:
    case qglslang::EOpSubgroupClusteredXor:
    case qglslang::EOpSubgroupQuadBroadcast:
    case qglslang::EOpSubgroupPartitionedAdd:
    case qglslang::EOpSubgroupPartitionedMul:
    case qglslang::EOpSubgroupPartitionedMin:
    case qglslang::EOpSubgroupPartitionedMax:
    case qglslang::EOpSubgroupPartitionedAnd:
    case qglslang::EOpSubgroupPartitionedOr:
    case qglslang::EOpSubgroupPartitionedXor:
    case qglslang::EOpSubgroupPartitionedInclusiveAdd:
    case qglslang::EOpSubgroupPartitionedInclusiveMul:
    case qglslang::EOpSubgroupPartitionedInclusiveMin:
    case qglslang::EOpSubgroupPartitionedInclusiveMax:
    case qglslang::EOpSubgroupPartitionedInclusiveAnd:
    case qglslang::EOpSubgroupPartitionedInclusiveOr:
    case qglslang::EOpSubgroupPartitionedInclusiveXor:
    case qglslang::EOpSubgroupPartitionedExclusiveAdd:
    case qglslang::EOpSubgroupPartitionedExclusiveMul:
    case qglslang::EOpSubgroupPartitionedExclusiveMin:
    case qglslang::EOpSubgroupPartitionedExclusiveMax:
    case qglslang::EOpSubgroupPartitionedExclusiveAnd:
    case qglslang::EOpSubgroupPartitionedExclusiveOr:
    case qglslang::EOpSubgroupPartitionedExclusiveXor:
        return createSubgroupOperation(op, typeId, operands, typeProxy);

    case qglslang::EOpSwizzleInvocations:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_shader_ballot);
        libCall = spv::SwizzleInvocationsAMD;
        break;
    case qglslang::EOpSwizzleInvocationsMasked:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_shader_ballot);
        libCall = spv::SwizzleInvocationsMaskedAMD;
        break;
    case qglslang::EOpWriteInvocation:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_shader_ballot);
        libCall = spv::WriteInvocationAMD;
        break;

    case qglslang::EOpMin3:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_shader_trinary_minmax);
        if (isFloat)
            libCall = spv::FMin3AMD;
        else {
            if (isUnsigned)
                libCall = spv::UMin3AMD;
            else
                libCall = spv::SMin3AMD;
        }
        break;
    case qglslang::EOpMax3:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_shader_trinary_minmax);
        if (isFloat)
            libCall = spv::FMax3AMD;
        else {
            if (isUnsigned)
                libCall = spv::UMax3AMD;
            else
                libCall = spv::SMax3AMD;
        }
        break;
    case qglslang::EOpMid3:
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_shader_trinary_minmax);
        if (isFloat)
            libCall = spv::FMid3AMD;
        else {
            if (isUnsigned)
                libCall = spv::UMid3AMD;
            else
                libCall = spv::SMid3AMD;
        }
        break;

    case qglslang::EOpInterpolateAtVertex:
        if (typeProxy == qglslang::EbtFloat16)
            builder.addExtension(spv::E_SPV_AMD_gpu_shader_half_float);
        extBuiltins = getExtBuiltins(spv::E_SPV_AMD_shader_explicit_vertex_parameter);
        libCall = spv::InterpolateAtVertexAMD;
        break;

    case qglslang::EOpReportIntersection:
        typeId = builder.makeBoolType();
        opCode = spv::OpReportIntersectionKHR;
        break;
    case qglslang::EOpTraceNV:
        builder.createNoResultOp(spv::OpTraceNV, operands);
        return 0;
    case qglslang::EOpTraceRayMotionNV:
        builder.addExtension(spv::E_SPV_NV_ray_tracing_motion_blur);
        builder.addCapability(spv::CapabilityRayTracingMotionBlurNV);
        builder.createNoResultOp(spv::OpTraceRayMotionNV, operands);
        return 0;
    case qglslang::EOpTraceKHR:
        builder.createNoResultOp(spv::OpTraceRayKHR, operands);
        return 0;
    case qglslang::EOpExecuteCallableNV:
        builder.createNoResultOp(spv::OpExecuteCallableNV, operands);
        return 0;
    case qglslang::EOpExecuteCallableKHR:
        builder.createNoResultOp(spv::OpExecuteCallableKHR, operands);
        return 0;

    case qglslang::EOpRayQueryInitialize:
        builder.createNoResultOp(spv::OpRayQueryInitializeKHR, operands);
        return 0;
    case qglslang::EOpRayQueryTerminate:
        builder.createNoResultOp(spv::OpRayQueryTerminateKHR, operands);
        return 0;
    case qglslang::EOpRayQueryGenerateIntersection:
        builder.createNoResultOp(spv::OpRayQueryGenerateIntersectionKHR, operands);
        return 0;
    case qglslang::EOpRayQueryConfirmIntersection:
        builder.createNoResultOp(spv::OpRayQueryConfirmIntersectionKHR, operands);
        return 0;
    case qglslang::EOpRayQueryProceed:
        typeId = builder.makeBoolType();
        opCode = spv::OpRayQueryProceedKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionType:
        typeId = builder.makeUintType(32);
        opCode = spv::OpRayQueryGetIntersectionTypeKHR;
        break;
    case qglslang::EOpRayQueryGetRayTMin:
        typeId = builder.makeFloatType(32);
        opCode = spv::OpRayQueryGetRayTMinKHR;
        break;
    case qglslang::EOpRayQueryGetRayFlags:
        typeId = builder.makeIntType(32);
        opCode = spv::OpRayQueryGetRayFlagsKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionT:
        typeId = builder.makeFloatType(32);
        opCode = spv::OpRayQueryGetIntersectionTKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionInstanceCustomIndex:
        typeId = builder.makeIntType(32);
        opCode = spv::OpRayQueryGetIntersectionInstanceCustomIndexKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionInstanceId:
        typeId = builder.makeIntType(32);
        opCode = spv::OpRayQueryGetIntersectionInstanceIdKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionInstanceShaderBindingTableRecordOffset:
        typeId = builder.makeUintType(32);
        opCode = spv::OpRayQueryGetIntersectionInstanceShaderBindingTableRecordOffsetKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionGeometryIndex:
        typeId = builder.makeIntType(32);
        opCode = spv::OpRayQueryGetIntersectionGeometryIndexKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionPrimitiveIndex:
        typeId = builder.makeIntType(32);
        opCode = spv::OpRayQueryGetIntersectionPrimitiveIndexKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionBarycentrics:
        typeId = builder.makeVectorType(builder.makeFloatType(32), 2);
        opCode = spv::OpRayQueryGetIntersectionBarycentricsKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionFrontFace:
        typeId = builder.makeBoolType();
        opCode = spv::OpRayQueryGetIntersectionFrontFaceKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionCandidateAABBOpaque:
        typeId = builder.makeBoolType();
        opCode = spv::OpRayQueryGetIntersectionCandidateAABBOpaqueKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionObjectRayDirection:
        typeId = builder.makeVectorType(builder.makeFloatType(32), 3);
        opCode = spv::OpRayQueryGetIntersectionObjectRayDirectionKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionObjectRayOrigin:
        typeId = builder.makeVectorType(builder.makeFloatType(32), 3);
        opCode = spv::OpRayQueryGetIntersectionObjectRayOriginKHR;
        break;
    case qglslang::EOpRayQueryGetWorldRayDirection:
        typeId = builder.makeVectorType(builder.makeFloatType(32), 3);
        opCode = spv::OpRayQueryGetWorldRayDirectionKHR;
        break;
    case qglslang::EOpRayQueryGetWorldRayOrigin:
        typeId = builder.makeVectorType(builder.makeFloatType(32), 3);
        opCode = spv::OpRayQueryGetWorldRayOriginKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionObjectToWorld:
        typeId = builder.makeMatrixType(builder.makeFloatType(32), 4, 3);
        opCode = spv::OpRayQueryGetIntersectionObjectToWorldKHR;
        break;
    case qglslang::EOpRayQueryGetIntersectionWorldToObject:
        typeId = builder.makeMatrixType(builder.makeFloatType(32), 4, 3);
        opCode = spv::OpRayQueryGetIntersectionWorldToObjectKHR;
        break;
    case qglslang::EOpWritePackedPrimitiveIndices4x8NV:
        builder.createNoResultOp(spv::OpWritePackedPrimitiveIndices4x8NV, operands);
        return 0;
    case qglslang::EOpCooperativeMatrixMulAdd:
        opCode = spv::OpCooperativeMatrixMulAddNV;
        break;
#endif // GLSLANG_WEB
    default:
        return 0;
    }

    spv::Id id = 0;
    if (libCall >= 0) {
        // Use an extended instruction from the standard library.
        // Construct the call arguments, without modifying the original operands vector.
        // We might need the remaining arguments, e.g. in the EOpFrexp case.
        std::vector<spv::Id> callArguments(operands.begin(), operands.begin() + consumedOperands);
        id = builder.createBuiltinCall(typeId, extBuiltins >= 0 ? extBuiltins : stdBuiltins, libCall, callArguments);
    } else if (opCode == spv::OpDot && !isFloat) {
        // int dot(int, int)
        // NOTE: never called for scalar/vector1, this is turned into simple mul before this can be reached
        const int componentCount = builder.getNumComponents(operands[0]);
        spv::Id mulOp = builder.createBinOp(spv::OpIMul, builder.getTypeId(operands[0]), operands[0], operands[1]);
        builder.setPrecision(mulOp, precision);
        id = builder.createCompositeExtract(mulOp, typeId, 0);
        for (int i = 1; i < componentCount; ++i) {
            builder.setPrecision(id, precision);
            id = builder.createBinOp(spv::OpIAdd, typeId, id, builder.createCompositeExtract(mulOp, typeId, i));
        }
    } else {
        switch (consumedOperands) {
        case 0:
            // should all be handled by visitAggregate and createNoArgOperation
            assert(0);
            return 0;
        case 1:
            // should all be handled by createUnaryOperation
            assert(0);
            return 0;
        case 2:
            id = builder.createBinOp(opCode, typeId, operands[0], operands[1]);
            break;
        default:
            // anything 3 or over doesn't have l-value operands, so all should be consumed
            assert(consumedOperands == operands.size());
            id = builder.createOp(opCode, typeId, operands);
            break;
        }
    }

#ifndef GLSLANG_WEB
    // Decode the return types that were structures
    switch (op) {
    case qglslang::EOpAddCarry:
    case qglslang::EOpSubBorrow:
        builder.createStore(builder.createCompositeExtract(id, typeId0, 1), operands[2]);
        id = builder.createCompositeExtract(id, typeId0, 0);
        break;
    case qglslang::EOpUMulExtended:
    case qglslang::EOpIMulExtended:
        builder.createStore(builder.createCompositeExtract(id, typeId0, 0), operands[3]);
        builder.createStore(builder.createCompositeExtract(id, typeId0, 1), operands[2]);
        break;
    case qglslang::EOpFrexp:
        {
            assert(operands.size() == 2);
            if (builder.isFloatType(builder.getScalarTypeId(typeId1))) {
                // "exp" is floating-point type (from HLSL intrinsic)
                spv::Id member1 = builder.createCompositeExtract(id, frexpIntType, 1);
                member1 = builder.createUnaryOp(spv::OpConvertSToF, typeId1, member1);
                builder.createStore(member1, operands[1]);
            } else
                // "exp" is integer type (from GLSL built-in function)
                builder.createStore(builder.createCompositeExtract(id, frexpIntType, 1), operands[1]);
            id = builder.createCompositeExtract(id, typeId0, 0);
        }
        break;
    default:
        break;
    }
#endif

    return builder.setPrecision(id, precision);
}

// Intrinsics with no arguments (or no return value, and no precision).
spv::Id TGlslangToSpvTraverser::createNoArgOperation(qglslang::TOperator op, spv::Decoration precision, spv::Id typeId)
{
    // GLSL memory barriers use queuefamily scope in new model, device scope in old model
    spv::Scope memoryBarrierScope = glslangIntermediate->usingVulkanMemoryModel() ?
        spv::ScopeQueueFamilyKHR : spv::ScopeDevice;

    switch (op) {
    case qglslang::EOpBarrier:
        if (glslangIntermediate->getStage() == EShLangTessControl) {
            if (glslangIntermediate->usingVulkanMemoryModel()) {
                builder.createControlBarrier(spv::ScopeWorkgroup, spv::ScopeWorkgroup,
                                             spv::MemorySemanticsOutputMemoryKHRMask |
                                             spv::MemorySemanticsAcquireReleaseMask);
                builder.addCapability(spv::CapabilityVulkanMemoryModelKHR);
            } else {
                builder.createControlBarrier(spv::ScopeWorkgroup, spv::ScopeInvocation, spv::MemorySemanticsMaskNone);
            }
        } else {
            builder.createControlBarrier(spv::ScopeWorkgroup, spv::ScopeWorkgroup,
                                            spv::MemorySemanticsWorkgroupMemoryMask |
                                            spv::MemorySemanticsAcquireReleaseMask);
        }
        return 0;
    case qglslang::EOpMemoryBarrier:
        builder.createMemoryBarrier(memoryBarrierScope, spv::MemorySemanticsAllMemory |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpMemoryBarrierBuffer:
        builder.createMemoryBarrier(memoryBarrierScope, spv::MemorySemanticsUniformMemoryMask |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpMemoryBarrierShared:
        builder.createMemoryBarrier(memoryBarrierScope, spv::MemorySemanticsWorkgroupMemoryMask |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpGroupMemoryBarrier:
        builder.createMemoryBarrier(spv::ScopeWorkgroup, spv::MemorySemanticsAllMemory |
                                                         spv::MemorySemanticsAcquireReleaseMask);
        return 0;
#ifndef GLSLANG_WEB
    case qglslang::EOpMemoryBarrierAtomicCounter:
        builder.createMemoryBarrier(memoryBarrierScope, spv::MemorySemanticsAtomicCounterMemoryMask |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpMemoryBarrierImage:
        builder.createMemoryBarrier(memoryBarrierScope, spv::MemorySemanticsImageMemoryMask |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpAllMemoryBarrierWithGroupSync:
        builder.createControlBarrier(spv::ScopeWorkgroup, spv::ScopeDevice,
                                        spv::MemorySemanticsAllMemory |
                                        spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpDeviceMemoryBarrier:
        builder.createMemoryBarrier(spv::ScopeDevice, spv::MemorySemanticsUniformMemoryMask |
                                                      spv::MemorySemanticsImageMemoryMask |
                                                      spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpDeviceMemoryBarrierWithGroupSync:
        builder.createControlBarrier(spv::ScopeWorkgroup, spv::ScopeDevice, spv::MemorySemanticsUniformMemoryMask |
                                                                            spv::MemorySemanticsImageMemoryMask |
                                                                            spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpWorkgroupMemoryBarrier:
        builder.createMemoryBarrier(spv::ScopeWorkgroup, spv::MemorySemanticsWorkgroupMemoryMask |
                                                         spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpWorkgroupMemoryBarrierWithGroupSync:
        builder.createControlBarrier(spv::ScopeWorkgroup, spv::ScopeWorkgroup,
                                        spv::MemorySemanticsWorkgroupMemoryMask |
                                        spv::MemorySemanticsAcquireReleaseMask);
        return 0;
    case qglslang::EOpSubgroupBarrier:
        builder.createControlBarrier(spv::ScopeSubgroup, spv::ScopeSubgroup, spv::MemorySemanticsAllMemory |
                                                                             spv::MemorySemanticsAcquireReleaseMask);
        return spv::NoResult;
    case qglslang::EOpSubgroupMemoryBarrier:
        builder.createMemoryBarrier(spv::ScopeSubgroup, spv::MemorySemanticsAllMemory |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return spv::NoResult;
    case qglslang::EOpSubgroupMemoryBarrierBuffer:
        builder.createMemoryBarrier(spv::ScopeSubgroup, spv::MemorySemanticsUniformMemoryMask |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return spv::NoResult;
    case qglslang::EOpSubgroupMemoryBarrierImage:
        builder.createMemoryBarrier(spv::ScopeSubgroup, spv::MemorySemanticsImageMemoryMask |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return spv::NoResult;
    case qglslang::EOpSubgroupMemoryBarrierShared:
        builder.createMemoryBarrier(spv::ScopeSubgroup, spv::MemorySemanticsWorkgroupMemoryMask |
                                                        spv::MemorySemanticsAcquireReleaseMask);
        return spv::NoResult;

    case qglslang::EOpEmitVertex:
        builder.createNoResultOp(spv::OpEmitVertex);
        return 0;
    case qglslang::EOpEndPrimitive:
        builder.createNoResultOp(spv::OpEndPrimitive);
        return 0;

    case qglslang::EOpSubgroupElect: {
        std::vector<spv::Id> operands;
        return createSubgroupOperation(op, typeId, operands, qglslang::EbtVoid);
    }
    case qglslang::EOpTime:
    {
        std::vector<spv::Id> args; // Dummy arguments
        spv::Id id = builder.createBuiltinCall(typeId, getExtBuiltins(spv::E_SPV_AMD_gcn_shader), spv::TimeAMD, args);
        return builder.setPrecision(id, precision);
    }
    case qglslang::EOpIgnoreIntersectionNV:
        builder.createNoResultOp(spv::OpIgnoreIntersectionNV);
        return 0;
    case qglslang::EOpTerminateRayNV:
        builder.createNoResultOp(spv::OpTerminateRayNV);
        return 0;
    case qglslang::EOpRayQueryInitialize:
        builder.createNoResultOp(spv::OpRayQueryInitializeKHR);
        return 0;
    case qglslang::EOpRayQueryTerminate:
        builder.createNoResultOp(spv::OpRayQueryTerminateKHR);
        return 0;
    case qglslang::EOpRayQueryGenerateIntersection:
        builder.createNoResultOp(spv::OpRayQueryGenerateIntersectionKHR);
        return 0;
    case qglslang::EOpRayQueryConfirmIntersection:
        builder.createNoResultOp(spv::OpRayQueryConfirmIntersectionKHR);
        return 0;
    case qglslang::EOpBeginInvocationInterlock:
        builder.createNoResultOp(spv::OpBeginInvocationInterlockEXT);
        return 0;
    case qglslang::EOpEndInvocationInterlock:
        builder.createNoResultOp(spv::OpEndInvocationInterlockEXT);
        return 0;

    case qglslang::EOpIsHelperInvocation:
    {
        std::vector<spv::Id> args; // Dummy arguments
        builder.addExtension(spv::E_SPV_EXT_demote_to_helper_invocation);
        builder.addCapability(spv::CapabilityDemoteToHelperInvocationEXT);
        return builder.createOp(spv::OpIsHelperInvocationEXT, typeId, args);
    }

    case qglslang::EOpReadClockSubgroupKHR: {
        std::vector<spv::Id> args;
        args.push_back(builder.makeUintConstant(spv::ScopeSubgroup));
        builder.addExtension(spv::E_SPV_KHR_shader_clock);
        builder.addCapability(spv::CapabilityShaderClockKHR);
        return builder.createOp(spv::OpReadClockKHR, typeId, args);
    }

    case qglslang::EOpReadClockDeviceKHR: {
        std::vector<spv::Id> args;
        args.push_back(builder.makeUintConstant(spv::ScopeDevice));
        builder.addExtension(spv::E_SPV_KHR_shader_clock);
        builder.addCapability(spv::CapabilityShaderClockKHR);
        return builder.createOp(spv::OpReadClockKHR, typeId, args);
    }
#endif
    default:
        break;
    }

    logger->missingFunctionality("unknown operation with no arguments");

    return 0;
}

spv::Id TGlslangToSpvTraverser::getSymbolId(const qglslang::TIntermSymbol* symbol)
{
    auto iter = symbolValues.find(symbol->getId());
    spv::Id id;
    if (symbolValues.end() != iter) {
        id = iter->second;
        return id;
    }

    // it was not found, create it
    spv::BuiltIn builtIn = TranslateBuiltInDecoration(symbol->getQualifier().builtIn, false);
    auto forcedType = getForcedType(symbol->getQualifier().builtIn, symbol->getType());

    // There are pairs of symbols that map to the same SPIR-V built-in:
    // gl_ObjectToWorldEXT and gl_ObjectToWorld3x4EXT, and gl_WorldToObjectEXT
    // and gl_WorldToObject3x4EXT. SPIR-V forbids having two OpVariables
    // with the same BuiltIn in the same storage class, so we must re-use one.
    const bool mayNeedToReuseBuiltIn =
        builtIn == spv::BuiltInObjectToWorldKHR ||
        builtIn == spv::BuiltInWorldToObjectKHR;

    if (mayNeedToReuseBuiltIn) {
        auto iter = builtInVariableIds.find(uint32_t(builtIn));
        if (builtInVariableIds.end() != iter) {
            id = iter->second;
            symbolValues[symbol->getId()] = id;
            if (forcedType.second != spv::NoType)
                forceType[id] = forcedType.second;
            return id;
        }
    }

    id = createSpvVariable(symbol, forcedType.first);

    if (mayNeedToReuseBuiltIn) {
        builtInVariableIds.insert({uint32_t(builtIn), id});
    }

    symbolValues[symbol->getId()] = id;
    if (forcedType.second != spv::NoType)
        forceType[id] = forcedType.second;

    if (symbol->getBasicType() != qglslang::EbtBlock) {
        builder.addDecoration(id, TranslatePrecisionDecoration(symbol->getType()));
        builder.addDecoration(id, TranslateInterpolationDecoration(symbol->getType().getQualifier()));
        builder.addDecoration(id, TranslateAuxiliaryStorageDecoration(symbol->getType().getQualifier()));
#ifndef GLSLANG_WEB
        addMeshNVDecoration(id, /*member*/ -1, symbol->getType().getQualifier());
        if (symbol->getQualifier().hasComponent())
            builder.addDecoration(id, spv::DecorationComponent, symbol->getQualifier().layoutComponent);
        if (symbol->getQualifier().hasIndex())
            builder.addDecoration(id, spv::DecorationIndex, symbol->getQualifier().layoutIndex);
#endif
        if (symbol->getType().getQualifier().hasSpecConstantId())
            builder.addDecoration(id, spv::DecorationSpecId, symbol->getType().getQualifier().layoutSpecConstantId);
        // atomic counters use this:
        if (symbol->getQualifier().hasOffset())
            builder.addDecoration(id, spv::DecorationOffset, symbol->getQualifier().layoutOffset);
    }

    if (symbol->getQualifier().hasLocation()) {
        if (!(glslangIntermediate->isRayTracingStage() && glslangIntermediate->IsRequestedExtension(qglslang::E_GL_EXT_ray_tracing)
              && (builder.getStorageClass(id) == spv::StorageClassRayPayloadKHR ||
                  builder.getStorageClass(id) == spv::StorageClassIncomingRayPayloadKHR ||
                  builder.getStorageClass(id) == spv::StorageClassCallableDataKHR ||
                  builder.getStorageClass(id) == spv::StorageClassIncomingCallableDataKHR))) {
            // Location values are used to link TraceRayKHR and ExecuteCallableKHR to corresponding variables
            // but are not valid in SPIRV since they are supported only for Input/Output Storage classes.
            builder.addDecoration(id, spv::DecorationLocation, symbol->getQualifier().layoutLocation);
        }
    }

    builder.addDecoration(id, TranslateInvariantDecoration(symbol->getType().getQualifier()));
    if (symbol->getQualifier().hasStream() && glslangIntermediate->isMultiStream()) {
        builder.addCapability(spv::CapabilityGeometryStreams);
        builder.addDecoration(id, spv::DecorationStream, symbol->getQualifier().layoutStream);
    }
    if (symbol->getQualifier().hasSet())
        builder.addDecoration(id, spv::DecorationDescriptorSet, symbol->getQualifier().layoutSet);
    else if (IsDescriptorResource(symbol->getType())) {
        // default to 0
        builder.addDecoration(id, spv::DecorationDescriptorSet, 0);
    }
    if (symbol->getQualifier().hasBinding())
        builder.addDecoration(id, spv::DecorationBinding, symbol->getQualifier().layoutBinding);
    else if (IsDescriptorResource(symbol->getType())) {
        // default to 0
        builder.addDecoration(id, spv::DecorationBinding, 0);
    }
    if (symbol->getQualifier().hasAttachment())
        builder.addDecoration(id, spv::DecorationInputAttachmentIndex, symbol->getQualifier().layoutAttachment);
    if (glslangIntermediate->getXfbMode()) {
        builder.addCapability(spv::CapabilityTransformFeedback);
        if (symbol->getQualifier().hasXfbBuffer()) {
            builder.addDecoration(id, spv::DecorationXfbBuffer, symbol->getQualifier().layoutXfbBuffer);
            unsigned stride = glslangIntermediate->getXfbStride(symbol->getQualifier().layoutXfbBuffer);
            if (stride != qglslang::TQualifier::layoutXfbStrideEnd)
                builder.addDecoration(id, spv::DecorationXfbStride, stride);
        }
        if (symbol->getQualifier().hasXfbOffset())
            builder.addDecoration(id, spv::DecorationOffset, symbol->getQualifier().layoutXfbOffset);
    }

    // add built-in variable decoration
    if (builtIn != spv::BuiltInMax) {
        // WorkgroupSize deprecated in spirv1.6
        if (glslangIntermediate->getSpv().spv < qglslang::EShTargetSpv_1_6 ||
            builtIn != spv::BuiltInWorkgroupSize)
            builder.addDecoration(id, spv::DecorationBuiltIn, (int)builtIn);
    }

    // Add volatile decoration to HelperInvocation for spirv1.6 and beyond
    if (builtIn == spv::BuiltInHelperInvocation &&
        glslangIntermediate->getSpv().spv >= qglslang::EShTargetSpv_1_6) {
        builder.addDecoration(id, spv::DecorationVolatile);
    }

#ifndef GLSLANG_WEB
    // Subgroup builtins which have input storage class are volatile for ray tracing stages.
    if (symbol->getType().isImage() || symbol->getQualifier().isPipeInput()) {
        std::vector<spv::Decoration> memory;
        TranslateMemoryDecoration(symbol->getType().getQualifier(), memory,
            glslangIntermediate->usingVulkanMemoryModel());
        for (unsigned int i = 0; i < memory.size(); ++i)
            builder.addDecoration(id, memory[i]);
    }

    if (builtIn == spv::BuiltInSampleMask) {
          spv::Decoration decoration;
          // GL_NV_sample_mask_override_coverage extension
          if (glslangIntermediate->getLayoutOverrideCoverage())
              decoration = (spv::Decoration)spv::DecorationOverrideCoverageNV;
          else
              decoration = (spv::Decoration)spv::DecorationMax;
        builder.addDecoration(id, decoration);
        if (decoration != spv::DecorationMax) {
            builder.addCapability(spv::CapabilitySampleMaskOverrideCoverageNV);
            builder.addExtension(spv::E_SPV_NV_sample_mask_override_coverage);
        }
    }
    else if (builtIn == spv::BuiltInLayer) {
        // SPV_NV_viewport_array2 extension
        if (symbol->getQualifier().layoutViewportRelative) {
            builder.addDecoration(id, (spv::Decoration)spv::DecorationViewportRelativeNV);
            builder.addCapability(spv::CapabilityShaderViewportMaskNV);
            builder.addExtension(spv::E_SPV_NV_viewport_array2);
        }
        if (symbol->getQualifier().layoutSecondaryViewportRelativeOffset != -2048) {
            builder.addDecoration(id, (spv::Decoration)spv::DecorationSecondaryViewportRelativeNV,
                                  symbol->getQualifier().layoutSecondaryViewportRelativeOffset);
            builder.addCapability(spv::CapabilityShaderStereoViewNV);
            builder.addExtension(spv::E_SPV_NV_stereo_view_rendering);
        }
    }

    if (symbol->getQualifier().layoutPassthrough) {
        builder.addDecoration(id, spv::DecorationPassthroughNV);
        builder.addCapability(spv::CapabilityGeometryShaderPassthroughNV);
        builder.addExtension(spv::E_SPV_NV_geometry_shader_passthrough);
    }
    if (symbol->getQualifier().pervertexNV) {
        builder.addDecoration(id, spv::DecorationPerVertexNV);
        builder.addCapability(spv::CapabilityFragmentBarycentricNV);
        builder.addExtension(spv::E_SPV_NV_fragment_shader_barycentric);
    }

    if (symbol->getQualifier().pervertexEXT) {
        builder.addDecoration(id, spv::DecorationPerVertexKHR);
        builder.addCapability(spv::CapabilityFragmentBarycentricKHR);
        builder.addExtension(spv::E_SPV_KHR_fragment_shader_barycentric);
    }

    if (glslangIntermediate->getHlslFunctionality1() && symbol->getType().getQualifier().semanticName != nullptr) {
        builder.addExtension("SPV_GOOGLE_hlsl_functionality1");
        builder.addDecoration(id, (spv::Decoration)spv::DecorationHlslSemanticGOOGLE,
                              symbol->getType().getQualifier().semanticName);
    }

    if (symbol->isReference()) {
        builder.addDecoration(id, symbol->getType().getQualifier().restrict ?
            spv::DecorationRestrictPointerEXT : spv::DecorationAliasedPointerEXT);
    }

    //
    // Add SPIR-V decorations for structure (GL_EXT_spirv_intrinsics)
    //
    if (symbol->getType().getQualifier().hasSprivDecorate()) {
        const qglslang::TSpirvDecorate& spirvDecorate = symbol->getType().getQualifier().getSpirvDecorate();

        // Add spirv_decorate
        for (auto& decorate : spirvDecorate.decorates) {
            if (!decorate.second.empty()) {
                std::vector<unsigned> literals;
                TranslateLiterals(decorate.second, literals);
                builder.addDecoration(id, static_cast<spv::Decoration>(decorate.first), literals);
            }
            else
                builder.addDecoration(id, static_cast<spv::Decoration>(decorate.first));
        }

        // Add spirv_decorate_id
        for (auto& decorateId : spirvDecorate.decorateIds) {
            std::vector<spv::Id> operandIds;
            assert(!decorateId.second.empty());
            for (auto extraOperand : decorateId.second) {
                if (extraOperand->getQualifier().isSpecConstant())
                    operandIds.push_back(getSymbolId(extraOperand->getAsSymbolNode()));
                else
                    operandIds.push_back(createSpvConstant(*extraOperand));
            }
            builder.addDecorationId(id, static_cast<spv::Decoration>(decorateId.first), operandIds);
        }

        // Add spirv_decorate_string
        for (auto& decorateString : spirvDecorate.decorateStrings) {
            std::vector<const char*> strings;
            assert(!decorateString.second.empty());
            for (auto extraOperand : decorateString.second) {
                const char* string = extraOperand->getConstArray()[0].getSConst()->c_str();
                strings.push_back(string);
            }
            builder.addDecoration(id, static_cast<spv::Decoration>(decorateString.first), strings);
        }
    }
#endif

    return id;
}

#ifndef GLSLANG_WEB
// add per-primitive, per-view. per-task decorations to a struct member (member >= 0) or an object
void TGlslangToSpvTraverser::addMeshNVDecoration(spv::Id id, int member, const qglslang::TQualifier& qualifier)
{
    if (member >= 0) {
        if (qualifier.perPrimitiveNV) {
            // Need to add capability/extension for fragment shader.
            // Mesh shader already adds this by default.
            if (glslangIntermediate->getStage() == EShLangFragment) {
                builder.addCapability(spv::CapabilityMeshShadingNV);
                builder.addExtension(spv::E_SPV_NV_mesh_shader);
            }
            builder.addMemberDecoration(id, (unsigned)member, spv::DecorationPerPrimitiveNV);
        }
        if (qualifier.perViewNV)
            builder.addMemberDecoration(id, (unsigned)member, spv::DecorationPerViewNV);
        if (qualifier.perTaskNV)
            builder.addMemberDecoration(id, (unsigned)member, spv::DecorationPerTaskNV);
    } else {
        if (qualifier.perPrimitiveNV) {
            // Need to add capability/extension for fragment shader.
            // Mesh shader already adds this by default.
            if (glslangIntermediate->getStage() == EShLangFragment) {
                builder.addCapability(spv::CapabilityMeshShadingNV);
                builder.addExtension(spv::E_SPV_NV_mesh_shader);
            }
            builder.addDecoration(id, spv::DecorationPerPrimitiveNV);
        }
        if (qualifier.perViewNV)
            builder.addDecoration(id, spv::DecorationPerViewNV);
        if (qualifier.perTaskNV)
            builder.addDecoration(id, spv::DecorationPerTaskNV);
    }
}
#endif

// Make a full tree of instructions to build a SPIR-V specialization constant,
// or regular constant if possible.
//
// TBD: this is not yet done, nor verified to be the best design, it does do the leaf symbols though
//
// Recursively walk the nodes.  The nodes form a tree whose leaves are
// regular constants, which themselves are trees that createSpvConstant()
// recursively walks.  So, this function walks the "top" of the tree:
//  - emit specialization constant-building instructions for specConstant
//  - when running into a non-spec-constant, switch to createSpvConstant()
spv::Id TGlslangToSpvTraverser::createSpvConstant(const qglslang::TIntermTyped& node)
{
    assert(node.getQualifier().isConstant());

    // Handle front-end constants first (non-specialization constants).
    if (! node.getQualifier().specConstant) {
        // hand off to the non-spec-constant path
        assert(node.getAsConstantUnion() != nullptr || node.getAsSymbolNode() != nullptr);
        int nextConst = 0;
        return createSpvConstantFromConstUnionArray(node.getType(), node.getAsConstantUnion() ?
            node.getAsConstantUnion()->getConstArray() : node.getAsSymbolNode()->getConstArray(),
            nextConst, false);
    }

    // We now know we have a specialization constant to build

    // Extra capabilities may be needed.
    if (node.getType().contains8BitInt())
        builder.addCapability(spv::CapabilityInt8);
    if (node.getType().contains16BitFloat())
        builder.addCapability(spv::CapabilityFloat16);
    if (node.getType().contains16BitInt())
        builder.addCapability(spv::CapabilityInt16);
    if (node.getType().contains64BitInt())
        builder.addCapability(spv::CapabilityInt64);
    if (node.getType().containsDouble())
        builder.addCapability(spv::CapabilityFloat64);

    // gl_WorkGroupSize is a special case until the front-end handles hierarchical specialization constants,
    // even then, it's specialization ids are handled by special case syntax in GLSL: layout(local_size_x = ...
    if (node.getType().getQualifier().builtIn == qglslang::EbvWorkGroupSize) {
        std::vector<spv::Id> dimConstId;
        for (int dim = 0; dim < 3; ++dim) {
            bool specConst = (glslangIntermediate->getLocalSizeSpecId(dim) != qglslang::TQualifier::layoutNotSet);
            dimConstId.push_back(builder.makeUintConstant(glslangIntermediate->getLocalSize(dim), specConst));
            if (specConst) {
                builder.addDecoration(dimConstId.back(), spv::DecorationSpecId,
                                      glslangIntermediate->getLocalSizeSpecId(dim));
            }
        }
        return builder.makeCompositeConstant(builder.makeVectorType(builder.makeUintType(32), 3), dimConstId, true);
    }

    // An AST node labelled as specialization constant should be a symbol node.
    // Its initializer should either be a sub tree with constant nodes, or a constant union array.
    if (auto* sn = node.getAsSymbolNode()) {
        spv::Id result;
        if (auto* sub_tree = sn->getConstSubtree()) {
            // Traverse the constant constructor sub tree like generating normal run-time instructions.
            // During the AST traversal, if the node is marked as 'specConstant', SpecConstantOpModeGuard
            // will set the builder into spec constant op instruction generating mode.
            sub_tree->traverse(this);
            result = accessChainLoad(sub_tree->getType());
        } else if (auto* const_union_array = &sn->getConstArray()) {
            int nextConst = 0;
            result = createSpvConstantFromConstUnionArray(sn->getType(), *const_union_array, nextConst, true);
        } else {
            logger->missingFunctionality("Invalid initializer for spec onstant.");
            return spv::NoResult;
        }
        builder.addName(result, sn->getName().c_str());
        return result;
    }

    // Neither a front-end constant node, nor a specialization constant node with constant union array or
    // constant sub tree as initializer.
    logger->missingFunctionality("Neither a front-end constant nor a spec constant.");
    return spv::NoResult;
}

// Use 'consts' as the flattened glslang source of scalar constants to recursively
// build the aggregate SPIR-V constant.
//
// If there are not enough elements present in 'consts', 0 will be substituted;
// an empty 'consts' can be used to create a fully zeroed SPIR-V constant.
//
spv::Id TGlslangToSpvTraverser::createSpvConstantFromConstUnionArray(const qglslang::TType& glslangType,
    const qglslang::TConstUnionArray& consts, int& nextConst, bool specConstant)
{
    // vector of constants for SPIR-V
    std::vector<spv::Id> spvConsts;

    // Type is used for struct and array constants
    spv::Id typeId = convertGlslangToSpvType(glslangType);

    if (glslangType.isArray()) {
        qglslang::TType elementType(glslangType, 0);
        for (int i = 0; i < glslangType.getOuterArraySize(); ++i)
            spvConsts.push_back(createSpvConstantFromConstUnionArray(elementType, consts, nextConst, false));
    } else if (glslangType.isMatrix()) {
        qglslang::TType vectorType(glslangType, 0);
        for (int col = 0; col < glslangType.getMatrixCols(); ++col)
            spvConsts.push_back(createSpvConstantFromConstUnionArray(vectorType, consts, nextConst, false));
    } else if (glslangType.isCoopMat()) {
        qglslang::TType componentType(glslangType.getBasicType());
        spvConsts.push_back(createSpvConstantFromConstUnionArray(componentType, consts, nextConst, false));
    } else if (glslangType.isStruct()) {
        qglslang::TVector<qglslang::TTypeLoc>::const_iterator iter;
        for (iter = glslangType.getStruct()->begin(); iter != glslangType.getStruct()->end(); ++iter)
            spvConsts.push_back(createSpvConstantFromConstUnionArray(*iter->type, consts, nextConst, false));
    } else if (glslangType.getVectorSize() > 1) {
        for (unsigned int i = 0; i < (unsigned int)glslangType.getVectorSize(); ++i) {
            bool zero = nextConst >= consts.size();
            switch (glslangType.getBasicType()) {
            case qglslang::EbtInt:
                spvConsts.push_back(builder.makeIntConstant(zero ? 0 : consts[nextConst].getIConst()));
                break;
            case qglslang::EbtUint:
                spvConsts.push_back(builder.makeUintConstant(zero ? 0 : consts[nextConst].getUConst()));
                break;
            case qglslang::EbtFloat:
                spvConsts.push_back(builder.makeFloatConstant(zero ? 0.0F : (float)consts[nextConst].getDConst()));
                break;
            case qglslang::EbtBool:
                spvConsts.push_back(builder.makeBoolConstant(zero ? false : consts[nextConst].getBConst()));
                break;
#ifndef GLSLANG_WEB
            case qglslang::EbtInt8:
                builder.addCapability(spv::CapabilityInt8);
                spvConsts.push_back(builder.makeInt8Constant(zero ? 0 : consts[nextConst].getI8Const()));
                break;
            case qglslang::EbtUint8:
                builder.addCapability(spv::CapabilityInt8);
                spvConsts.push_back(builder.makeUint8Constant(zero ? 0 : consts[nextConst].getU8Const()));
                break;
            case qglslang::EbtInt16:
                builder.addCapability(spv::CapabilityInt16);
                spvConsts.push_back(builder.makeInt16Constant(zero ? 0 : consts[nextConst].getI16Const()));
                break;
            case qglslang::EbtUint16:
                builder.addCapability(spv::CapabilityInt16);
                spvConsts.push_back(builder.makeUint16Constant(zero ? 0 : consts[nextConst].getU16Const()));
                break;
            case qglslang::EbtInt64:
                spvConsts.push_back(builder.makeInt64Constant(zero ? 0 : consts[nextConst].getI64Const()));
                break;
            case qglslang::EbtUint64:
                spvConsts.push_back(builder.makeUint64Constant(zero ? 0 : consts[nextConst].getU64Const()));
                break;
            case qglslang::EbtDouble:
                spvConsts.push_back(builder.makeDoubleConstant(zero ? 0.0 : consts[nextConst].getDConst()));
                break;
            case qglslang::EbtFloat16:
                builder.addCapability(spv::CapabilityFloat16);
                spvConsts.push_back(builder.makeFloat16Constant(zero ? 0.0F : (float)consts[nextConst].getDConst()));
                break;
#endif
            default:
                assert(0);
                break;
            }
            ++nextConst;
        }
    } else {
        // we have a non-aggregate (scalar) constant
        bool zero = nextConst >= consts.size();
        spv::Id scalar = 0;
        switch (glslangType.getBasicType()) {
        case qglslang::EbtInt:
            scalar = builder.makeIntConstant(zero ? 0 : consts[nextConst].getIConst(), specConstant);
            break;
        case qglslang::EbtUint:
            scalar = builder.makeUintConstant(zero ? 0 : consts[nextConst].getUConst(), specConstant);
            break;
        case qglslang::EbtFloat:
            scalar = builder.makeFloatConstant(zero ? 0.0F : (float)consts[nextConst].getDConst(), specConstant);
            break;
        case qglslang::EbtBool:
            scalar = builder.makeBoolConstant(zero ? false : consts[nextConst].getBConst(), specConstant);
            break;
#ifndef GLSLANG_WEB
        case qglslang::EbtInt8:
            builder.addCapability(spv::CapabilityInt8);
            scalar = builder.makeInt8Constant(zero ? 0 : consts[nextConst].getI8Const(), specConstant);
            break;
        case qglslang::EbtUint8:
            builder.addCapability(spv::CapabilityInt8);
            scalar = builder.makeUint8Constant(zero ? 0 : consts[nextConst].getU8Const(), specConstant);
            break;
        case qglslang::EbtInt16:
            builder.addCapability(spv::CapabilityInt16);
            scalar = builder.makeInt16Constant(zero ? 0 : consts[nextConst].getI16Const(), specConstant);
            break;
        case qglslang::EbtUint16:
            builder.addCapability(spv::CapabilityInt16);
            scalar = builder.makeUint16Constant(zero ? 0 : consts[nextConst].getU16Const(), specConstant);
            break;
        case qglslang::EbtInt64:
            scalar = builder.makeInt64Constant(zero ? 0 : consts[nextConst].getI64Const(), specConstant);
            break;
        case qglslang::EbtUint64:
            scalar = builder.makeUint64Constant(zero ? 0 : consts[nextConst].getU64Const(), specConstant);
            break;
        case qglslang::EbtDouble:
            scalar = builder.makeDoubleConstant(zero ? 0.0 : consts[nextConst].getDConst(), specConstant);
            break;
        case qglslang::EbtFloat16:
            builder.addCapability(spv::CapabilityFloat16);
            scalar = builder.makeFloat16Constant(zero ? 0.0F : (float)consts[nextConst].getDConst(), specConstant);
            break;
        case qglslang::EbtReference:
            scalar = builder.makeUint64Constant(zero ? 0 : consts[nextConst].getU64Const(), specConstant);
            scalar = builder.createUnaryOp(spv::OpBitcast, typeId, scalar);
            break;
#endif
        case qglslang::EbtString:
            scalar = builder.getStringId(consts[nextConst].getSConst()->c_str());
            break;
        default:
            assert(0);
            break;
        }
        ++nextConst;
        return scalar;
    }

    return builder.makeCompositeConstant(typeId, spvConsts);
}

// Return true if the node is a constant or symbol whose reading has no
// non-trivial observable cost or effect.
bool TGlslangToSpvTraverser::isTrivialLeaf(const qglslang::TIntermTyped* node)
{
    // don't know what this is
    if (node == nullptr)
        return false;

    // a constant is safe
    if (node->getAsConstantUnion() != nullptr)
        return true;

    // not a symbol means non-trivial
    if (node->getAsSymbolNode() == nullptr)
        return false;

    // a symbol, depends on what's being read
    switch (node->getType().getQualifier().storage) {
    case qglslang::EvqTemporary:
    case qglslang::EvqGlobal:
    case qglslang::EvqIn:
    case qglslang::EvqInOut:
    case qglslang::EvqConst:
    case qglslang::EvqConstReadOnly:
    case qglslang::EvqUniform:
        return true;
    default:
        return false;
    }
}

// A node is trivial if it is a single operation with no side effects.
// HLSL (and/or vectors) are always trivial, as it does not short circuit.
// Otherwise, error on the side of saying non-trivial.
// Return true if trivial.
bool TGlslangToSpvTraverser::isTrivial(const qglslang::TIntermTyped* node)
{
    if (node == nullptr)
        return false;

    // count non scalars as trivial, as well as anything coming from HLSL
    if (! node->getType().isScalarOrVec1() || glslangIntermediate->getSource() == qglslang::EShSourceHlsl)
        return true;

    // symbols and constants are trivial
    if (isTrivialLeaf(node))
        return true;

    // otherwise, it needs to be a simple operation or one or two leaf nodes

    // not a simple operation
    const qglslang::TIntermBinary* binaryNode = node->getAsBinaryNode();
    const qglslang::TIntermUnary* unaryNode = node->getAsUnaryNode();
    if (binaryNode == nullptr && unaryNode == nullptr)
        return false;

    // not on leaf nodes
    if (binaryNode && (! isTrivialLeaf(binaryNode->getLeft()) || ! isTrivialLeaf(binaryNode->getRight())))
        return false;

    if (unaryNode && ! isTrivialLeaf(unaryNode->getOperand())) {
        return false;
    }

    switch (node->getAsOperator()->getOp()) {
    case qglslang::EOpLogicalNot:
    case qglslang::EOpConvIntToBool:
    case qglslang::EOpConvUintToBool:
    case qglslang::EOpConvFloatToBool:
    case qglslang::EOpConvDoubleToBool:
    case qglslang::EOpEqual:
    case qglslang::EOpNotEqual:
    case qglslang::EOpLessThan:
    case qglslang::EOpGreaterThan:
    case qglslang::EOpLessThanEqual:
    case qglslang::EOpGreaterThanEqual:
    case qglslang::EOpIndexDirect:
    case qglslang::EOpIndexDirectStruct:
    case qglslang::EOpLogicalXor:
    case qglslang::EOpAny:
    case qglslang::EOpAll:
        return true;
    default:
        return false;
    }
}

// Emit short-circuiting code, where 'right' is never evaluated unless
// the left side is true (for &&) or false (for ||).
spv::Id TGlslangToSpvTraverser::createShortCircuit(qglslang::TOperator op, qglslang::TIntermTyped& left,
    qglslang::TIntermTyped& right)
{
    spv::Id boolTypeId = builder.makeBoolType();

    // emit left operand
    builder.clearAccessChain();
    left.traverse(this);
    spv::Id leftId = accessChainLoad(left.getType());

    // Operands to accumulate OpPhi operands
    std::vector<spv::Id> phiOperands;
    // accumulate left operand's phi information
    phiOperands.push_back(leftId);
    phiOperands.push_back(builder.getBuildPoint()->getId());

    // Make the two kinds of operation symmetric with a "!"
    //   || => emit "if (! left) result = right"
    //   && => emit "if (  left) result = right"
    //
    // TODO: this runtime "not" for || could be avoided by adding functionality
    // to 'builder' to have an "else" without an "then"
    if (op == qglslang::EOpLogicalOr)
        leftId = builder.createUnaryOp(spv::OpLogicalNot, boolTypeId, leftId);

    // make an "if" based on the left value
    spv::Builder::If ifBuilder(leftId, spv::SelectionControlMaskNone, builder);

    // emit right operand as the "then" part of the "if"
    builder.clearAccessChain();
    right.traverse(this);
    spv::Id rightId = accessChainLoad(right.getType());

    // accumulate left operand's phi information
    phiOperands.push_back(rightId);
    phiOperands.push_back(builder.getBuildPoint()->getId());

    // finish the "if"
    ifBuilder.makeEndIf();

    // phi together the two results
    return builder.createOp(spv::OpPhi, boolTypeId, phiOperands);
}

#ifndef GLSLANG_WEB
// Return type Id of the imported set of extended instructions corresponds to the name.
// Import this set if it has not been imported yet.
spv::Id TGlslangToSpvTraverser::getExtBuiltins(const char* name)
{
    if (extBuiltinMap.find(name) != extBuiltinMap.end())
        return extBuiltinMap[name];
    else {
        builder.addExtension(name);
        spv::Id extBuiltins = builder.import(name);
        extBuiltinMap[name] = extBuiltins;
        return extBuiltins;
    }
}
#endif

};  // end anonymous namespace

namespace qglslang {

void GetSpirvVersion(std::string& version)
{
    const int bufSize = 100;
    char buf[bufSize];
    snprintf(buf, bufSize, "0x%08x, Revision %d", spv::Version, spv::Revision);
    version = buf;
}

// For low-order part of the generator's magic number. Bump up
// when there is a change in the style (e.g., if SSA form changes,
// or a different instruction sequence to do something gets used).
int GetSpirvGeneratorVersion()
{
    // return 1; // start
    // return 2; // EOpAtomicCounterDecrement gets a post decrement, to map between GLSL -> SPIR-V
    // return 3; // change/correct barrier-instruction operands, to match memory model group decisions
    // return 4; // some deeper access chains: for dynamic vector component, and local Boolean component
    // return 5; // make OpArrayLength result type be an int with signedness of 0
    // return 6; // revert version 5 change, which makes a different (new) kind of incorrect code,
                 // versions 4 and 6 each generate OpArrayLength as it has long been done
    // return 7; // GLSL volatile keyword maps to both SPIR-V decorations Volatile and Coherent
    // return 8; // switch to new dead block eliminator; use OpUnreachable
    // return 9; // don't include opaque function parameters in OpEntryPoint global's operand list
    return 10; // Generate OpFUnordNotEqual for != comparisons
}

// Write SPIR-V out to a binary file
void OutputSpvBin(const std::vector<unsigned int>& spirv, const char* baseName)
{
    std::ofstream out;
    out.open(baseName, std::ios::binary | std::ios::out);
    if (out.fail())
        printf("ERROR: Failed to open file: %s\n", baseName);
    for (int i = 0; i < (int)spirv.size(); ++i) {
        unsigned int word = spirv[i];
        out.write((const char*)&word, 4);
    }
    out.close();
}

// Write SPIR-V out to a text file with 32-bit hexadecimal words
void OutputSpvHex(const std::vector<unsigned int>& spirv, const char* baseName, const char* varName)
{
#if !defined(GLSLANG_WEB) && !defined(GLSLANG_ANGLE)
    std::ofstream out;
    out.open(baseName, std::ios::binary | std::ios::out);
    if (out.fail())
        printf("ERROR: Failed to open file: %s\n", baseName);
    out << "\t// " <<
        GetSpirvGeneratorVersion() <<
        GLSLANG_VERSION_MAJOR << "." << GLSLANG_VERSION_MINOR << "." << GLSLANG_VERSION_PATCH <<
        GLSLANG_VERSION_FLAVOR << std::endl;
    if (varName != nullptr) {
        out << "\t #pragma once" << std::endl;
        out << "const uint32_t " << varName << "[] = {" << std::endl;
    }
    const int WORDS_PER_LINE = 8;
    for (int i = 0; i < (int)spirv.size(); i += WORDS_PER_LINE) {
        out << "\t";
        for (int j = 0; j < WORDS_PER_LINE && i + j < (int)spirv.size(); ++j) {
            const unsigned int word = spirv[i + j];
            out << "0x" << std::hex << std::setw(8) << std::setfill('0') << word;
            if (i + j + 1 < (int)spirv.size()) {
                out << ",";
            }
        }
        out << std::endl;
    }
    if (varName != nullptr) {
        out << "};";
        out << std::endl;
    }
    out.close();
#endif
}

//
// Set up the glslang traversal
//
void GlslangToSpv(const TIntermediate& intermediate, std::vector<unsigned int>& spirv, SpvOptions* options)
{
    spv::SpvBuildLogger logger;
    GlslangToSpv(intermediate, spirv, &logger, options);
}

void GlslangToSpv(const TIntermediate& intermediate, std::vector<unsigned int>& spirv,
                  spv::SpvBuildLogger* logger, SpvOptions* options)
{
    TIntermNode* root = intermediate.getTreeRoot();

    if (root == 0)
        return;

    SpvOptions defaultOptions;
    if (options == nullptr)
        options = &defaultOptions;

    GetThreadPoolAllocator().push();

    TGlslangToSpvTraverser it(intermediate.getSpv().spv, &intermediate, logger, *options);
    root->traverse(&it);
    it.finishSpv();
    it.dumpSpv(spirv);

#if ENABLE_OPT
    // If from HLSL, run spirv-opt to "legalize" the SPIR-V for Vulkan
    // eg. forward and remove memory writes of opaque types.
    bool prelegalization = intermediate.getSource() == EShSourceHlsl;
    if ((prelegalization || options->optimizeSize) && !options->disableOptimizer) {
        SpirvToolsTransform(intermediate, spirv, logger, options);
        prelegalization = false;
    }
    else if (options->stripDebugInfo) {
        // Strip debug info even if optimization is disabled.
        SpirvToolsStripDebugInfo(intermediate, spirv, logger);
    }

    if (options->validate)
        SpirvToolsValidate(intermediate, spirv, logger, prelegalization);

    if (options->disassemble)
        SpirvToolsDisassemble(std::cout, spirv);

#endif

    GetThreadPoolAllocator().pop();
}

}; // end namespace qglslang
