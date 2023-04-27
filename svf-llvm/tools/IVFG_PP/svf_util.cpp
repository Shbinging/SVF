//
// Created by 水兵 on 2023/4/28.
//
#include "svf_util.h"

bool var_has_val(const SVFVar* var) {
    assert(var != nullptr);
    if (var->getNodeKind() == SVFVar::PNODEK::DummyObjNode ||
        var->getNodeKind() == SVFVar::PNODEK::DummyObjNode) {
        return false;
    }
    if (SymbolTableInfo::isBlkObj(var->getId())) {
        assert(0);
        return false;
    }
    if (SymbolTableInfo::isConstantObj(var->getId())) {
        assert(0);
        return false;
    }
    return var->hasValue();
}

#define NodeK(x) \
    { SVFGNode::VFGNodeK::x, #x }

std::map<SVFGNode::VFGNodeK, std::string> nodeType2Str = {
    NodeK(Addr),   NodeK(Copy),      NodeK(Gep),       NodeK(Store),
    NodeK(Load),   NodeK(Cmp),       NodeK(BinaryOp),  NodeK(UnaryOp),
    NodeK(Branch), NodeK(TPhi),      NodeK(TIntraPhi), NodeK(TInterPhi),
    NodeK(MPhi),   NodeK(MIntraPhi), NodeK(MInterPhi), NodeK(FRet),
    NodeK(ARet),   NodeK(AParm),     NodeK(FParm),     NodeK(FunRet),
    NodeK(APIN),   NodeK(APOUT),     NodeK(FPIN),      NodeK(FPOUT),
    NodeK(NPtr),   NodeK(DummyVProp)};

const PAGNode* getLHSTopLevPtr(const VFGNode* node) {
    if (const AddrVFGNode* addr = SVFUtil::dyn_cast<AddrVFGNode>(node))
        return addr->getPAGDstNode();
    else if (const CopyVFGNode* copy = SVFUtil::dyn_cast<CopyVFGNode>(node))
        return copy->getPAGDstNode();
    else if (const GepVFGNode* gep = SVFUtil::dyn_cast<GepVFGNode>(node))
        return gep->getPAGDstNode();
    else if (const LoadVFGNode* load = SVFUtil::dyn_cast<LoadVFGNode>(node))
        return load->getPAGDstNode();
    else if (const PHIVFGNode* phi = SVFUtil::dyn_cast<PHIVFGNode>(node))
        return phi->getRes();
    else if (const CmpVFGNode* cmp = SVFUtil::dyn_cast<CmpVFGNode>(node))
        return cmp->getRes();
    else if (const BinaryOPVFGNode* bop =
                 SVFUtil::dyn_cast<BinaryOPVFGNode>(node))
        return bop->getRes();
    else if (const UnaryOPVFGNode* uop =
                 SVFUtil::dyn_cast<UnaryOPVFGNode>(node))
        return uop->getRes();
    else if (const ActualParmVFGNode* ap =
                 SVFUtil::dyn_cast<ActualParmVFGNode>(node))
        return ap->getParam();
    else if (const FormalParmVFGNode* fp =
                 SVFUtil::dyn_cast<FormalParmVFGNode>(node))
        return fp->getParam();
    else if (const ActualRetVFGNode* ar =
                 SVFUtil::dyn_cast<ActualRetVFGNode>(node))
        return ar->getRev();
    else if (const FormalRetVFGNode* fr =
                 SVFUtil::dyn_cast<FormalRetVFGNode>(node))
        return fr->getRet();
    else if (const NullPtrVFGNode* nullVFG =
                 SVFUtil::dyn_cast<NullPtrVFGNode>(node))
        return nullVFG->getPAGNode();
    else if (const BranchVFGNode* branch =
                 SVFUtil::dyn_cast<BranchVFGNode>(node))
        return branch->getBranchStmt()->getBranchInst();
    // FIXME::whther store can use DestNode?
    else if (const StoreVFGNode* store =
                 SVFUtil::dyn_cast<StoreVFGNode>(node)) {
        return store->getPAGSrcNode();
    }
    return nullptr;
}