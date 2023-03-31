#include "SVF-LLVM/LLVMUtil.h"
#include "Graphs/SVFG.h"
#include "WPA/Andersen.h"
#include "SVF-LLVM/SVFIRBuilder.h"
#include "Util/CommandLine.h"
#include "Util/Options.h"
#include<stdio.h>

using namespace std;
using namespace SVF;

static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
                return !std::isspace(ch);
            }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
                return !std::isspace(ch);
            }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    rtrim(s);
    ltrim(s);
}

void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}


static std::string dict2str(std::map<std::string, std::string> dict){
    std::string str;
    std::stringstream  rawstr(str);
    for(auto &it: dict){
        trim(it.second);
    }
    //print
    rawstr << "node_feature: {";
    for(auto& item: dict){
        rawstr << "\'" << item.first << "\'" << " : " << "\'" << item.second << "\', ";
    }
    rawstr << "}";
    return rawstr.str();
}
typedef std::map<std::string, std::string> dictTy;
void fea_loc(SVFGNode* node, dictTy& dict){
    if (node->getICFGNode()->getFun() == nullptr){
        dict["func_name"] = "GLOBAL";
    }else{
        dict["func_name"] = node->getICFGNode()->getFun()->getName();
    }
    if (node->getICFGNode()->getBB() == nullptr){
        dict["block_name"] = "GLOBAL";
    }else
    {
        dict["block_name"] = node->getICFGNode()->getBB()->getName();
    }
}

#define NodeK(x)                                                               \
    {                                                                          \
        SVFGNode::VFGNodeK::x, #x                                              \
    }
std::map<SVFGNode::VFGNodeK, std::string> nodeType2Str = {
    NodeK(Addr),   NodeK(Copy),      NodeK(Gep),       NodeK(Store),
    NodeK(Load),   NodeK(Cmp),       NodeK(BinaryOp),  NodeK(UnaryOp),
    NodeK(Branch), NodeK(TPhi),      NodeK(TIntraPhi), NodeK(TInterPhi),
    NodeK(MPhi),   NodeK(MIntraPhi), NodeK(MInterPhi), NodeK(FRet),
    NodeK(ARet),   NodeK(AParm),     NodeK(FParm),     NodeK(FunRet),
    NodeK(APIN),   NodeK(APOUT),     NodeK(FPIN),      NodeK(FPOUT),
    NodeK(NPtr),   NodeK(DummyVProp)};

void fea_nodeType(SVFGNode* node, dictTy& dict)
{
    dict["node_type"] = nodeType2Str[SVFGNode::VFGNodeK(node->getNodeKind())];
}
// typedef SVFVar::PNODEK vpd;
// void fea_instFull(SVFG* svfg, SVFGNode * node, dictTy& dict){
//     const SVFVar* var = svfg->getLHSTopLevPtr(node);
//     if(var && var->hasValue()){
//         if (var->getNodeKind() != vpd::DummyValNode && var->getNodeKind() !=
//         vpd::DummyObjNode){
//             dict["inst_full"] = var->getValue()->toString();
//         }
//     }
////    switch(var->getNodeKind()){
////    case vpd::DummyValNode:
////    case vpd::DummyObjNode:
////    }
////    if (var->getNodeKind() == SVFVar::PNODEK::DummyValNode || var)
////    if (MRSVFGNode::classof(node) || BranchVFGNode::classof(node) ||
///NullPtrVFGNode::classof(node) || UnaryOPVFGNode::classof(node)){
////
////    }else{
////
////    }
//}

const PAGNode* getLHSTopLevPtr(const VFGNode* node)
{

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
    return nullptr;
}
#define NodeTy(x) SVF::x* ptr = SVFUtil::dyn_cast<SVF::x>(node)
void dump_nodes_features(SVFG* svfg){
    for(SVF::u32_t i = 0; i < svfg->getSVFGNodeNum(); i++)
    {
        SVFGNode* node = svfg->getSVFGNode(i);
        dictTy dict;
        fea_loc(node, dict);
        fea_nodeType(node, dict);
        // fea_instFull(svfg, node, dict);
        // node->getNodeKind()
        // std::cout << svfg->getLHSTopLevPtr(node)->getValueName() <<
        // std::endl;
        if (NodeTy(ArgumentVFGNode))
        {
            dict["inst_full"] = ptr->getValue()->toString();
        }
        else if (NodeTy(BinaryOPVFGNode))
        {
            ptr->toString();
            dict["inst_full"] = ptr->getRes()->getValue()->toString();
            const llvm::Value* val =
                LLVMModuleSet::getLLVMModuleSet()->getLLVMValue(
                    ptr->getRes()->getValue());
            const llvm::Instruction* ins = llvm::cast<llvm::Instruction>(val);
            dict["inst_name"] = std::string(ins->getOpcodeName());
            dict["dest_name"] = ptr->getRes()->getValueName();
            dict["dest_type"] =
                ptr->getRes()->getValue()->getType()->toString();
        }
        else if (NodeTy(BranchVFGNode))
        {
            ptr->toString();
        }
        else if (NodeTy(CmpVFGNode))
        {
            dict["inst_full"] = ptr->getRes()->getValue()->toString();
            const llvm::Value* val =
                LLVMModuleSet::getLLVMModuleSet()->getLLVMValue(
                    ptr->getRes()->getValue());
            const llvm::Instruction* ins = llvm::cast<llvm::Instruction>(val);
            dict["inst_name"] = std::string(ins->getOpcodeName());
        }
        else if (NodeTy(MRSVFGNode))
        {
            ptr->toString();
            const MRVer* mr;
            if (NodeTy(MSSAPHISVFGNode))
            {
                // FIXME::In ALL PHI Node we need to count about opverNum and
                // info about dest
                dict["branch_num"] = std::to_string(ptr->getOpVerNum());
                mr = ptr->getResVer();
            }
            else
            {
                if (NodeTy(ActualINSVFGNode))
                {
                    mr = ptr->getMRVer();
                    // dict["callsite_full"] =
                    // ptr->getCallSite()->getCallSite()->toString();
                }
                if (NodeTy(ActualOUTSVFGNode))
                {
                    mr = ptr->getMRVer();
                    // dict["callsite_full"] =
                    // ptr->getCallSite()->getCallSite()->toString();
                }
                if (NodeTy(FormalINSVFGNode))
                {
                    mr = ptr->getMRVer();
                }
                if (NodeTy(FormalOUTSVFGNode))
                {
                    mr = ptr->getMRVer();
                }
            }
            dict["mr_id"] = std::to_string(mr->getID());
            dict["mr_version"] = std::to_string(mr->getSSAVersion());
            dict["mr_size"] = std::to_string(mr->getMR()->getRegionSize());
        }
        else if (NodeTy(NullPtrVFGNode))
        {
            ptr->toString();
        }
        else if (NodeTy(PHIVFGNode))
        {
            dict["branch_num"] = std::to_string(ptr->getOpVerNum());
            dict["inst_full"] = ptr->getRes()->getValue()->toString();
            dict["dest_name"] = ptr->getRes()->getValueName();
            dict["dest_type"] = ptr->getRes()->getType()->toString();
        }
        else if (NodeTy(StmtVFGNode))
        {
            dict["dest_name"] = ptr->getPAGDstNode()->getValueName();
            const AssignStmt* stmt = (AssignStmt*)ptr->getPAGEdge();
            if (stmt->)
            {
                dict["dest_type"] = stmt->getLHSVar()->getType()->toString();
            }
            else
            {
                dict["dest_type"] = "";
            }
            //            if (ptr->getPAGDstNode()->getType() == nullptr){
            //                dict["dest_type"] = "NULL";
            //            }else
            //            {
            //                dict["dest_type"] =
            //                ptr->getPAGDstNode()->getType()->toString();
            //            }
            //            if (ptr->getInst() == nullptr){
            //                dict["inst_full"] = "";
            //            }else{
            //                dict["inst_full"] = ptr->getInst()->toString();
            ////                const llvm::Value* val =
            ///LLVMModuleSet::getLLVMModuleSet()->getLLVMValue(ptr->getInst());
            ////                const llvm::Instruction* ins =
            ///llvm::dyn_cast<llvm::Instruction>(val); /                if (ins
            ///== nullptr){ /                    dict["inst_name"] = ""; /
            ///}else{ /                    dict["inst_name"] =
            ///std::string(ins->getOpcodeName()); /                }
            //};
            // ptr->getInst()
            // dict["inst_full"] = ptr->getPAGDstNode()->getValue()->toString();
            printf("%s\n", dict2str(dict).c_str());
        }
        else if (NodeTy(UnaryOPVFGNode))
        {
            ptr->toString();
        }
        else
        {
            assert("out of type!");
        }
    }
}


int main(int argc, char ** argv)
{
    char **arg_value = new char*[argc];
    std::vector<std::string> moduleNameVec;
    moduleNameVec = OptionBase::parseOptions(
        argc, argv, "Whole Program Points-to Analysis", "[options] <input-bitcode...>"
    );

    if (Options::WriteAnder() == "ir_annotator")
    {
        LLVMModuleSet::getLLVMModuleSet()->preProcessBCs(moduleNameVec);
    }

    SVFModule* svfModule = LLVMModuleSet::getLLVMModuleSet()->buildSVFModule(moduleNameVec);

    /// Build Program Assignment Graph (SVFIR)
    SVFIRBuilder builder(svfModule);
    SVFIR* pag = builder.build();

    /// Create Andersen's pointer analysis
    Andersen* ander = AndersenWaveDiff::createAndersenWaveDiff(pag);

    /// Sparse value-flow graph (SVFG)

    SVFGBuilder svfBuilder(true);
    SVFG* svfg = svfBuilder.buildFullSVFG(ander);

    ///dump node features
    dump_nodes_features(svfg);

    AndersenWaveDiff::releaseAndersenWaveDiff();
    SVFIR::releaseSVFIR();

    SVF::LLVMModuleSet::releaseLLVMModuleSet();


    llvm::llvm_shutdown();
    delete[] arg_value;
    return 0;
}

