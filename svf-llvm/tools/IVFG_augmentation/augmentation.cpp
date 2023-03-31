#include "SVF-LLVM/LLVMUtil.h"
#include "Graphs/SVFG.h"
#include "WPA/Andersen.h"
#include "SVF-LLVM/SVFIRBuilder.h"
#include "Util/CommandLine.h"
#include "Util/Options.h"
#include "SVFIR/SymbolTableInfo.h"
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

bool var_has_val(const SVFVar* var)
{
    assert(var != nullptr);
    if (var->getNodeKind() == SVFVar::PNODEK::DummyObjNode || var->getNodeKind() == SVFVar::PNODEK::DummyObjNode){
        return false;
    }
    if (SymbolTableInfo::isBlkObj(var->getId())){
        return false;
        assert(0);
    }
    if (SymbolTableInfo::isConstantObj(var->getId())){
        return false;
        assert(0);
    }
    return var->hasValue();
}

#define NodeTy(x, y) SVF::x* ptr = SVFUtil::dyn_cast<SVF::x>(y)
#define Is(x, y) (x::classof(y))
void dump_nodes_features(SVFG* svfg){
    for(SVF::u32_t i = 0; i < svfg->getSVFGNodeNum(); i++)
    {
        SVFGNode* node = svfg->getSVFGNode(i);
        dictTy dict;
        fea_loc(node, dict);
        fea_nodeType(node, dict);
        bool is_triple_instructions = Is(BinaryOPVFGNode, node) || Is(CmpVFGNode, node) || Is(PHIVFGNode, node) || Is(StmtVFGNode, node) || Is(UnaryOPVFGNode, node);
        bool is_mr_instructions = Is(MRSVFGNode, node);
        bool is_arg_instructions = Is(ArgumentVFGNode, node);
        bool is_top_phi_instructions = Is(PHIVFGNode, node);
        //bool is_mem_phi_instructions = Is(MSSAPHISVFGNode, node);
        //bool is_other_instructions = Is(BranchVFGNode, node) || Is(NullPtrVFGNode, node) || Is(DummyVersionPropSVFGNode, node);
        if (is_triple_instructions){
            const SVFVar* LVar = getLHSTopLevPtr(node);
            dict["dest_name"] = "";
            dict["dest_type"] = "";
            dict["inst_full"] = "";
            if (LVar == nullptr){
                //FIXME::current if LVar is nullptr, then SVFGNode is StoreSVFGNode, or LoadSVFGNode(?) about call @f
                dict["inst_full"] = node->getValue()->toString();
            }else if (!var_has_val(LVar)){
                //FIXME::currnet this means LVar is DummyObjectVar or DummyTopLevelVar such as @llvm.memcpy, null ...
                dict["dest_name"] = node->getValue()->getName();
                //they don't have type
                dict["dest_type"] = "";
                dict["inst_full"] = node->getValue()->toString();
                cout << dict2str(dict) << "\n";
            }else
            {
                dict["dest_name"] = LVar->getValueName();
                // FIXME::SVF has bug when using LVar->getType
                dict["dest_type"] = node->getValue()->getType()->toString();
                dict["inst_full"] = LVar->getValue()->toString();
                const Value* val = LLVMModuleSet::getLLVMModuleSet()->getLLVMValue(LVar->getValue());
                const Instruction* ins = llvm::dyn_cast<Instruction>(val);
                if (ins){
                    dict["inst_op"] = ins->getOpcodeName();
                }
            }
        }
        if (is_arg_instructions || is_top_phi_instructions)
        {
            const SVFVar* LVar = getLHSTopLevPtr(node);
            assert(LVar != nullptr);
            if (var_has_val(LVar))
            {
                dict["dest_name"] = LVar->getValueName();
                // FIXME::SVF has bug if use LVar->getType()->toString();
                dict["dest_type"] = node->getValue()->getType()->toString();
                dict["inst_full"] = LVar->getValue()->toString();
            }
            else
            {
                dict["inst_full"] = node->getValue()->toString();
            }
        }
        if (is_mr_instructions)
        {
            dict["mr_id"] = "";
            dict["mr_size"] = "";
            dict["mr_version"] = "";
            const MRVer* mr;
            if (NodeTy(MSSAPHISVFGNode, node))
            {
                dict["branch_num"] = std::to_string(ptr->getOpVerNum());
                mr = ptr->getResVer();
            }
            else
            {
                if (NodeTy(ActualINSVFGNode, node))
                {
                    mr = ptr->getMRVer();
                }
                if (NodeTy(ActualOUTSVFGNode, node))
                {
                    mr = ptr->getMRVer();
                }
                if (NodeTy(FormalINSVFGNode, node))
                {
                    mr = ptr->getMRVer();
                }
                if (NodeTy(FormalOUTSVFGNode, node))
                {
                    mr = ptr->getMRVer();
                }
            }
            dict["mr_id"] = std::to_string(mr->getID());
            dict["mr_version"] = std::to_string(mr->getSSAVersion());
            dict["mr_size"] = std::to_string(mr->getMR()->getRegionSize());
        }
        if (NodeTy(PHIVFGNode, node)){
            dict["branch_num"] = std::to_string(ptr->getOpVerNum());
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

