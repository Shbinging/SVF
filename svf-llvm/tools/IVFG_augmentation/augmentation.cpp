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

#define NodeK(x) {SVFGNode::VFGNodeK::x, #x}
std::map<SVFGNode::VFGNodeK, std::string> nodeType2Str = {
    NodeK(Addr), NodeK(Copy), NodeK(Gep), NodeK(Store), NodeK(Load), NodeK(Cmp), NodeK(BinaryOp), NodeK(UnaryOp), NodeK(Branch), NodeK(TPhi), NodeK(TIntraPhi), NodeK(TInterPhi), NodeK(MPhi), NodeK(MIntraPhi), NodeK(MInterPhi), NodeK(FRet), NodeK(ARet), NodeK(AParm), NodeK(FParm), NodeK(FunRet), NodeK(APIN), NodeK(APOUT), NodeK(FPIN), NodeK(FPOUT), NodeK(NPtr), NodeK(DummyVProp)
};

void fea_nodeType(SVFGNode * node, dictTy& dict) {
    dict["node_type"] = nodeType2Str[SVFGNode::VFGNodeK(node->getNodeKind())];
}

#define NodeTy(x) SVF::x* ptr = SVFUtil::dyn_cast<SVF::x>(node)
void dump_nodes_features(SVFG* svfg){
    for(SVF::u32_t i = 0; i < svfg->getSVFGNodeNum(); i++){
        SVFGNode * node = svfg->getSVFGNode(i);
        dictTy  dict;
        fea_loc(node, dict);
        fea_nodeType(node, dict);
        //node->getNodeKind()
        //std::cout << svfg->getLHSTopLevPtr(node)->getValueName() << std::endl;
//        if (NodeTy(ArgumentVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(BinaryOPVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(BranchVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(CmpVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(CmpVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(MRSVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(NullPtrVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(PHIVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(StmtVFGNode))
//        {
//            ptr->toString();
//        }
//        else if (NodeTy(UnaryOPVFGNode))
//        {
//            ptr->toString();
//        }
//        else{
//            assert("out of type!");
//        }
        printf("%s\n", dict2str(dict).c_str());
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

