#include "SVF-LLVM/LLVMUtil.h"
#include "Graphs/SVFG.h"
#include "WPA/Andersen.h"
#include "SVF-LLVM/SVFIRBuilder.h"
#include "Util/CommandLine.h"
#include "Util/Options.h"

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

std::map<std::string, std::string> get_loc_dict(SVFGNode* node){
    std::map<std::string, std::string> dict;
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
    return dict;
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

#include<stdio.h>
void dump_nodes_features(SVFG* svfg){
    for(SVF::u32_t i = 0; i < svfg->getSVFGNodeNum(); i++){
        SVFGNode * node = svfg->getSVFGNode(i);
        auto dict = get_loc_dict(node);
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

