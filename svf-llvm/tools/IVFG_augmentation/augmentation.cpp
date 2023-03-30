#include "SVF-LLVM/LLVMUtil.h"
#include "Graphs/SVFG.h"
#include "WPA/Andersen.h"
#include "SVF-LLVM/SVFIRBuilder.h"
#include "Util/CommandLine.h"
#include "Util/Options.h"

using namespace std;
using namespace SVF;

//void dump_nodes_features(SVFG* svfg){
//    VFG
//    for(int i = 0; i < svfg->getDefSVFGNode())
//
//}
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

    VFG* vfg = new VFG(ander->getPTACallGraph());
    vfg->dump("/home/shuibing/ptCode2vec/SVFRemote/cmake-build-debug/bin/haha");
    /// Sparse value-flow graph (SVFG)

    SVFGBuilder svfBuilder(true);
    SVFG* svfg = svfBuilder.buildFullSVFG(ander);
    //svfg->dump("/home/shuibing/ptCode2vec/SVFRemote/cmake-build-debug/bin/haha");
    /// Collect uses of an LLVM Value
    for(SVF::u32_t i = 0; i < svfg->getSVFGNodeNum(); i++){
        std::cout << svfg->getSVFGNode(i)->toString() << "\n\n";
    }


    delete vfg;
    AndersenWaveDiff::releaseAndersenWaveDiff();
    SVFIR::releaseSVFIR();

    SVF::LLVMModuleSet::releaseLLVMModuleSet();


    llvm::llvm_shutdown();
    delete[] arg_value;
    return 0;
}

