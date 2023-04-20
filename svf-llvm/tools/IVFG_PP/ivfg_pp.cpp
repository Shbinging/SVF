//#include "Graphs/SVFG.h"
//#include "SVF-LLVM/LLVMUtil.h"
//#include "SVF-LLVM/SVFIRBuilder.h"
//#include "SVFIR/SymbolTableInfo.h"
//#include "Util/CommandLine.h"
//#include "Util/Options.h"
//#include "WPA/Andersen.h"
//#include "rustc_demangle.h"
//#include <stdio.h>
//#include <regex>
//#include <cxxabi.h>
//#include <numpy/arrayobject.h>
//using namespace std;
//using namespace SVF;
//typedef std::map<std::string, std::string> dictTy;
//
//static inline void ltrim(std::string &s) {
//    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
//                return !std::isspace(ch);
//            }));
//}
//
//// trim from end (in place)
//static inline void rtrim(std::string &s) {
//    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
//                return !std::isspace(ch);
//            }).base(), s.end());
//}
//
//// trim from both ends (in place)
//static inline void trim(std::string &s) {
//    rtrim(s);
//    ltrim(s);
//}
//
//void replaceAll(std::string& str, const std::string& from, const std::string& to) {
//    if(from.empty())
//        return;
//    size_t start_pos = 0;
//    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
//        str.replace(start_pos, from.length(), to);
//        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
//    }
//}
//std::string rustDemangle(std::string oriName){
//    char st[300];
//    int res = rustc_demangle(oriName.c_str(), st, 300);
//    return std::string(st);
//    assert(res);
//}
//
//void demangleRustName(dictTy& dict){
//    std::regex regex_pattern("_ZN[a-zA-Z0-9.$_]+");
//    for (auto& it : dict)
//    {
//        trim(it.second);
//        std::string old_target, new_target;
//        old_target = it.second;
//        std::sregex_iterator sr_it(old_target.begin(), old_target.end(),
//                                   regex_pattern);
//        std::sregex_iterator end;
//        while (sr_it != end)
//        {
//            std::string catch_str = (*sr_it)[0];
//            replaceAll(catch_str, "_ret", "");
//            replaceAll(it.second, catch_str, rustDemangle(catch_str));
//            sr_it++;
//        }
//    }
//}
//
//void demangleCPPName(dictTy& dict){
//    std::regex regex_pattern("_Z[a-zA-Z0-9.$_]+");
//    for (auto& it : dict)
//    {
//        std::string old_target, new_target;
//        old_target = it.second;
//        std::sregex_iterator sr_it(old_target.begin(), old_target.end(),
//                                   regex_pattern);
//        std::sregex_iterator end;
//        while (sr_it != end)
//        {
//            std::string catch_str = (*sr_it)[0];
//            replaceAll(catch_str, "_ret", "");
//            sr_it++;
//            s32_t status;
//            char* realname = abi::__cxa_demangle(catch_str.c_str(), 0, 0, &status);
//            if (realname == nullptr){
//                continue;
//            }
//            assert(realname != nullptr);
//            std::string  realname_str(realname);
//            replaceAll(it.second, catch_str, realname_str);
//        }
//    }
//}
//
//static std::string dict2str(std::map<std::string, std::string> dict){
//    std::string str;
//    std::stringstream  rawstr(str);
//    for(auto &it: dict){
//        trim(it.second);
//    }
//    if (Options::demangleRust()){
//        demangleRustName(dict);
//    }
//    if (Options::demangleCPP()){
//        demangleCPPName(dict);
//    }
//    //print
//    rawstr << "node_feature: {";
//    for(auto& item: dict){
//        // if (item.first != "inst_full" && item.first != "var_name" &&
//        // item.first != "idx") continue;
//        rawstr << "\"\"\"" << item.first << "\"\"\"" << " : " << "\"\"\"" << item.second << "\"\"\", ";
//    }
//    rawstr << "}";
//    return rawstr.str();
//}
//
//
//void fea_loc(SVFGNode* node, dictTy& dict){
//    if (node->getICFGNode()->getFun() == nullptr){
//        dict["func_name"] = "GLOBAL";
//    }else{
//        dict["func_name"] = node->getICFGNode()->getFun()->getName();
//    }
//    if (node->getICFGNode()->getBB() == nullptr){
//        dict["block_name"] = "GLOBAL";
//    }else
//    {
//        dict["block_name"] = node->getICFGNode()->getBB()->getName();
//    }
//}
//
//#define NodeK(x)                                                               \
//    {                                                                          \
//        SVFGNode::VFGNodeK::x, #x                                              \
//    }
//std::map<SVFGNode::VFGNodeK, std::string> nodeType2Str = {
//    NodeK(Addr),   NodeK(Copy),      NodeK(Gep),       NodeK(Store),
//    NodeK(Load),   NodeK(Cmp),       NodeK(BinaryOp),  NodeK(UnaryOp),
//    NodeK(Branch), NodeK(TPhi),      NodeK(TIntraPhi), NodeK(TInterPhi),
//    NodeK(MPhi),   NodeK(MIntraPhi), NodeK(MInterPhi), NodeK(FRet),
//    NodeK(ARet),   NodeK(AParm),     NodeK(FParm),     NodeK(FunRet),
//    NodeK(APIN),   NodeK(APOUT),     NodeK(FPIN),      NodeK(FPOUT),
//    NodeK(NPtr),   NodeK(DummyVProp)};
//
//void fea_nodeType(SVFGNode* node, dictTy& dict)
//{
//    // FIXME::type maybe wrong
//    dict["node_type"] = nodeType2Str[SVFGNode::VFGNodeK(node->getNodeKind())];
//}
//
//const PAGNode* getLHSTopLevPtr(const VFGNode* node)
//{
//
//    if (const AddrVFGNode* addr = SVFUtil::dyn_cast<AddrVFGNode>(node))
//        return addr->getPAGDstNode();
//    else if (const CopyVFGNode* copy = SVFUtil::dyn_cast<CopyVFGNode>(node))
//        return copy->getPAGDstNode();
//    else if (const GepVFGNode* gep = SVFUtil::dyn_cast<GepVFGNode>(node))
//        return gep->getPAGDstNode();
//    else if (const LoadVFGNode* load = SVFUtil::dyn_cast<LoadVFGNode>(node))
//        return load->getPAGDstNode();
//    else if (const PHIVFGNode* phi = SVFUtil::dyn_cast<PHIVFGNode>(node))
//        return phi->getRes();
//    else if (const CmpVFGNode* cmp = SVFUtil::dyn_cast<CmpVFGNode>(node))
//        return cmp->getRes();
//    else if (const BinaryOPVFGNode* bop =
//                 SVFUtil::dyn_cast<BinaryOPVFGNode>(node))
//        return bop->getRes();
//    else if (const UnaryOPVFGNode* uop =
//                 SVFUtil::dyn_cast<UnaryOPVFGNode>(node))
//        return uop->getRes();
//    else if (const ActualParmVFGNode* ap =
//                 SVFUtil::dyn_cast<ActualParmVFGNode>(node))
//        return ap->getParam();
//    else if (const FormalParmVFGNode* fp =
//                 SVFUtil::dyn_cast<FormalParmVFGNode>(node))
//        return fp->getParam();
//    else if (const ActualRetVFGNode* ar =
//                 SVFUtil::dyn_cast<ActualRetVFGNode>(node))
//        return ar->getRev();
//    else if (const FormalRetVFGNode* fr =
//                 SVFUtil::dyn_cast<FormalRetVFGNode>(node))
//        return fr->getRet();
//    else if (const NullPtrVFGNode* nullVFG =
//                 SVFUtil::dyn_cast<NullPtrVFGNode>(node))
//        return nullVFG->getPAGNode();
//    else if (const BranchVFGNode* branch =
//                 SVFUtil::dyn_cast<BranchVFGNode>(node))
//        return branch->getBranchStmt()->getBranchInst();
//    // FIXME::whther store can use DestNode?
//    else if (const StoreVFGNode* store = SVFUtil::dyn_cast<StoreVFGNode>(node))
//    {
//        return store->getPAGDstNode();
//    }
//    return nullptr;
//}
//
//bool var_has_val(const SVFVar* var)
//{
//    assert(var != nullptr);
//    if (var->getNodeKind() == SVFVar::PNODEK::DummyObjNode || var->getNodeKind() == SVFVar::PNODEK::DummyObjNode){
//        return false;
//    }
//    if (SymbolTableInfo::isBlkObj(var->getId())){
//        return false;
//        assert(0);
//    }
//    if (SymbolTableInfo::isConstantObj(var->getId()))
//    {
//        return false;
//        assert(0);
//    }
//    return var->hasValue();
//}
//typedef std::map<SVFGNode*, dictTy> node_dict_type;
//#define NodeTy(x, y) SVF::x* ptr = SVFUtil::dyn_cast<SVF::x>(y)
//#define Is(x, y) (x::classof(y))
//
//node_dict_type get_nodes_features(SVFG* svfg)
//{
//    // std::ofstream fout;
//    // fout.open(featruePath.c_str(), std::ios::out | std::ios::trunc);
//    node_dict_type res;
//    for (SVF::u32_t i = 0; i < svfg->getSVFGNodeNum(); i++)
//    {
//        SVFGNode* node = svfg->getSVFGNode(i);
//        dictTy dict;
//        fea_loc(node, dict);
//        fea_nodeType(node, dict);
//        dict["idx"] = std::to_string(i);
//        bool is_triple_instructions =
//            Is(BinaryOPVFGNode, node) || Is(CmpVFGNode, node) ||
//            Is(PHIVFGNode, node) || Is(StmtVFGNode, node) ||
//            Is(UnaryOPVFGNode, node);
//        bool is_mr_instructions = Is(MRSVFGNode, node);
//        bool is_arg_instructions = Is(ArgumentVFGNode, node);
//        bool is_top_phi_instructions = Is(PHIVFGNode, node);
//        //bool is_mem_phi_instructions = Is(MSSAPHISVFGNode, node);
//        //bool is_other_instructions = Is(BranchVFGNode, node) || Is(NullPtrVFGNode, node) || Is(DummyVersionPropSVFGNode, node);
//        if (is_triple_instructions){
//            const SVFVar* LVar = getLHSTopLevPtr(node);
//            dict["dest_name"] = "";
//            dict["dest_type"] = "";
//            dict["inst_full"] = "";
//            if (LVar == nullptr){
//                //FIXME::current if LVar is nullptr, then SVFGNode is StoreSVFGNode, or LoadSVFGNode(?) about call @f , value is SVFConstant...
//                dict["inst_full"] = node->getValue()->toString();
//            }else if (!var_has_val(LVar)){
//                //FIXME::currnet this means LVar is DummyObjectVar or DummyTopLevelVar such as @llvm.memcpy, null ...
//                if (node->getValue())
//                {
//                    dict["dest_name"] = node->getValue()->getName();
//                    // they don't have type
//                    dict["dest_type"] = "";
//                    dict["inst_full"] = node->getValue()->toString();
//                }
//                dict2str(dict);
//            }else
//            {
//                dict["dest_name"] = LVar->getValueName();
//                // FIXME::SVF has bug when using LVar->getType
//                dict["dest_type"] = node->getValue()->getType()->toString();
//                dict["inst_full"] = LVar->getValue()->toString();
//                const Value* val = LLVMModuleSet::getLLVMModuleSet()->getLLVMValue(LVar->getValue());
//                const Instruction* ins = llvm::dyn_cast<Instruction>(val);
//                if (ins){
//                    dict["inst_op"] = ins->getOpcodeName();
//                }
//            }
//        }
//        if (is_arg_instructions || is_top_phi_instructions)
//        {
//            const SVFVar* LVar = getLHSTopLevPtr(node);
//            assert(LVar != nullptr);
//            if (var_has_val(LVar))
//            {
//                dict["dest_name"] = LVar->getValueName();
//                // FIXME::SVF has bug if use LVar->getType()->toString();
//                dict["dest_type"] = node->getValue()->getType()->toString();
//                dict["inst_full"] = LVar->getValue()->toString();
//            }
//            else
//            {
//                if (node->getValue()){
//                    dict["inst_full"] = node->getValue()->toString();
//                }
//            }
//        }
//        if (is_mr_instructions)
//        {
//            dict["mr_id"] = "";
//            dict["mr_size"] = "";
//            dict["mr_version"] = "";
//            const MRVer* mr;
//            if (NodeTy(MSSAPHISVFGNode, node))
//            {
//                dict["branch_num"] = std::to_string(ptr->getOpVerNum());
//                mr = ptr->getResVer();
//            }
//            else
//            {
//                if (NodeTy(ActualINSVFGNode, node))
//                {
//                    mr = ptr->getMRVer();
//                }
//                if (NodeTy(ActualOUTSVFGNode, node))
//                {
//                    mr = ptr->getMRVer();
//                }
//                if (NodeTy(FormalINSVFGNode, node))
//                {
//                    mr = ptr->getMRVer();
//                }
//                if (NodeTy(FormalOUTSVFGNode, node))
//                {
//                    mr = ptr->getMRVer();
//                }
//            }
//            dict["mr_id"] = std::to_string(mr->getID());
//            dict["mr_version"] = std::to_string(mr->getSSAVersion());
//            dict["mr_size"] = std::to_string(mr->getMR()->getRegionSize());
//        }
//        if (NodeTy(PHIVFGNode, node))
//        {
//            dict["branch_num"] = std::to_string(ptr->getOpVerNum());
//        }
//        res[node] = dict;
//        //        fout << "Node" << static_cast<const void*>(node) << "\t";
//        //        fout << dict2str(dict) << "\n";
//    }
//    // fout.close();
//    return res;
//}
//
//void dump_nodes_feature(node_dict_type& dict, std::string feature_path)
//{
//    std::ofstream fout;
//    fout.open(feature_path.c_str(), std::ios::out | std::ios::trunc);
//    for (auto& it : dict)
//    {
//        fout << "Node" << static_cast<const void*>(it.first) << "\t";
//        fout << dict2str(it.second) << "\n";
//    }
//    fout.close();
//}
//
//void dfs_var_name(SVFGNode* node, node_dict_type& dict,
//                  unordered_map<SVF::u32_t, int>& visited, std::string var_name)
//{
//    if (node == nullptr || visited.count(node->getId()))
//    {
//        return;
//    }
//    auto& node_dict = dict[node];
//    if (node_dict.count("dest_name") && node_dict["dest_name"] != "")
//    {
//        var_name = node_dict["dest_name"];
//    }
//    if (var_name != "")
//    {
//        node_dict["var_name"] = var_name;
//        visited[node->getId()] = 1;
//        for (auto it = node->getOutEdges().begin();
//             it != node->getOutEdges().end(); it++)
//        {
//            if ((*it)->isDirectVFGEdge())
//            {
//                dfs_var_name((*it)->getDstNode(), dict, visited, var_name);
//            }
//        }
//    }
//}
//
//void del_svfg_all_edges(SVFGNode* node, SVFG* svfg)
//{
//    std::vector<SVF::GenericNode<SVF::VFGNode, SVF::VFGEdge>::EdgeType*> in,
//        out;
//    for (auto it = node->getInEdges().begin(); it != node->getInEdges().end();
//         it++)
//    {
//        in.push_back(*it);
//    }
//    for (auto it : in)
//    {
//        svfg->removeSVFGEdge(it);
//    }
//    for (auto it = node->OutEdgeBegin(); it != node->OutEdgeEnd(); it++)
//    {
//        out.push_back(*it);
//    }
//    for (auto it : out)
//    {
//        svfg->removeSVFGEdge(it);
//    }
//}
//
//void impl_var_name(SVFG* svfg, node_dict_type& dict)
//{
//    unordered_map<SVF::u32_t, int> visited;
//    for (SVF::u32_t i = 0; i < svfg->nodeNum; i++)
//    {
//        SVFGNode* node = svfg->getSVFGNode(i);
//        dfs_var_name(node, dict, visited, "");
//    }
//    for (auto& it : dict)
//    {
//        if (!it.second.count("var_name"))
//        {
//            del_svfg_all_edges(it.first, svfg);
//            svfg->removeSVFGNode(it.first);
//        }
//    }
//}
//
//std::map<SVFGNode*, int> get_new_node_id(SVFG* svfg, const SVFFunction* fun){
//    std::map<SVFGNode*, int> node2id;
//    int s = 0;
//    for (auto it = svfg->getVFGNodeBegin(fun); it != svfg->getVFGNodeEnd(fun); it++){
//        SVFGNode* node = (*it);
//        node2id[node] = s++;
//    }
//    for (auto it = svfg->getVFGNodeBegin(fun); it != svfg->getVFGNodeEnd(fun); it++){
//        SVFGNode* node = (*it);
//        for(auto it = node->OutEdgeBegin(); it != node->OutEdgeEnd(); it++){
//            SVFGNode* node = (*it)->getDstNode();
//            if (node2id.find(node) == node2id.end()){
//                node2id[node] = s++;
//            }
//        }
//    }
//    return node2id;
//}
//
//void dump_graph(SVFG* svfg, const SVFFunction* fun, std::string graphPath, std::map<SVFGNode*, int>& node2id){
//    std::ofstream fout;
//    fout.open(graphPath.c_str(), std::ios::out | std::ios::trunc);
//    for (auto it = svfg->getVFGNodeBegin(fun); it != svfg->getVFGNodeEnd(fun); it++){
//        SVFGNode* node = (*it);
//        fout << std::to_string(node2id[node]);
//        for(auto it = node->OutEdgeBegin(); it != node->OutEdgeEnd(); it++){
//            SVFGNode* dst_node = (*it)->getDstNode();
//            fout << " " << std::to_string(node2id[dst_node]);
//        }
//        fout << "\n";
//    }
//    fout.close();
//}
//
//void dump_varname(std::string featurePath, std::map<SVFGNode*, int>& node2id, node_dict_type& dict){
//    std::ofstream fout;
//    fout.open(featurePath.c_str(), std::ios::out | std::ios::trunc);
//    for(auto it = node2id.begin(); it != node2id.end(); it++){
//        fout << std::to_string(it->second) << "," << dict[it->first]["var_name"] << "\n";
//    }
//    fout.close();
//}
//
//void generate_graph_dataset(SVFG* svfg, SVFModule* svfModule, node_dict_type& dict){
//    for(auto it = svfModule->getFunctionSet().begin(); it != svfModule->getFunctionSet().end(); it++){
//        const SVFFunction* func = (*it);
//        cout << func->getName() << endl;
//        if (!svfg->hasVFGNodes(func)) continue;
//        std::map<SVFGNode*, int> node2id = get_new_node_id(svfg, func);
//        //FIXME::cpp/rust need mangle name
//        dump_graph(svfg, func, Options::graphPath() + func->getName(), node2id);
//        dump_varname(Options::featurePath() + func->getName(), node2id, dict);
//    }
//}
//
//
//class SVFGBuilder_Opt:public SVFGBuilder{
//public:
//    explicit SVFGBuilder_Opt(bool _SVFGWithIndCall = false): SVFGBuilder(_SVFGWithIndCall){
//
//    }
//    virtual ~SVFGBuilder_Opt() = default;
//    SVFG* buildOptSVFG(BVDataPTAImpl* pta){
//        return build(pta, VFG::FULLSVFG_OPT);
//    }
//};
//
//void write_array_to_file(char *filename, PyArrayObject *array) {
//    int n = PyArray_SIZE(array);
//    void *data = PyArray_DATA(array);
//    FILE *file = fopen(filename, "wb");
//    if (file != NULL) {
//        fwrite(data, sizeof(double), n, file);
//        fclose(file);
//    }
//}
//
//int main(int argc, char** argv)
//{
//    int num_nodes = 4;
//    int num_edges = 5;
//    int edges[5][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 2}};
//
//    // 创建一个numpy数组来保存边
//    npy_intp dims[2] = {num_edges, 2};
//    PyArrayObject* edges_arr = (PyArrayObject*)PyArray_SimpleNewFromData(2, dims, NPY_INT, (void*)edges);
//
//    // 保存到文件
//    write_array_to_file("array.bin", edges_arr);
//    // 释放资源
//    Py_DECREF(edges_arr);
//
//    return 0;
//
//    char** arg_value = new char*[argc];
//    std::vector<std::string> moduleNameVec;
//    moduleNameVec =
//        OptionBase::parseOptions(argc, argv, "Whole Program Points-to Analysis",
//                                 "[options] <input-bitcode...>");
//
//    if (Options::WriteAnder() == "ir_annotator")
//    {
//        LLVMModuleSet::getLLVMModuleSet()->preProcessBCs(moduleNameVec);
//    }
//
//    SVFModule* svfModule = LLVMModuleSet::getLLVMModuleSet()->buildSVFModule(moduleNameVec);
//
//    /// Build Program Assignment Graph (SVFIR)
//    SVFIRBuilder builder(svfModule);
//    SVFIR* pag = builder.build();
//
//    /// Create Andersen's pointer analysis
//    Andersen* ander = AndersenWaveDiff::createAndersenWaveDiff(pag);
//
//    /// Sparse value-flow graph (SVFG)
//
//    SVFGBuilder_Opt svfBuilder(true);
//    SVFG* svfg = svfBuilder.buildOptSVFG(ander);
//
//    node_dict_type res = get_nodes_features(svfg);
//
//    generate_graph_dataset(svfg, svfModule, res);
//    //dump_nodes_feature(res, Options::featurePath());
//    //dump_graph(svfg, Options::graphPath(), get_new_node_id(svfg));
//    //svfg->dump(Options::graphPath(), true);
//
//    /// dump_edge_features(svfg, Options::featurePath());
//
//    AndersenWaveDiff::releaseAndersenWaveDiff();
//    SVFIR::releaseSVFIR();
//
//    SVF::LLVMModuleSet::releaseLLVMModuleSet();
//
//    llvm::llvm_shutdown();
//    delete[] arg_value;
//    return 0;
//}
////
//
//#include <stdio.h>
//#include <numpy/arrayobject.h>
//
//void write_array_to_file(char *filename, PyArrayObject *array) {
//    int nrows = PyArray_DIM(array, 0);
//    int ncols = PyArray_DIM(array, 1);
//    int elem_size = PyArray_ITEMSIZE(array);
//    void *data = PyArray_DATA(array);
//    FILE *file = fopen(filename, "wb");
//    if (file != NULL) {
//        for (int i = 0; i < nrows; i++) {
//            fwrite((char *)data + i * ncols * elem_size, elem_size, ncols, file);
//        }
//        fclose(file);
//    }
//}

//PyMODINIT_FUNC
//    init{name}(void)
//{
//    (void)Py_InitModule({name}, mymethods);
//    import_array();
#include"nlohmann/json.hpp"
#include"numpy_api.hpp"
int main() {
    nlohmann::json j;
    j[1] = 2;
    std::cout << j;
//    const std::vector<long unsigned> shape{2, 3};
//    const bool fortran_order{false};
//    const std::string path{"out.npy"};
//
//    const std::vector<double> data1{1, 2, 3, 4, 5, 6};
//    npy::SaveArrayAsNumpy(path, fortran_order, shape.size(), shape.data(), data1);
}