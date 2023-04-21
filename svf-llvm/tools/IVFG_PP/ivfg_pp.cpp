#include <cxxabi.h>
#include <stdio.h>
#include <regex>
#include "nlohmann/json.hpp"
#include "Graphs/SVFG.h"
#include "SVF-LLVM/LLVMUtil.h"
#include "SVF-LLVM/SVFIRBuilder.h"
#include "SVFIR/SymbolTableInfo.h"
#include "Util/CommandLine.h"
#include "Util/Options.h"
#include "WPA/Andersen.h"
#include "rustc_demangle.h"
using namespace nlohmann;
using namespace std;
using namespace SVF;
typedef std::map<std::string, std::string> dictTy;

static inline void ltrim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
                return !std::isspace(ch);
            }));
}

// trim from end (in place)
static inline void rtrim(std::string& s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
                         [](unsigned char ch) { return !std::isspace(ch); })
                .base(),
            s.end());
}

// trim from both ends (in place)
static inline void trim(std::string& s) {
    rtrim(s);
    ltrim(s);
}

void replaceAll(std::string& str, const std::string& from,
                const std::string& to) {
    if (from.empty()) return;
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();  // In case 'to' contains 'from', like
                                   // replacing 'x' with 'yx'
    }
}
std::string rustDemangle(std::string oriName) {
    char st[300];
    int res = rustc_demangle(oriName.c_str(), st, 300);
    return std::string(st);
    assert(res);
}

void demangleRustName(dictTy& dict) {
    std::regex regex_pattern("_ZN[a-zA-Z0-9.$_]+");
    for (auto& it : dict) {
        trim(it.second);
        std::string old_target, new_target;
        old_target = it.second;
        std::sregex_iterator sr_it(old_target.begin(), old_target.end(),
                                   regex_pattern);
        std::sregex_iterator end;
        while (sr_it != end) {
            std::string catch_str = (*sr_it)[0];
            replaceAll(catch_str, "_ret", "");
            replaceAll(it.second, catch_str, rustDemangle(catch_str));
            sr_it++;
        }
    }
}

void demangleCPPName(dictTy& dict) {
    std::regex regex_pattern("_Z[a-zA-Z0-9.$_]+");
    for (auto& it : dict) {
        std::string old_target, new_target;
        old_target = it.second;
        std::sregex_iterator sr_it(old_target.begin(), old_target.end(),
                                   regex_pattern);
        std::sregex_iterator end;
        while (sr_it != end) {
            std::string catch_str = (*sr_it)[0];
            replaceAll(catch_str, "_ret", "");
            sr_it++;
            s32_t status;
            char* realname =
                abi::__cxa_demangle(catch_str.c_str(), 0, 0, &status);
            if (realname == nullptr) {
                continue;
            }
            assert(realname != nullptr);
            std::string realname_str(realname);
            replaceAll(it.second, catch_str, realname_str);
        }
    }
}

static std::string dict2str(std::map<std::string, std::string> dict) {
    std::string str;
    std::stringstream rawstr(str);
    for (auto& it : dict) {
        trim(it.second);
    }
    if (Options::demangleRust()) {
        demangleRustName(dict);
    }
    if (Options::demangleCPP()) {
        demangleCPPName(dict);
    }
    // print
    rawstr << "node_feature: {";
    for (auto& item : dict) {
        // if (item.first != "inst_full" && item.first != "var_name" &&
        // item.first != "idx") continue;
        rawstr << "\"\"\"" << item.first << "\"\"\""
               << " : "
               << "\"\"\"" << item.second << "\"\"\", ";
    }
    rawstr << "}";
    return rawstr.str();
}

void fea_loc(SVFGNode* node, dictTy& dict) {
    if (node->getICFGNode()->getFun() == nullptr) {
        dict["func_name"] = "GLOBAL";
    } else {
        dict["func_name"] = node->getICFGNode()->getFun()->getName();
    }
    if (node->getICFGNode()->getBB() == nullptr) {
        dict["block_name"] = "GLOBAL";
    } else {
        dict["block_name"] = node->getICFGNode()->getBB()->getName();
    }
}

#define NodeK(x) \
    { SVFGNode::VFGNodeK::x, #x }
#define Name(x) ((uint64_t)x)
std::map<SVFGNode::VFGNodeK, std::string> nodeType2Str = {
    NodeK(Addr),   NodeK(Copy),      NodeK(Gep),       NodeK(Store),
    NodeK(Load),   NodeK(Cmp),       NodeK(BinaryOp),  NodeK(UnaryOp),
    NodeK(Branch), NodeK(TPhi),      NodeK(TIntraPhi), NodeK(TInterPhi),
    NodeK(MPhi),   NodeK(MIntraPhi), NodeK(MInterPhi), NodeK(FRet),
    NodeK(ARet),   NodeK(AParm),     NodeK(FParm),     NodeK(FunRet),
    NodeK(APIN),   NodeK(APOUT),     NodeK(FPIN),      NodeK(FPOUT),
    NodeK(NPtr),   NodeK(DummyVProp)};

void fea_nodeType(SVFGNode* node, dictTy& dict) {
    // FIXME::type maybe wrong
    dict["node_type"] = nodeType2Str[SVFGNode::VFGNodeK(node->getNodeKind())];
    dict["uid"] = Name(node);
}

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
        return store->getPAGDstNode();
    }
    return nullptr;
}

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
typedef std::map<SVFGNode*, dictTy> node_dict_type;
#define NodeTy(x, y) SVF::x* ptr = SVFUtil::dyn_cast<SVF::x>(y)
#define Is(x, y) (x::classof(y))

node_dict_type get_nodes_features(SVFG* svfg) {
    // std::ofstream fout;
    // fout.open(featruePath.c_str(), std::ios::out | std::ios::trunc);
    node_dict_type res;
    for (SVF::u32_t i = 0; i < svfg->getSVFGNodeNum(); i++) {
        SVFGNode* node = svfg->getSVFGNode(i);
        dictTy dict;
        fea_loc(node, dict);
        fea_nodeType(node, dict);
        dict["idx"] = std::to_string(i);
        bool is_triple_instructions =
            Is(BinaryOPVFGNode, node) || Is(CmpVFGNode, node) ||
            Is(PHIVFGNode, node) || Is(StmtVFGNode, node) ||
            Is(UnaryOPVFGNode, node);
        bool is_mr_instructions = Is(MRSVFGNode, node);
        bool is_arg_instructions = Is(ArgumentVFGNode, node);
        bool is_top_phi_instructions = Is(PHIVFGNode, node);
        // bool is_mem_phi_instructions = Is(MSSAPHISVFGNode, node);
        // bool is_other_instructions = Is(BranchVFGNode, node) ||
        // Is(NullPtrVFGNode, node) || Is(DummyVersionPropSVFGNode, node);
        if (is_triple_instructions) {
            const SVFVar* LVar = getLHSTopLevPtr(node);
            dict["dest_name"] = "";
            dict["dest_type"] = "";
            dict["inst_full"] = "";
            if (LVar == nullptr) {
                // FIXME::current if LVar is nullptr, then SVFGNode is
                // StoreSVFGNode, or LoadSVFGNode(?) about call @f , value is
                // SVFConstant...
                dict["inst_full"] = node->getValue()->toString();
            } else if (!var_has_val(LVar)) {
                // FIXME::currnet this means LVar is DummyObjectVar or
                // DummyTopLevelVar such as @llvm.memcpy, null ...
                if (node->getValue()) {
                    dict["dest_name"] = node->getValue()->getName();
                    // they don't have type
                    dict["dest_type"] = "";
                    dict["inst_full"] = node->getValue()->toString();
                }
                dict2str(dict);
            } else {
                dict["dest_name"] = LVar->getValueName();
                // FIXME::SVF has bug when using LVar->getType
                dict["dest_type"] = node->getValue()->getType()->toString();
                dict["inst_full"] = LVar->getValue()->toString();
                const Value* val =
                    LLVMModuleSet::getLLVMModuleSet()->getLLVMValue(
                        LVar->getValue());
                const Instruction* ins = llvm::dyn_cast<Instruction>(val);
                if (ins) {
                    dict["inst_op"] = ins->getOpcodeName();
                }
            }
        }
        if (is_arg_instructions || is_top_phi_instructions) {
            const SVFVar* LVar = getLHSTopLevPtr(node);
            assert(LVar != nullptr);
            if (var_has_val(LVar)) {
                dict["dest_name"] = LVar->getValueName();
                // FIXME::SVF has bug if use LVar->getType()->toString();
                dict["dest_type"] = node->getValue()->getType()->toString();
                dict["inst_full"] = LVar->getValue()->toString();
            } else {
                if (node->getValue()) {
                    dict["inst_full"] = node->getValue()->toString();
                }
            }
        }
        if (is_mr_instructions) {
            dict["mr_id"] = "";
            dict["mr_size"] = "";
            dict["mr_version"] = "";
            const MRVer* mr;
            if (NodeTy(MSSAPHISVFGNode, node)) {
                dict["branch_num"] = std::to_string(ptr->getOpVerNum());
                mr = ptr->getResVer();
            } else {
                if (NodeTy(ActualINSVFGNode, node)) {
                    mr = ptr->getMRVer();
                }
                if (NodeTy(ActualOUTSVFGNode, node)) {
                    mr = ptr->getMRVer();
                }
                if (NodeTy(FormalINSVFGNode, node)) {
                    mr = ptr->getMRVer();
                }
                if (NodeTy(FormalOUTSVFGNode, node)) {
                    mr = ptr->getMRVer();
                }
            }
            dict["mr_id"] = std::to_string(mr->getID());
            dict["mr_version"] = std::to_string(mr->getSSAVersion());
            dict["mr_size"] = std::to_string(mr->getMR()->getRegionSize());
        }
        if (NodeTy(PHIVFGNode, node)) {
            dict["branch_num"] = std::to_string(ptr->getOpVerNum());
        }
        res[node] = dict;
        //        fout << "Node" << static_cast<const void*>(node) << "\t";
        //        fout << dict2str(dict) << "\n";
    }
    // fout.close();
    return res;
}

void dump_nodes_feature(node_dict_type& dict, std::string feature_path) {
    std::ofstream fout;
    fout.open(feature_path.c_str(), std::ios::out | std::ios::trunc);
    for (auto& it : dict) {
        fout << "Node" << static_cast<const void*>(it.first) << "\t";
        fout << dict2str(it.second) << "\n";
    }
    fout.close();
}

void dfs_var_name(SVFGNode* node, node_dict_type& dict,
                  unordered_map<SVF::u32_t, int>& visited,
                  std::string var_name) {
    if (node == nullptr || visited.count(node->getId())) {
        return;
    }
    auto& node_dict = dict[node];
    if (node_dict.count("dest_name") && node_dict["dest_name"] != "") {
        var_name = node_dict["dest_name"];
    }
    if (var_name != "") {
        node_dict["var_name"] = var_name;
        visited[node->getId()] = 1;
        for (auto it = node->getOutEdges().begin();
             it != node->getOutEdges().end(); it++) {
            if ((*it)->isDirectVFGEdge()) {
                dfs_var_name((*it)->getDstNode(), dict, visited, var_name);
            }
        }
    }
}

void del_svfg_all_edges(SVFGNode* node, SVFG* svfg) {
    std::vector<SVF::GenericNode<SVF::VFGNode, SVF::VFGEdge>::EdgeType*> in,
        out;
    for (auto it = node->getInEdges().begin(); it != node->getInEdges().end();
         it++) {
        in.push_back(*it);
    }
    for (auto it : in) {
        svfg->removeSVFGEdge(it);
    }
    for (auto it = node->OutEdgeBegin(); it != node->OutEdgeEnd(); it++) {
        out.push_back(*it);
    }
    for (auto it : out) {
        svfg->removeSVFGEdge(it);
    }
}

void impl_var_name(SVFG* svfg, node_dict_type& dict) {
    unordered_map<SVF::u32_t, int> visited;
    for (SVF::u32_t i = 0; i < svfg->nodeNum; i++) {
        SVFGNode* node = svfg->getSVFGNode(i);
        dfs_var_name(node, dict, visited, "");
    }
    for (auto& it : dict) {
        if (!it.second.count("var_name")) {
            del_svfg_all_edges(it.first, svfg);
            svfg->removeSVFGNode(it.first);
        }
    }
}

std::map<SVFGNode*, int> get_new_node_id(SVFG* svfg, const SVFFunction* fun) {
    std::map<SVFGNode*, int> node2id;
    int s = 0;
    for (auto it = svfg->getVFGNodeBegin(fun); it != svfg->getVFGNodeEnd(fun);
         it++) {
        SVFGNode* node = (*it);
        node2id[node] = s++;
    }
    for (auto it = svfg->getVFGNodeBegin(fun); it != svfg->getVFGNodeEnd(fun);
         it++) {
        SVFGNode* node = (*it);
        for (auto it = node->OutEdgeBegin(); it != node->OutEdgeEnd(); it++) {
            SVFGNode* node = (*it)->getDstNode();
            if (node2id.find(node) == node2id.end()) {
                node2id[node] = s++;
            }
        }
    }
    return node2id;
}

void dump_graph(SVFG* svfg, const SVFFunction* fun, std::string graphPath,
                std::map<SVFGNode*, int>& node2id) {
    std::ofstream fout;
    fout.open(graphPath.c_str(), std::ios::out | std::ios::trunc);
    for (auto it = svfg->getVFGNodeBegin(fun); it != svfg->getVFGNodeEnd(fun);
         it++) {
        SVFGNode* node = (*it);
        fout << std::to_string(node2id[node]);
        for (auto it = node->OutEdgeBegin(); it != node->OutEdgeEnd(); it++) {
            SVFGNode* dst_node = (*it)->getDstNode();
            fout << " " << std::to_string(node2id[dst_node]);
        }
        fout << "\n";
    }
    fout.close();
}

void dump_varname(std::string featurePath, std::map<SVFGNode*, int>& node2id,
                  node_dict_type& dict) {
    std::ofstream fout;
    fout.open(featurePath.c_str(), std::ios::out | std::ios::trunc);
    for (auto it = node2id.begin(); it != node2id.end(); it++) {
        fout << std::to_string(it->second) << "," << dict[it->first]["var_name"]
             << "\n";
    }
    fout.close();
}

void generate_graph_dataset(SVFG* svfg, SVFModule* svfModule,
                            node_dict_type& dict) {
    for (auto it = svfModule->getFunctionSet().begin();
         it != svfModule->getFunctionSet().end(); it++) {
        const SVFFunction* func = (*it);
        cout << func->getName() << endl;
        if (!svfg->hasVFGNodes(func)) continue;
        std::map<SVFGNode*, int> node2id = get_new_node_id(svfg, func);
        // FIXME::cpp/rust need mangle name
        dump_graph(svfg, func, Options::graphPath() + func->getName(), node2id);
        dump_varname(Options::featurePath() + func->getName(), node2id, dict);
    }
}

class SVFGBuilder_Opt : public SVFGBuilder {
public:
    explicit SVFGBuilder_Opt(bool _SVFGWithIndCall = false)
        : SVFGBuilder(_SVFGWithIndCall) {}
    virtual ~SVFGBuilder_Opt() = default;
    SVFG* buildOptSVFG(BVDataPTAImpl* pta) {
        return build(pta, VFG::FULLSVFG_OPT);
    }
};

typedef map<const SVFInstruction*, SVFGNode*> inst2VFGNode_type;
typedef map<const SVFArgument*, SVFGNode*> arg2VFGNode_type;
typedef map<const SVFBasicBlock*, vector<uint64_t>> bb2VFGNodeId_type;
#define ISA(type) const type* res = SVFUtil::dyn_cast<type>(node)
bb2VFGNodeId_type map_BB2VFGNode(SVFModule* svfModule, SVFG* svfg) {
    inst2VFGNode_type inst2VFGNode;
    arg2VFGNode_type arg2VFGNode;
    for (uint32_t i = 0; i < svfg->getSVFGNodeNum(); i++) {
        auto node = svfg->getSVFGNode(i);
        // global
        if (node->getICFGNode()->getBB() == nullptr) continue;
        auto var = getLHSTopLevPtr(node);
        // dummy vfgNode
        if (var == nullptr || !var_has_val(var)) continue;
        bool isInst = 0;
        if (ISA(FormalParmVFGNode)) {
            auto val = SVFUtil::dyn_cast<SVFArgument>(var->getValue());
            // FIME::it is strange
            if (val == nullptr) continue;
            assert(val);
            arg2VFGNode[val] = node;
        } else if (ISA(ActualRetVFGNode)) {
            assert(SVFUtil::isa<SVFInstruction>(var->getValue()));
            inst2VFGNode[(SVFInstruction*)var->getValue()] = node;
        } else if (ISA(BinaryOPVFGNode)) {
            assert(SVFUtil::isa<SVFInstruction>(var->getValue()));
            inst2VFGNode[(SVFInstruction*)var->getValue()] = node;
        } else if (ISA(BranchVFGNode)) {
            assert(SVFUtil::isa<SVFInstruction>(var->getValue()));
            inst2VFGNode[(SVFInstruction*)var->getValue()] = node;
        } else if (ISA(CmpVFGNode)) {
            assert(SVFUtil::isa<SVFInstruction>(var->getValue()));
            inst2VFGNode[(SVFInstruction*)var->getValue()] = node;
        } else if (ISA(PHIVFGNode)) {
            assert(SVFUtil::isa<IntraPHIVFGNode>(node));
            if (SVFUtil::isa<SVFFunction>(var->getValue())) {
                inst2VFGNode[((SVFFunction*)var->getValue())
                                 ->getExitBB()
                                 ->getTerminator()] = node;
            } else {
                assert(SVFUtil::isa<SVFInstruction>(var->getValue()));
                inst2VFGNode[(SVFInstruction*)var->getValue()] = node;
            }
        } else if (ISA(StmtVFGNode)) {
            assert(SVFUtil::isa<SVFInstruction>(res->getInst()));
            inst2VFGNode[res->getInst()] = node;
        } else if (ISA(UnaryOPVFGNode)) {
            assert(SVFUtil::isa<SVFInstruction>(var->getValue()));
            inst2VFGNode[(SVFInstruction*)var->getValue()] = node;
        }
    }
    bb2VFGNodeId_type bb2VFGNodeId;
    for (auto func : svfModule->getFunctionSet()) {
        for (auto bb : func->getBasicBlockList()) {
            if (func->getEntryBlock() == bb) {
                bb2VFGNodeId[bb] = vector<uint64_t>();
                //add arg to basic block
                for(uint32_t i = 0; i < func->arg_size(); i++){
                    auto arg = func->getArg(i);
                    assert(arg2VFGNode.find(arg) != arg2VFGNode.end());
                    bb2VFGNodeId[bb].push_back(Name(arg2VFGNode[arg]));
                }
            } else if (false && func->getExitBB() == bb) {
                // TODO::current we don't insert ret node to exit block
            } else {
                bb2VFGNodeId[bb] = vector<uint64_t>();
                for(auto inst: bb->getInstructionList()){
                    if (inst2VFGNode.find(inst) == inst2VFGNode.end()){
                        /*XXX::this instruction is
                        1. ret void
                        2. call llvm_function
                         */
                        continue;
                    }
                    assert(inst2VFGNode.find(inst) != inst2VFGNode.end());
                    bb2VFGNodeId[bb].push_back(Name(inst2VFGNode[inst]));
                }
            }
        }
    }
    return bb2VFGNodeId;
}

void dump_bb_graph(SVFModule* svfModule, SVFG* svfg, string output_path){
    auto bb2VfgNodeId = map_BB2VFGNode(svfModule, svfg);
    json j;
    for (auto func : svfModule->getFunctionSet()) {
        auto node_list = vector<uint64_t>();
        auto edge_list = vector<vector<uint64_t> >();
        //auto node_labels = vector<string>();
        json j1 = json::array();
        for (auto bb : func->getBasicBlockList()) {
            node_list.push_back(Name(bb));
            assert(bb2VfgNodeId.find(bb) != bb2VfgNodeId.end());
            j1.push_back({{"uid", Name(bb)}, {"bb_label", bb->getName()}, {"vfg_nodes", bb2VfgNodeId[bb]}});
            for (auto nxt_bb : bb->getSuccessors()) {
                vector<uint64_t> edge = {Name(bb), Name(nxt_bb)};
                edge_list.push_back(edge);
            }
        }
        node_dict_type dict;
        j[func->getName()] = {{"node_list", node_list},
                              {"edge_list", edge_list},
                              {"node_attr", j1}};
    }
    auto buf = json::to_bjdata(j);
    std::ofstream outFile(output_path, std::ios::binary);
    outFile.write(reinterpret_cast<char*>(buf.data()),
                  sizeof(unsigned char) * buf.size());
    outFile.close();
}

void postprocesss_nodes_features(node_dict_type& node_dict){
    //mangle name
    for(auto& it:node_dict) {
        auto& dict = it.second;
        for (auto& it : dict) {
            trim(it.second);
        }
        if (Options::demangleRust()) {
            demangleRustName(dict);
        }
        if (Options::demangleCPP()) {
            demangleCPPName(dict);
        }
    }
}

void dump_valueflow_graph(SVFG* svfg, string output_path) {
    auto fea = get_nodes_features(svfg);
    postprocesss_nodes_features(fea);
    json j;
    j["node_list"] = json::array();
    j["edge_list"] = json::array();
    j["node_attr"] = json::array();
    for (uint32_t i = 0; i < svfg->getSVFGNodeNum(); ++i) {
        auto node = svfg->getSVFGNode(i);
        j["node_list"].push_back(Name(node));
        j["node_attr"].push_back(fea[node]);
        for (auto edge : node->getOutEdges()) {
            auto dstNode = edge->getDstNode();
            j["edge_list"].push_back({Name(node), Name(dstNode)});
        }
    }
    auto buf = json::to_bjdata(j);
    std::ofstream outFile(output_path, std::ios::binary);
    outFile.write(reinterpret_cast<char*>(buf.data()),
                  sizeof(unsigned char) * buf.size());
    outFile.close();
}

void tranverse_svfg(SVFG* svfg, node_dict_type& fea) {
    json j;
    j["node_list"] = json::array();
    j["edge_list"] = json::array();
    j["node_attr"] = json::array();
    for (uint32_t i = 0; i < svfg->getSVFGNodeNum(); ++i) {
        auto node = svfg->getSVFGNode(i);
        j["node_list"].push_back(Name(node));
        j["node_attr"].push_back(fea[node]);
        for (auto edge : node->getOutEdges()) {
            auto dstNode = edge->getDstNode();
            j["edge_list"].push_back({Name(node), Name(dstNode)});
        }
        //        for(auto it :fea[node]){
        //
        //        }
    }
    auto buf = json::to_bjdata(j);
    std::ofstream outFile("ivfg.bin", std::ios::binary);
    outFile.write(reinterpret_cast<char*>(buf.data()),
                  sizeof(unsigned char) * buf.size());
    outFile.close();
}

void tranverse_bb(SVFG* svfg, SVFModule* svfModule) {
    json j;
    for (auto func : svfModule->getFunctionSet()) {
        auto node_list = vector<uint64_t>();
        auto edge_list = vector<vector<uint64_t> >();
        auto node_labels = vector<string>();
        for (auto bb : func->getBasicBlockList()) {
            node_list.push_back(Name(bb));
            node_labels.push_back(bb->getName());
            for (auto nxt_bb : bb->getSuccessors()) {
                vector<uint64_t> edge = {Name(bb), Name(nxt_bb)};
                edge_list.push_back(edge);
            }
        }

        json j1 = json::array();
        for (auto& name : node_labels) {
            j1.push_back({{"label", name}});
        }
        node_dict_type dict;
        j[func->getName()] = {{"node_list", node_list},
                              {"edge_list", edge_list},
                              {"node_attr", j1}};
    }
    auto buf = json::to_bjdata(j);
    std::ofstream outFile("bb.bin", std::ios::binary);
    outFile.write(reinterpret_cast<char*>(buf.data()),
                  sizeof(unsigned char) * buf.size());
    outFile.close();
}


int main(int argc, char** argv) {
    char** arg_value = new char*[argc];
    std::vector<std::string> moduleNameVec;
    moduleNameVec =
        OptionBase::parseOptions(argc, argv, "Whole Program Points-to Analysis",
                                 "[options] <input-bitcode...>");

    if (Options::WriteAnder() == "ir_annotator") {
        LLVMModuleSet::getLLVMModuleSet()->preProcessBCs(moduleNameVec);
    }

    SVFModule* svfModule =
        LLVMModuleSet::getLLVMModuleSet()->buildSVFModule(moduleNameVec);

    /// Build Program Assignment Graph (SVFIR)
    SVFIRBuilder builder(svfModule);
    SVFIR* pag = builder.build();
    /// Create Andersen's pointer analysis
    Andersen* ander = AndersenWaveDiff::createAndersenWaveDiff(pag);

    /// Sparse value-flow graph (SVFG)

    SVFGBuilder svfBuilder(true);
    SVFG* svfg = svfBuilder.buildFullSVFG(ander);
    map_BB2VFGNode(svfModule, svfg);
    //    svfg->getPAG()->dump("pag-example.dot");
    //    svfg->getPAG()->getICFG()->dump("icfg-example.dot");
    //    svfg->dump("svfg-example.dot");
    //    tranverse_bb(svfg, svfModule);
    //    node_dict_type res = get_nodes_features(svfg);
    //    tranverse_svfg(svfg, res);
    // generate_graph_dataset(svfg, svfModule, res);

    // dump_nodes_feature(res, Options::featurePath());
    // dump_graph(svfg, Options::graphPath(), get_new_node_id(svfg));
    // svfg->dump(Options::graphPath(), true);

    /// dump_edge_features(svfg, Options::featurePath());

    AndersenWaveDiff::releaseAndersenWaveDiff();
    SVFIR::releaseSVFIR();

    SVF::LLVMModuleSet::releaseLLVMModuleSet();

    llvm::llvm_shutdown();
    delete[] arg_value;
    return 0;
}

// #include"nlohmann/json.hpp"
// #include"numpy_api.hpp"
// using namespace nlohmann;
// int main() {
//     json j;
//     j["pi"] = {1, 2, 3};
//     j["answer"]["everything"] = 42;
//     j["edge vector"] = {{1,2},{3,4},{5,6}, {7, 8}};
//     std::cout << j << std::endl;
//     auto buf = json::to_bjdata(j);
//     std::cout << buf.size() << std::endl;
//     std::ofstream outFile("output.bin", std::ios::binary);
//     outFile.write(reinterpret_cast<char*>(buf.data()), sizeof(unsigned char)
//     * buf.size()); outFile.close(); return 0;
////    const std::vector<long unsigned> shape{2, 3};
////    const bool fortran_order{false};
////    const std::string path{"out.npy"};
////
////    const std::vector<double> data1{1, 2, 3, 4, 5, 6};
////    npy::SaveArrayAsNumpy(path, fortran_order, shape.size(), shape.data(),
/// data1);
//}