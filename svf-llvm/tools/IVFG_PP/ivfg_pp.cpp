#include <cxxabi.h>
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
#include "Graphs/GenericGraph.h"


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
    assert(res);
    return std::string(st);
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
                abi::__cxa_demangle(catch_str.c_str(), nullptr, nullptr, &status);
            if (realname == nullptr) {
                continue;
            }
            assert(realname != nullptr);
            std::string realname_str(realname);
            replaceAll(it.second, catch_str, realname_str);
        }
    }
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
//    if (node->getNodeKind() == SVFGNode::VFGNodeK::Store){
//        printf("ok");
//    }
    dict["uid"] = std::to_string(Name(node));
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
        return store->getPAGSrcNode();
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

string get_nodes_constant(const SVFVar* var){
    if (var == nullptr || !var_has_val(var)) return "";
    if (const SVFConstantInt* cint = SVFUtil::dyn_cast<SVFConstantInt>(var->getValue())){
        json j;
        j["type_uid"] = Name(cint->getType());
        j["value"] = cint->getSExtValue();
        return j.dump();
    }else if (const SVFConstantFP* cfp = SVFUtil::dyn_cast<SVFConstantFP>(var->getValue())){
        json j;
        j["type_uid"] = Name(cfp->getType());
        j["value"] = cfp->getFPValue();
        return j.dump();
    }
    return "";
}

typedef std::map<SVFGNode*, dictTy> node_dict_type;
#define NodeTy(x, y) SVF::x* ptr = SVFUtil::dyn_cast<SVF::x>(y)
#define Is(x, y) (x::classof(y))
#define ISA(type) const type* res = SVFUtil::dyn_cast<type>(node)
node_dict_type get_nodes_features(SVFG* svfg) {
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
            } else {
                dict["dest_name"] = LVar->getValueName();
                // FIXME::SVF has bug when using LVar->getType
                dict["dest_type"] = node->getValue()->getType()->toString();
                if (node->getValue()->getType()){
                    dict["dest_type_uid"] = std::to_string(Name(node->getValue()->getType()));
                }
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
        if (NodeTy(BranchVFGNode, node)){
            //TODO
        }
        if (NodeTy(PHIVFGNode, node)) {
            dict["branch_num"] = std::to_string(ptr->getOpVerNum());
        }
        if (NodeTy(StoreVFGNode, node)){
            if (ptr->getInst() == nullptr){
                dict.erase("dest_type");
                dict.erase("dest_type_uid");
                dict["inst_full"] = "";
            }else{
                dict.erase("dest_type");
                dict.erase("dest_type_uid");
                dict["inst_full"] = ptr->getInst()->toString();
            }
            dict["inst_op"] = "store";
        }
        res[node] = dict;
    }
    node_dict_type dict = res;
    //generate constant information
    for (uint32_t i = 0; i < svfg->getSVFGNodeNum(); i++) {
        auto node = svfg->getSVFGNode(i);
        // global
        if (node->getICFGNode()->getBB() == nullptr) continue;
        auto var = getLHSTopLevPtr(node);
        // dummy vfgNode
        if (var == nullptr || !var_has_val(var)) continue;
        const SVFInstruction* inst = nullptr;
        if (ISA(BinaryOPVFGNode)) {
            json j = json::array();
            bool f = 0;
            for(uint32_t i = 0; i < res->getOpVerNum(); i++){
                string st = get_nodes_constant(res->getOpVer(i));
                if (st != ""){
                    j.push_back(st);
                    f = 1;
                }
            }
            if (f) dict[node]["constant"] = j.dump();
        } else if (ISA(BranchVFGNode)) { //br
            //TODO current we don't have label node
        } else if (ISA(CmpVFGNode)) {
            json j = json::array();
            bool f = 0;
            for(uint32_t i = 0; i < res->getOpVerNum(); i++){
                string st = get_nodes_constant(res->getOpVer(i));
                if (st != ""){
                    j.push_back(st);
                    f = 1;
                }
            }
            if (f) dict[node]["constant"] = j.dump();
        } else if (ISA(PHIVFGNode)) {
            //TODO current we don't have label node
            inst = nullptr;
        } else if (ISA(StmtVFGNode)) {
            if (const StoreVFGNode* sNode = SVFUtil::dyn_cast<StoreVFGNode>(node)){
                json j = json::array();
                bool f = 0;
                string st = get_nodes_constant(sNode->getPAGSrcNode());
                if (st != ""){
                        j.push_back(st);
                        f = 1;
                    }
                if (f){
                        dict[node]["constant"] = j.dump();
                }
            }
        } else if (ISA(UnaryOPVFGNode)) {
            json j = json::array();
            bool f = 0;
            for(uint32_t i = 0; i < res->getOpVerNum(); i++){
                string st = get_nodes_constant(res->getOpVer(i));
                if (st != ""){
                    j.push_back(st);
                    f = 1;
                }
            }
            if (f) dict[node]["constant"] = j.dump();
        }
    }
    return res;
}


typedef map<const SVFInstruction*, SVFGNode*> inst2VFGNode_type;
typedef map<const SVFArgument*, SVFGNode*> arg2VFGNode_type;
typedef map<const SVFBasicBlock*, vector<uint64_t>> bb2VFGNodeId_type;

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
    auto node_list = vector<uint64_t>();
    auto edge_list = vector<vector<uint64_t> >();
    json j1 = json::array();
    for (auto func : svfModule->getFunctionSet()) {
        //auto node_labels = vector<string>();
        for (auto bb : func->getBasicBlockList()) {
            node_list.push_back(Name(bb));
            assert(bb2VfgNodeId.find(bb) != bb2VfgNodeId.end());
            j1.push_back({{"uid", Name(bb)}, {"bb_name", bb->getName()}, {"func_name", bb->getFunction()->getName()}, {"vfg_nodes", bb2VfgNodeId[bb]}});
            for (auto nxt_bb : bb->getSuccessors()) {
                vector<uint64_t> edge = {Name(bb), Name(nxt_bb)};
                edge_list.push_back(edge);
            }
        }
        node_dict_type dict;
    }
    j = {{"node_list", node_list},
                          {"edge_list", edge_list},
                          {"node_attr", j1}};
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

#define EdgeK(x) \
    { SVFGEdge::VFGEdgeK::x, #x }
std::map<SVFGEdge::VFGEdgeK, std::string> edgeType2Str = {
    EdgeK(IntraDirectVF), EdgeK(IntraIndirectVF), EdgeK(CallDirVF), EdgeK(RetDirVF),
    EdgeK(CallIndVF), EdgeK(RetIndVF)
};

void dump_valueflow_graph(SVFG* svfg, string output_path) {
    auto fea = get_nodes_features(svfg);
    postprocesss_nodes_features(fea);
    json j;
    j["node_list"] = json::array();
    j["edge_list"] = json::array();
    j["node_attr"] = json::array();
    j["edge_attr"] = json::array();
    for (uint32_t i = 0; i < svfg->getSVFGNodeNum(); ++i) {
        auto node = svfg->getSVFGNode(i);
        j["node_list"].push_back(Name(node));
        assert(fea.find(node) != fea.end());
        j["node_attr"].push_back(fea[node]);
        int s = 0;
        json j1;
        for (auto edge : node->getInEdges()) {
            auto srcNode = edge->getSrcNode();
            j["edge_list"].push_back({Name(srcNode), Name(node)});
            j1["edge_type"] = edgeType2Str[(SVFGEdge::VFGEdgeK)edge->getEdgeKind()];
            j1["edge_ord"] = s++;
            j["edge_attr"].push_back(j1);
        }
    }
    auto buf = json::to_bjdata(j);
    std::ofstream outFile(output_path, std::ios::binary);
    outFile.write(reinterpret_cast<char*>(buf.data()),
                  sizeof(unsigned char) * buf.size());
    outFile.close();
}

void dump_call_graph(SVFG* svfg, string output_path){
    auto cg = svfg->getPTA()->getPTACallGraph();
    json j;
    j["node_list"] = json::array();
    j["edge_list"] = json::array();
    j["node_attr"] = json::array();
    //TODO::maybe can record callsite
    for(auto it : *cg){
        auto node = it.second;
        j["node_list"].push_back(Name(node));
        for(auto edge:node->getOutEdges()){
            auto dstNode = edge->getDstNode();
            j["edge_list"].push_back({Name(node), Name(dstNode)});
        }
        auto ja = json::array();
        for(auto bb:node->getFunction()->getBasicBlockList()){
            ja.push_back(Name(bb));
        }
        j["node_attr"].push_back({{"uid", Name(node)}, {"bb_nodes", ja}, {"func_name", node->getFunction()->getName()}});
    }
    auto buf = json::to_bjdata(j);
    std::ofstream outFile(output_path, std::ios::binary);
    outFile.write(reinterpret_cast<char*>(buf.data()),
                  sizeof(unsigned char) * buf.size());
    outFile.close();
}

#define ISTy(type) const type* res = SVFUtil::dyn_cast<type>(ty)
std::string get_type_string(const SVFType* ty)
{
    std::string str;
    llvm::raw_string_ostream rawstr(str);
    const Type* llvm_ty = LLVMModuleSet::getLLVMModuleSet()->getLLVMType(ty);
    if (llvm_ty == nullptr){
        return "None";
    }
    rawstr << *llvm_ty;
    std::string name = rawstr.str();
    if (name.find('=') != name.npos){
        return name.substr(0, name.find('='));
    }else return name;
}
#include<algorithm>
json get_type_attr(const SVFType* ty, std::unordered_map<const SVFType*, string>& visNode){
    json attr;
    if (ISTy(SVFIntergerType)){
        attr["uid"] = Name(ty);
        attr["ty_type"] = "int";
        attr["ty_name"] = get_type_string(ty);
        attr["ty_sig"] = get_type_string(ty);
    }else if(ISTy(SVFPointerType)){
        attr["uid"] = Name(ty);
        attr["ty_type"] = "pointer";
        attr["ty_name"] = get_type_string(ty);
        attr["ty_sig"] = std::string("(*") + visNode[res->getPtrElementType()] + std::string(")");
    }else if(ISTy(SVFArrayType)){
        attr["uid"] = Name(ty);
        attr["ty_type"] = "array";
        attr["ty_name"] = get_type_string(ty);
        vector<string> sub_ty_sigs;
        for(int idx = 0; true;idx++){
            const SVFType* ty_sub = res->getTypeInfo()->getOriginalElemType(idx);
            if (ty_sub) {
                sub_ty_sigs.push_back(visNode[ty_sub]);
            }else break;
        }
        sort(sub_ty_sigs.begin(), sub_ty_sigs.end());
        std::string str;
        llvm::raw_string_ostream rawstr(str);
        rawstr << "[";
        for(auto st: sub_ty_sigs){
            rawstr << st << ",";
        }
        rawstr << "]";
        attr["ty_sig"] = rawstr.str();
    }else if (ISTy(SVFStructType)){
        attr["uid"] = Name(ty);
        attr["ty_type"] = "struct";
        attr["ty_name"] = get_type_string(ty);
        vector<string> sub_ty_sigs;
        for(int idx = 0; true;idx++){
            const SVFType* ty_sub = res->getTypeInfo()->getOriginalElemType(idx);
            if (ty_sub) {
                sub_ty_sigs.push_back(visNode[ty_sub]);
            }else break;
        }
        sort(sub_ty_sigs.begin(), sub_ty_sigs.end());
        std::string str;
        llvm::raw_string_ostream rawstr(str);
        rawstr << "{";
        for(auto st: sub_ty_sigs){
            rawstr << st << "|";
        }
        rawstr << "}";
        attr["ty_sig"] = rawstr.str();
    }else if (ISTy(SVFFunctionType)){
        attr["uid"] = Name(ty);
        attr["ty_type"] = "func";
        attr["ty_name"] = "func";
        attr["ty_sig"] = "func";
        //TODO
    }else if (ISTy(SVFOtherType)){
        //TODO
        if (ty->toString() == "float" || ty->toString() == "double"){
            attr["uid"] = Name(ty);
            attr["ty_type"] = get_type_string(ty);
            attr["ty_name"] = get_type_string(ty);
            attr["ty_sig"] = get_type_string(ty);
        }else{
            attr["uid"] = Name(ty);
            attr["ty_type"] = "None";
            attr["ty_name"] = "None";
            attr["ty_sig"] = "None";
        }
    }else{
        assert(0 && "unkown kind of type");
    }
    visNode[ty] = attr["ty_sig"];
    return attr;
}

void dfs_type(const SVFType* ty, json& j, std::unordered_map<const SVFType*, string>& visNode){
    if (ty->getKind())
    if (visNode.find(ty) != visNode.end()){
        return;
    }
    visNode[ty] = "self";
    if (ISTy(SVFIntergerType)){

    }else if(ISTy(SVFPointerType)){
        j["edge_list"].push_back({Name(ty), Name(res->getPtrElementType())});
        j["edge_attr"].push_back({{"edge_type", "is_point_of"}});
        dfs_type(res->getPtrElementType(), j, visNode);
    }else if(ISTy(SVFArrayType)){
        int idx = 0;
        for(;true;idx++){
            if (res->getTypeInfo()->getOriginalElemType(idx)) {
                j["edge_list"].push_back({Name(ty), Name(res->getTypeInfo()->getOriginalElemType(idx))});
                j["edge_attr"].push_back({{"edge_type", "is_array_of"}});
                dfs_type(res->getTypeInfo()->getOriginalElemType(idx), j, visNode);
            }else break;
        }
    }else if (ISTy(SVFStructType)){
        int idx = 0;
        for(; true;idx++){
            if (res->getTypeInfo()->getOriginalElemType(idx)) {
                j["edge_list"].push_back({Name(ty), Name(res->getTypeInfo()->getOriginalElemType(idx))});
                j["edge_attr"].push_back({{"edge_type", "is_struct_of"}});
                dfs_type(res->getTypeInfo()->getOriginalElemType(idx), j, visNode);
            }else break;
        }
    }else if (ISTy(SVFFunctionType)){
        //TODO

    }else if (ISTy(SVFOtherType)){
        //TODO
    }
    j["node_list"].push_back(Name(ty));
    j["node_attr"].push_back(get_type_attr(ty, visNode));
}

void dump_type_graph(SVFG* svfg, string output_path){
    auto fea = get_nodes_features(svfg);
    std::unordered_map<const SVFType*, string> visNode;
    json j;
    j["node_list"] = json::array();
    j["edge_list"] = json::array();
    j["node_attr"] = json::array();
    j["edge_attr"] = json::array();
    for(auto it : fea){
        if (it.second.find("dest_type_uid") != it.second.end()){
            SVFType* ty = (SVFType*) std::stoull(it.second["dest_type_uid"]);
            dfs_type(ty, j, visNode);
        }
    }
    auto buf = json::to_bjdata(j);
    std::ofstream outFile(output_path, std::ios::binary);
    outFile.write(reinterpret_cast<char*>(buf.data()),
                  sizeof(unsigned char) * buf.size());
    outFile.close();
}

typedef GenericGraph<json, json> graph_ty;

typedef unordered_map<uint64_t, uint64_t> inst_or_param2hvfg_type;
graph_ty* build_hvfg_graph(SVFG* svfg){
    inst_or_param2hvfg_type  inst_or_param2hvfg;
    auto hvfg = new graph_ty();
    int uid = 0;
    for(uint32_t i = 0; i < svfg->getSVFGNodeNum(); i++){
        auto node = svfg->getSVFGNode(i);
        if (node->getICFGNode()->getBB() == nullptr){

        }else {
            auto var = getLHSTopLevPtr(node);
            if (var == nullptr || !var_has_val(var)) continue;
            if (ISA(FormalParmVFGNode)) {
                if (res->getParam() == nullptr) continue;
                json node_attr;
                json attr;
                node_attr["uid"] = uid++;
                node_attr["type"] = "TVN";
                node_attr["data"] = "param";
                attr["type_uid"] = Name(res->getParam()->getType());
                attr["inst_full"] = res->getParam()->getValue()->toString();
                attr["name"] = res->getParam()->getValue()->getName();
                node_attr["attr"] = attr;
            } else if (ISA(ActualRetVFGNode)) {
            } else if (ISA(BinaryOPVFGNode)) {
            } else if (ISA(BranchVFGNode)) {
            } else if (ISA(CmpVFGNode)) {
            } else if (ISA(MSSAPHISVFGNode)) {
            } else if (ISA(IntraPHIVFGNode)) {
            } else if (ISA(StmtVFGNode)) {
            } else if (ISA(UnaryOPVFGNode)) {
            }
        }
    }
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

    build_hvfg_graph(svfg);
//    dump_bb_graph(svfModule, svfg, Options::bb_graph_path());
//    dump_valueflow_graph(svfg, Options::valueflow_graph_path());
//    dump_call_graph(svfg, Options::call_graph_path());
//    dump_type_graph(svfg, Options::type_graph_path());


    AndersenWaveDiff::releaseAndersenWaveDiff();
    SVFIR::releaseSVFIR();

    SVF::LLVMModuleSet::releaseLLVMModuleSet();

    llvm::llvm_shutdown();
    delete[] arg_value;
    return 0;
}

