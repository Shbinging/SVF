#include "nlohmann/json.hpp"
#include "Graphs/SVFG.h"
#include "SVF-LLVM/LLVMUtil.h"
#include "SVF-LLVM/SVFIRBuilder.h"
#include "SVFIR/SymbolTableInfo.h"
#include "Util/CommandLine.h"
#include "Util/Options.h"
#include "WPA/Andersen.h"
#include "Graphs/GenericGraph.h"
#include "graph.h"
#include "svf_util.h"
#include<algorithm>

using namespace nlohmann;
using namespace std;
using namespace SVF;


#define NodeTy(x, y) SVF::x* ptr = SVFUtil::dyn_cast<SVF::x>(y)
#define Is(x, y) (x::classof(y))
#define ISA(type) auto res = SVFUtil::dyn_cast<type>(node)



typedef map<const SVFInstruction*, SVFGNode*> inst2VFGNode_type;
typedef map<const SVFArgument*, SVFGNode*> arg2VFGNode_type;
typedef map<const SVFBasicBlock*, vector<uint64_t>> bb2VFGNodeId_type;
typedef std::map<std::string, std::string> dictTy;
typedef std::map<SVFGNode*, dictTy> node_dict_type;

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

json get_type_attr(const SVFType* ty, std::unordered_map<const SVFType*, string>& visNode){
#define ISTy(type) auto res = SVFUtil::dyn_cast<type>(ty)
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



typedef unordered_map<uint64_t, uint64_t> inst_or_param2hvfg_type;
string get_opcode(const SVFVar* LVar){
    const Value* val =
        LLVMModuleSet::getLLVMModuleSet()->getLLVMValue(
            LVar->getValue());
    const Instruction* ins = SVFUtil::dyn_cast<Instruction>(val);
    assert(ins);
    return std::string(ins->getOpcodeName());
}



class typeg_ty:public graph<const SVFType*, json, json>{
public:
    std::unordered_map<const SVFType*, string> visNode;
    int cur_id = 0;

    uint64_t get_ty_uid(const SVFType* ty){
        return Name(ty);
    }

    string get_ty_sig(const SVFType* ty){
        dfs_type(ty);
        assert(nodes[ty].contains("ty_sig"));
        return nodes[ty]["ty_sig"].get<string>();
    }

    json get_ty_attr(const SVFType* ty){
        dfs_type(ty);
        return nodes[ty];
    }

    void dfs_type(const SVFType* ty){
        if (ty->getKind())
            if (visNode.find(ty) != visNode.end()){
                return;
            }
        visNode[ty] = "self";
        addNode(ty, json());
        if (ISTy(SVFIntergerType)){

        }else if(ISTy(SVFPointerType)){
            dfs_type(res->getPtrElementType());
            addEdge(ty, res->getPtrElementType(), json{{"edge_type","is_point_of"}});
        }else if(ISTy(SVFArrayType)){
            int idx = 0;
            for(;true;idx++){
                if (res->getTypeInfo()->getOriginalElemType(idx)) {
                    dfs_type(res->getTypeInfo()->getOriginalElemType(idx));
                    addEdge(ty, res->getTypeInfo()->getOriginalElemType(idx), json{{"edge_type", "is_array_of"}});
                }else break;
            }
        }else if (ISTy(SVFStructType)){
            int idx = 0;
            for(; true;idx++){
                if (res->getTypeInfo()->getOriginalElemType(idx)) {
                    dfs_type(res->getTypeInfo()->getOriginalElemType(idx));
                    addEdge(ty, res->getTypeInfo()->getOriginalElemType(idx), json{{"edge_type", "is_struct_of"}});
                }else break;
            }
        }else if (ISTy(SVFFunctionType)){
            //TODO

        }else if (ISTy(SVFOtherType)){
            //TODO
        }
        nodes[ty] = get_type_attr(ty);
    }
    json get_type_attr(const SVFType* ty){
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
        visNode[ty] = attr["ty_sig"].get<string>();
        return attr;
    }
};

class hvfg_ty:public graph<uint32_t, json, json> {
public:
    typedef unordered_map<SVFGNode*, uint32_t> svfNode2hvfNode_ty;
    typedef unordered_map<uint32_t, SVFGNode*> hvfNode2svfNode_ty;
    typedef uint32_t nodeIdx_ty;

    void bind_svfNode(SVFGNode* svfNode, uint32_t hvfNode){
        hvfNode2svfNode[hvfNode] = svfNode;
        svfNode2hvfNode[svfNode] = hvfNode;
    }

    bool is_use_svfgnode(SVFGNode* svfgNode){
        return svfNode2hvfNode.find(svfgNode) != svfNode2hvfNode.end();
    }

    nodeIdx_ty get_svfgnode2_hvfgNode(SVFGNode* svfgNode){
        assert(is_use_svfgnode(svfgNode));
        return svfNode2hvfNode[svfgNode];
    }

    bool has_bind_svfgNode(nodeIdx_ty hvfgNode){
        return hvfNode2svfNode.find(hvfgNode) != hvfNode2svfNode.end();
    }

    const SVFGNode* get_bind_svfgNode(uint32_t hvfgNode){
        assert(has_bind_svfgNode(hvfgNode));
        return hvfNode2svfNode[hvfgNode];
    }

    uint32_t get_bind_svfgNode_id(uint32_t hvfgNode){
        return get_bind_svfgNode(hvfgNode)->getId();
    }

    string get_hvfgNode_type(uint32_t idx){
        assert(hasNode(idx));
        return getNode(idx)["type"].get<string>();
    }

    void add_hvfgNode(uint32_t uid, string typ, string data, json attr){
        json node;
        node["type"] = typ;
        node["uid"] = uid;
        node["data"] = data;
        node["attr"] = attr;
        addNode(uid, node);
    }

    void add_inst_hvfgNode(uint32_t uid, string typ, string data, json attr, SVFGNode* svfgNode){
        add_hvfgNode(uid, typ, data, attr);
        bind_svfNode(svfgNode, uid);
    }

    bool is_GN_hvfgNode(uint32_t idx){
        return get_hvfgNode_type(idx) == "GN";
    }

    bool is_MN_hvfgNode(uint32_t idx){
        return get_hvfgNode_type(idx) == "MN";
    }

    bool is_TVN_hvfgNode(uint32_t idx){
        return get_hvfgNode_type(idx) == "TVN";
    }

    bool is_TN_hvfgNode(uint32_t idx){
        return get_hvfgNode_type(idx) == "TN";
    }

    bool is_inst_hvfgNode(uint32_t idx){
        if (!hasNode(idx)) return false;
        string typ = getNode(idx)["type"].get<string>();
        return (typ == "GN") || (typ == "MN") || (typ == "TVN");
    }

    bool is_CN_hvfgNode(uint32_t idx){
        return get_hvfgNode_type(idx) == "CN";
    }


private:
    hvfNode2svfNode_ty hvfNode2svfNode;
    svfNode2hvfNode_ty svfNode2hvfNode;
};

hvfg_ty* svfg2hvfnode(SVFG* svfg){
    auto hvfg = new hvfg_ty();
    uint64_t uid = 0;
    for(uint32_t i = 0; i < svfg->getSVFGNodeNum(); i++){
        auto node = svfg->getSVFGNode(i);
        if (node->getICFGNode()->getBB() == nullptr){
            //TODO
            if (!node->getValue() || !SVFUtil::isa<SVFGlobalValue>(node->getValue())) continue;
            //FIXME(Deal with global)
            if (!SVFUtil::isa<AddrVFGNode>(node)) continue;
            json attr = {
                {"inst_full",node->getValue()->toString()},
                {"name", node->getValue()->getName()},
                {"type_uid", Name(node->getValue()->getType())}
            };
            hvfg->add_inst_hvfgNode(uid++, "GN", "glob", attr, node);

        }else {
            if (ISA(MSSAPHISVFGNode)) {
                json node_attr = json();
                json attr = {
                    {"name", ""}
                };
                hvfg->add_inst_hvfgNode(uid++, "MN", "MPHI", attr, node);
            }
            auto var = getLHSTopLevPtr(node);
            if (var == nullptr || !var_has_val(var)) continue;
            if (ISA(FormalParmVFGNode)) {
                if (res->getParam() == nullptr) continue;
                json attr = json();
                attr["inst_full"] = res->getParam()->getValue()->toString();
                attr["name"] = res->getParam()->getValue()->getName();
                attr["type_uid"] = Name(res->getParam()->getType());
                hvfg->add_inst_hvfgNode(uid++, "TVN", "param", attr, node);

            } else if (ISA(ActualRetVFGNode)) {
                json attr = json();
                attr["inst_full"] = var->getValue()->toString();
                //name
                attr["name"] = var->getValueName();
                //Type
                attr["type_uid"] = Name(var->getType());
                hvfg->add_inst_hvfgNode(uid++, "TVN", "call", attr, node);

            } else if (ISA(BinaryOPVFGNode)) {
                json attr = json();
                attr["inst_full"] = var->getValue()->toString();
                //name
                attr["name"] = var->getValueName();
                attr["op_code"] = get_opcode(var);
                //Type
                attr["type_uid"] = Name(var->getType());
                hvfg->add_inst_hvfgNode(uid++, "TVN", get_opcode(var), attr, node);

            } else if (ISA(BranchVFGNode)) {
                json attr = json();
                attr["inst_full"] = var->getValue()->toString();
                attr["op_code"] = get_opcode(var);
                hvfg->add_inst_hvfgNode(uid++, "TVN", get_opcode(var), attr, node);

            } else if (ISA(CmpVFGNode)) {
                json attr = json();
                attr["inst_full"] = var->getValue()->toString();
                attr["op_code"] = get_opcode(var);
                //type
                attr["type_uid"] = Name(var->getType());
                //name
                attr["name"] = var->getValueName();
                hvfg->add_inst_hvfgNode(uid++, "TVN", get_opcode(var), attr, node);

            } else if (ISA(IntraPHIVFGNode)) {
                if (SVFUtil::isa<SVFFunction>(var->getValue())){
                    //ret node
                    json attr = json();
                    attr["inst_full"] = res->getFun()->getExitBB()->getTerminator()->toString();
                    attr["op_code"]  = "ret";
                    //type
                    attr["type_uid"] = Name(res->getFun()->getReturnType());
                    hvfg->add_inst_hvfgNode(uid++, "TVN", "RET", attr, node);

                }else {
                    json attr = json();
                    attr["inst_full"] = var->getValue()->toString();
                    attr["op_code"] = get_opcode(var);
                    //type
                    attr["type_uid"] = Name(var->getType());
                    //name
                    attr["name"] = var->getValueName();
                    hvfg->add_inst_hvfgNode(uid++, "TVN", get_opcode(var), attr, node);
                }
            } else if (ISA(StmtVFGNode)) {
                //FIXME(Deal with GLobal)
                if (SVFUtil::isa<SVFCallInst>(res->getInst())){
                    continue;
                }
                json node_attr = json();
                json attr = json();
                node_attr["uid"] = uid++;
                if (ISA(CopyVFGNode)){
                    node_attr["type"] = "TVN";
                }else if (ISA(GepVFGNode)){
                    node_attr["type"] = "TVN";
                }else{
                    node_attr["type"] = "MN";
                }
                if (ISA(StoreVFGNode)){
                    node_attr["data"] = "store";
                    attr["inst_full"] = res->getInst()->toString();
                    attr["op_code"] = "store";
                    node_attr["attr"] = attr;
                }else {
                    node_attr["data"] = get_opcode(var);
                    attr["inst_full"] = var->getValue()->toString();
                    attr["op_code"] = get_opcode(var);
                    //type
                    attr["type_uid"] = Name(var->getType());
                    //name
                    attr["name"] = var->getValueName();
                    node_attr["attr"] = attr;
                }


                hvfg->addNode(node_attr["uid"].get<uint32_t>(), node_attr);

                hvfg->bind_svfNode(node, node_attr["uid"]);
            } else if (ISA(UnaryOPVFGNode)) {
                json attr = json();
                attr["inst_full"] = var->getValue()->toString();
                attr["name"] = var->getValueName();
                attr["op_code"] = get_opcode(var);
                //type
                attr["type_uid"] = Name(var->getType());
                hvfg->add_inst_hvfgNode(uid++, "TVN", get_opcode(var), attr, node);
            }
        }
    }
    return hvfg;
}


void write_json_to_file(json& j, string output_path){
    auto buf = json::to_bjdata(j);
    std::ofstream outFile(output_path, std::ios::binary);
    outFile.write(reinterpret_cast<char*>(buf.data()),
                  sizeof(unsigned char) * buf.size());
    outFile.close();
}


class shrink_svfg_ty:public del_graph<uint32_t, bool, bool>{
private:
    unordered_map<uint32_t, SVFGNode *> shrink2svfg_node;
    SVFG* svfg_back;
public:
    SVFGNode *get_bind_svfgNode(uint32_t node_idx){
        assert(shrink2svfg_node.find(node_idx) != shrink2svfg_node.end());
        return  shrink2svfg_node[node_idx];
    }

    void build_graph_from_svfg(SVFG* svfg, hvfg_ty* hvfg){
        svfg_back = svfg;
        for(uint32_t i = 0; i < svfg->getSVFGNodeNum(); i++){
            auto node = svfg->getSVFGNode(i);
            uint32_t node_id = node->getId();
            shrink2svfg_node[node_id] = node;
            bool notDel = false;
            addNode(node_id, hvfg->is_use_svfgnode(node) || notDel);
        }
        for(uint32_t i = 0; i < svfg->getSVFGNodeNum(); i++){
            auto node = svfg->getSVFGNode(i);
            uint32_t node_id = node->getId();
            for(auto edge:node->getInEdges()){
                auto pre = edge->getSrcNode();
                uint32_t pre_id = pre->getId();
                bool is_ind = edge->isRetIndirectVFGEdge() || edge->isIndirectVFGEdge() || edge->isCallIndirectVFGEdge();
                addEdge(pre_id, node_id, is_ind);
            }
        }
    }

    void rm_DEL_node(){
        int s = 0;
        for(auto node_idx: getNodeIdxList()){
            if (!getNode(node_idx)){
                flush(cout);
                int ss = get_succ(node_idx).size() * get_pred(node_idx).size();
                s += ss;
                //FIXME::we should del these nodes in the feature
                //continue;
                if (ss > 10000){
                    continue;
                }
                auto succ = get_succ(node_idx);
                for(auto pre: get_pred(node_idx)){
                    bool edge1_ty = getEdge(pre, node_idx);
                    for(auto nxt: succ){
                        bool edge2_ty = getEdge(node_idx, nxt);
                        if (pre != nxt) addEdge(pre, nxt, edge1_ty | edge2_ty);
                    }
                }
                del_node(node_idx);
            }
        }
    }
};

void link_hvfg(SVFG* svfg, hvfg_ty* hvfg){
    shrink_svfg_ty* shrink_svfg = new shrink_svfg_ty();
    shrink_svfg->build_graph_from_svfg(svfg, hvfg);
    shrink_svfg->rm_DEL_node();
    for(auto node_idx:hvfg->getNodeIdxList()){
        if (!hvfg->has_bind_svfgNode(node_idx)) continue;
        auto svfg_node_idx = hvfg->get_bind_svfgNode_id(node_idx);
        int ord = 1;
        for(auto pre_node_idx : shrink_svfg->get_pred(svfg_node_idx)){
            auto svfg_node = shrink_svfg->get_bind_svfgNode(pre_node_idx);
            //FIXME:: we should del these nodes in the feature
            if (!hvfg->is_use_svfgnode(svfg_node)) continue;

            assert(hvfg->is_use_svfgnode(svfg_node));
            bool is_ind = shrink_svfg->getEdge(pre_node_idx, svfg_node_idx);

            auto pre_idx = hvfg->get_svfgnode2_hvfgNode(svfg_node);
            if (is_ind){
                hvfg->addEdge(pre_idx, node_idx, json({{"type", "memory_use"}, {"ord", 0}}));
            }else{
                hvfg->addEdge(pre_idx, node_idx, json({{"type", "top_use"}, {"ord", ord++}}));
            }
        }
    }
}


json get_constant(const SVFVar* var){
    json j;
    j["is_constant"] = false;
    if (var == nullptr || !var_has_val(var)) return j;
    if (const SVFConstantInt* cint = SVFUtil::dyn_cast<SVFConstantInt>(var->getValue())){
        j["is_constant"] = true;
        j["type_uid"] = Name(cint->getType());
        j["value"] = to_string(cint->getSExtValue());
    }else if (const SVFConstantFP* cfp = SVFUtil::dyn_cast<SVFConstantFP>(var->getValue())){
        j["is_constant"] = true;
        j["type_uid"] = Name(cfp->getType());
        j["value"] = to_string(cfp->getFPValue());
    }
    return j;
}

void gen_const_node_then_add(uint32_t src, json& j, uint32_t&uid, hvfg_ty* hvfg){
    if (j["is_constant"].get<bool>()){
        json node;
        node["type"] = "CN";
        node["uid"] = uid++;
        node["data"] = j["value"].get<string>();
        json node_attr;
        node_attr["type_uid"] = j["type_uid"].get<uint64_t>();
        node["attr"] = node_attr;
        hvfg->addNode(node["uid"].get<uint32_t>(), node);
        json edge;
        edge["type"] = "const_use";
        edge["ord"] = 0;
        hvfg->addEdge(node["uid"].get<uint32_t>(), src, edge);
    }
}

void add_const_node(uint32_t node_idx, hvfg_ty* hvfg, uint32_t& uid){
    auto node = hvfg->get_bind_svfgNode(node_idx);
    auto var = getLHSTopLevPtr(node);
    if (ISA(FormalParmVFGNode)){
        //TODO
        //handle constant
        for(auto edge:res->getInEdges()){
            auto actual_param = edge->getSrcNode();
            assert(SVFUtil::isa<ActualParmVFGNode>(actual_param));
        }
    }
    else if (ISA(ActualRetVFGNode)){
        json j = get_constant(res->getRev());
        gen_const_node_then_add(node_idx, j, uid, hvfg);
    }
    else if (ISA(BinaryOPVFGNode)){
        for(uint32_t i = 0; i < res->getOpVerNum(); i++){
            json j = get_constant(var);
            gen_const_node_then_add(node_idx, j, uid, hvfg);
        }
    }
    else if (ISA(BranchVFGNode)){
        //no need
    }
    else if (ISA(CmpVFGNode)){
        for(uint32_t i = 0; i < res->getOpVerNum(); i++){
            json j = get_constant(var);
            gen_const_node_then_add(node_idx, j, uid, hvfg);
        }
    }
    else if (ISA(IntraPHIVFGNode)){
        for(uint32_t i = 0; i < res->getOpVerNum(); i++){
            json j = get_constant(var);
            gen_const_node_then_add(node_idx, j, uid, hvfg);
        }
    }
    else if (ISA(StmtVFGNode)){
        if (ISA(StoreVFGNode)) {
            json j = get_constant(res->getPAGSrcNode());
            gen_const_node_then_add(node_idx, j, uid, hvfg);
        }
    }
    else if (ISA(UnaryOPVFGNode)){
        for(uint32_t i = 0; i < res->getOpVerNum(); i++){
            json j = get_constant(var);
            gen_const_node_then_add(node_idx, j, uid, hvfg);
        }
    }
}

void add_constant_node2hvfg(hvfg_ty* hvfg){
    uint32_t uid = hvfg->getNodeIdxList().size();
    for(auto node_idx:hvfg->getNodeIdxList()){
        if(hvfg->is_inst_hvfgNode(node_idx)){
            //FIXME(Global Constant) current we don't dealwith global constant
            if (hvfg->is_GN_hvfgNode(node_idx)) continue;
            add_const_node(node_idx, hvfg, uid);
        }
    }
}

void add_type_node2hvfg(hvfg_ty* hvfg, typeg_ty* typeg){
    uint32_t uid = hvfg->getNxNodeIdx();
    for(auto node_idx:hvfg->getNodeIdxList()){
        if (hvfg->is_inst_hvfgNode(node_idx) || hvfg->is_CN_hvfgNode(node_idx)){
            assert(hvfg->getNode(node_idx).contains("attr"));
            if (hvfg->getNode(node_idx)["attr"].contains("type_uid")){
                SVFType* typ = (SVFType*)hvfg->getNode(node_idx)["attr"]["type_uid"].get<uint64_t>();
                assert(typ != nullptr);
                json node;
                node["type"] = "TN";
                node["uid"] = uid++;
                node["data"] = typeg->get_ty_sig(typ);
                json typ_attr = typeg->getNode(typ);
                //TODO(getType need add struct type Name)
                node["attr"] = typ_attr;
                hvfg->addNode(node["uid"].get<uint32_t>(), node);
                json edge;
                edge["ord"] = 0;
                edge["type"] = "type_use";
                hvfg->addEdge(node["uid"].get<uint32_t>(), node_idx, edge);
            }
        }
    }
}

void add_name_node2hvfg(hvfg_ty* hvfg){
    uint32_t uid = hvfg->getNxNodeIdx();
    for(auto node_idx : hvfg->getNodeIdxList()){
        if (hvfg->is_inst_hvfgNode(node_idx) || hvfg->is_TN_hvfgNode(node_idx)){
            assert(hvfg->getNode(node_idx).contains("attr"));
            if (hvfg->getNode(node_idx)["attr"].contains("name")){
                string name = hvfg->getNode(node_idx)["attr"]["name"];
                if (name.empty()) continue;
                json node;
                node["type"] = "NDN";
                node["uid"] = uid++;
                node["data"] = name;
                json node_attr;
                node_attr["name"] = name;
                node["attr"] = node_attr;
                json edge;
                edge["ord"] = 0;
                edge["type"] = "name_use";
                hvfg->addNode(node["uid"].get<uint32_t>(), node);
                hvfg->addEdge(node["uid"].get<uint32_t>(), node_idx, edge);
            }
        }
    }
}

void add_label_node(hvfg_ty* hvfg, uint32_t node_idx, string label_name, uint32_t& uid){
    json node;
    node["type"] = "LN";
    node["uid"] = uid++;
    node["data"] = label_name;
    json node_attr;
    node_attr["label_name"] = label_name;
    node["attr"] = node_attr;
    json edge;
    edge["type"] = "label_use";
    edge["ord"] = 0;
    hvfg->addNode(node["uid"].get<uint32_t>(), node);
    hvfg->addEdge(node["uid"].get<uint32_t>(), node_idx, edge);
}

void add_label_node2hvfg(hvfg_ty* hvfg){
    uint32_t uid = hvfg->getNxNodeIdx();
    for(auto node_idx : hvfg->getNodeIdxList()){
        if (hvfg->is_inst_hvfgNode(node_idx)){
            assert(hvfg->has_bind_svfgNode(node_idx));
            auto node = hvfg->get_bind_svfgNode(node_idx);
            if (ISA(BranchVFGNode)){
                for(uint32_t i = 0; i < res->getBranchStmt()->getNumSuccessors(); i++){
                    add_label_node(hvfg, node_idx, res->getBranchStmt()->getSuccessor(i)->getBB()->getName(), uid);
                }
            }
            else if (ISA(IntraPHIVFGNode)){
                if (SVFUtil::isa<SVFFunction>(res->getValue())) continue;
                if (SVFUtil::isa<PhiStmt>(res->getICFGNode()->getSVFStmts().begin().operator*())) {
                    auto phi = SVFUtil::cast<PhiStmt>(res->getICFGNode()->getSVFStmts().begin().operator*());
                    for (uint32_t i = 0; i < phi->getOpVarNum(); i++) {
                        add_label_node(
                            hvfg, node_idx,
                            phi->getOpICFGNode(i)->getBB()->getName(), uid);
                    }
                }
            }
        }
    }
}

void dump_bb_graph(SVFModule* svfModule, SVFG* svfg, hvfg_ty* hvfg, string output_path){
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
            vector<uint32_t> node_list;
            for(auto svfNode:bb2VfgNodeId[bb]){
                auto node = (SVFGNode*)(svfNode);
                if (!hvfg->is_use_svfgnode(node)) continue;
                node_list.push_back(hvfg->get_svfgnode2_hvfgNode(node));
            }
            j1.push_back({{"uid", Name(bb)}, {"bb_name", bb->getName()}, {"func_name", bb->getFunction()->getName()}, {"hvfg_nodes", node_list}});
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

    hvfg_ty* hvfg = svfg2hvfnode(svfg);
    link_hvfg(svfg, hvfg);
    add_constant_node2hvfg(hvfg);
    typeg_ty* typeg = new typeg_ty();
    add_type_node2hvfg(hvfg, typeg);
    add_name_node2hvfg(hvfg);
    add_label_node2hvfg(hvfg);
    auto j = hvfg->dump();
    write_json_to_file(j, Options::output_path() + "hvfg.bjd");
    auto j_ty = typeg->dump();
    write_json_to_file(j_ty, Options::output_path() + "typeg.bjd");
    dump_bb_graph(svfModule, svfg, hvfg, Options::output_path() + "bbg.bjd");
    dump_call_graph(svfg, Options::output_path() + "cg.bjd");
    /*TODO
     * 0. link node (ok) huge link is out of time, need fix it
     * 1. handle globalNode(ok) globalNode is alloca something, it will use store to initialize
     * 2. add ret node (OK)
     * 3. add constant: current we don't deal with array/struct/global constant value(for initialize and operand) （OK）
     * 4. add type node (OK)
     * 5. add name node (OK)
     * 6. add label node (OK)
     * 7. dump bbg (OK)
     * 8. dump typeg (OK)
     * 9. dump cg (OK)
     * 10. handle ord
     * 11. handle global constant
     * outside BB, we should retain alloca global
     * inside BB, we should retain getelementptr, llvm memcpy(this enable init variable use global)
     */


    AndersenWaveDiff::releaseAndersenWaveDiff();
    SVFIR::releaseSVFIR();

    SVF::LLVMModuleSet::releaseLLVMModuleSet();

    llvm::llvm_shutdown();
    delete[] arg_value;
    return 0;
}

