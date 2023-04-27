//
// Created by 水兵 on 2023/4/28.
//

#ifndef SVF_GRAPH_H
#define SVF_GRAPH_H
#include<unordered_map>
#include <vector>
#include "nlohmann/json.hpp"
using namespace std;
using namespace nlohmann;

#define Name(x) ((uint64_t)x)

template<class IdxTy, class NodeTy, class EdgeTy>
class del_graph {
private:
    unordered_map<IdxTy, unordered_map<IdxTy, EdgeTy>> edges;
    //unordered_map<pair<IdxTy, IdxTy>, EdgeTy, pair_hash> edges;
    unordered_map<IdxTy, NodeTy> nodes;
    unordered_map<IdxTy, unordered_set<IdxTy>> succ, pred;
public:
    bool hasNode(IdxTy idx){
        return nodes.find(idx) != nodes.end();
    }

    bool hasEdge(IdxTy src, IdxTy dst){
        return edges.find(src) != edges.end() && (edges[src].find(dst) != edges[src].end());
    }
    int totalEdge(){
        int s = 0;
        for(auto idx: getNodeIdxList()){
            s += succ[idx].size();
        }
        return s;
    }
    const vector<IdxTy> getNodeIdxList(){
        auto res = vector<IdxTy>();
        for(auto iter:nodes){
            res.push_back(iter.first);
        }
        return res;
    }

    const NodeTy getNode(IdxTy idx){
        assert(hasNode(idx));
        return nodes[idx];
    }

    EdgeTy getEdge(IdxTy src, IdxTy dst){
        assert(hasEdge(src, dst));
        return edges[src][dst];
    }

    void addNode(IdxTy idx , NodeTy node){
        if (!hasNode(idx)){
            nodes[idx] = node;
            succ[idx] = unordered_set<IdxTy>();
            pred[idx] = unordered_set<IdxTy>();
            edges[idx] = unordered_map<IdxTy, EdgeTy>();
        }
    }

    void addEdge(IdxTy src, IdxTy dest, EdgeTy edge){
        assert(hasNode(src) && hasNode(dest));
        if (!hasEdge(src, dest)) {
            assert(succ[src].find(dest) == succ[src].end());
            assert(pred[dest].find(src) == pred[dest].end());
            edges[src][dest] = edge;
            succ[src].insert(dest);
            pred[dest].insert(src);
        }
    }

    uint64_t getNodeId(IdxTy idx){
        return Name(idx);
    }

    void del_node(IdxTy idx){
        assert(hasNode(idx));
        nodes.erase(idx);
        for(auto dst: succ[idx]){
            pred[dst].erase(idx);
            edges[idx].erase(dst);
        }
        for(auto src: pred[idx]){
            succ[src].erase(idx);
            edges[src].erase(idx);
        }
    }

    void del_edge(IdxTy src, IdxTy dst){
        assert(hasEdge(src, dst));
        edges[src].erase(dst);
        succ[src].erase(dst);
        pred[dst].erase(src);
    }

    const unordered_set<IdxTy> get_pred(IdxTy node){
        assert(hasNode(node));
        return pred[node];
    }

    const unordered_set<IdxTy> get_succ(IdxTy node){
        assert(hasNode(node));
        return succ[node];
    }
};

#include <functional>
template <typename T>
inline void hash_combine(std::size_t &seed, const T &val) {
    seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
// auxiliary generic functions to create a hash value using a seed
template <typename T> inline void hash_val(std::size_t &seed, const T &val) {
    hash_combine(seed, val);
}
template <typename T, typename... Types>
inline void hash_val(std::size_t &seed, const T &val, const Types &... args) {
    hash_combine(seed, val);
    hash_val(seed, args...);
}

template <typename... Types>
inline std::size_t hash_val(const Types &... args) {
    std::size_t seed = 0;
    hash_val(seed, args...);
    return seed;
}

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
        return hash_val(p.first, p.second);
    }
};


template<class IdxTy, class NodeTy, class EdgeTy>
class graph {
public:

    unordered_map<pair<IdxTy, IdxTy>, EdgeTy, pair_hash> edges;
    unordered_map<IdxTy, NodeTy> nodes;
    unordered_map<IdxTy, vector<IdxTy>> succ, pred;


    const vector<IdxTy> getNodeIdxList(){
        auto res = vector<IdxTy>();
        for(auto iter:nodes){
            res.push_back(iter.first);
        }
        return res;
    }

    const IdxTy getNxNodeIdx(){
        IdxTy mx = 0;
        for(auto iter:nodes){
            mx = max(mx, iter.first);
        }
        return mx + 1;
    }

    const NodeTy getNode(IdxTy idx){
        assert(hasNode(idx));
        return nodes[idx];
    }

    bool hasEdge(IdxTy src, IdxTy dst){
        return edges.find(make_pair(src, dst)) != edges.end();
    }

    const vector<IdxTy> get_pred(IdxTy node){
        assert(hasNode(node));
        return pred[node];
    }

    const vector<IdxTy> get_succ(IdxTy node){
        assert(hasNode(node));
        return succ[node];
    }

    bool hasNode(IdxTy idx){
        return nodes.find(idx) != nodes.end();
    }

    void addNode(IdxTy idx , NodeTy node){
        if (!hasNode(idx)) {
            nodes[idx] = node;
            succ[idx] = vector<IdxTy>();
            pred[idx] = vector<IdxTy>();
        }
    }

    void addEdge(IdxTy src, IdxTy dest, EdgeTy edge){
        edges[make_pair(src, dest)] = edge;
        succ[src].push_back(dest);
        pred[dest].push_back(src);
    }

    uint64_t getNodeId(IdxTy idx){
        return Name(idx);
    }
    pair<uint64_t, uint64_t> getEdgeId(pair<IdxTy, IdxTy> edge_idx){
        return make_pair(Name(edge_idx.first), Name(edge_idx.second));
    }
    NodeTy getNodeAttr(NodeTy node){
        return node;
    }
    EdgeTy getEdgeAttr(EdgeTy edge){
        return edge;
    }

    json dump(){
        json j;
        j["node_list"] = json::array();
        j["node_attr"] = json::array();
        j["edge_list"] = json::array();
        j["edge_attr"] = json::array();
        for(auto it: nodes){
            j["node_list"].push_back(getNodeId(it.first));
            j["node_attr"].push_back(getNodeAttr(it.second));
        }
        for(auto it:edges){
            j["edge_list"].push_back(getEdgeId(it.first));
            j["edge_attr"].push_back(getEdgeAttr(it.second));
        }
        return j;
    }
};



#endif  // SVF_GRAPH_H
