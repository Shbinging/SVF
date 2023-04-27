//
// Created by 水兵 on 2023/4/28.
//

#include "demangle.h"

typedef std::map<std::string, std::string> dictTy;
typedef std::map<SVFGNode*, dictTy> node_dict_type;

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

