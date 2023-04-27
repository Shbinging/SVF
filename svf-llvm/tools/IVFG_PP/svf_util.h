//
// Created by 水兵 on 2023/4/28.
//

#ifndef SVF_SVF_UTIL_H
#define SVF_SVF_UTIL_H
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
#include "graph.h"
#include "svf_util.h"
using namespace nlohmann;
using namespace std;
using namespace SVF;

bool var_has_val(const SVFVar* var);

extern std::map<SVFGNode::VFGNodeK, std::string> nodeType2Str;

const PAGNode* getLHSTopLevPtr(const VFGNode* node);


#endif  // SVF_SVF_UTIL_H
