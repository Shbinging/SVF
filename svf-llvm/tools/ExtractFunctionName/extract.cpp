//
// Created by 水兵 on 2023/3/19.
//

#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Support/SourceMgr.h>
#include<iostream>
#include<stdio.h>
#include "rustc_demangle.h"
using namespace  llvm;


std::string demanlge_rust_funcname(const std::string oriName){
    char buf[300];
    printf("%s\n", oriName.c_str());
    if (rustc_demangle(oriName.c_str(), buf, 300) == 1){
        return std::string(buf);
    }else{
        return oriName;
    }
}

int main(int argc, char** argv){
    LLVMContext context;
    SMDiagnostic error;
    std::unique_ptr<Module> m(parseIRFile("/home/shuibing/ptCode2vec/dataset/compile_data/compile_time/test/test_llvm_ir/extract.svf.bc", error, context));
    if (m) {
        for(auto func_p = m->functions().begin(); func_p != m->functions().end(); func_p++){
            std::string func_ori_Name = func_p->getName().str();
            std::string func_mangle_Name = demanlge_rust_funcname(func_ori_Name);
            printf("%s\n",  func_mangle_Name.c_str());
        }

    }
    //delete m;
    return 0;
}