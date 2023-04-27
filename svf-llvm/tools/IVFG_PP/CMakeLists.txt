if(DEFINED IN_SOURCE_BUILD)
    add_llvm_tool(ivfg_pp augmentation.cpp)
else()
    link_directories("/home/shuibing/ptCode2vec/dataset/rustc-demangle/target/release/")
    add_executable(ivfg_pp ivfg_pp.cpp rustc_demangle.h graph.h svf_util.cpp svf_util.h demangle.cpp demangle.h)
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    add_subdirectory(json)

    target_link_libraries(ivfg_pp SvfLLVM ${llvm_libs} rustc_demangle nlohmann_json::nlohmann_json)

    set_target_properties(ivfg_pp PROPERTIES RUNTIME_OUTPUT_DIRECTORY
            ${CMAKE_BINARY_DIR}/bin)

endif()
