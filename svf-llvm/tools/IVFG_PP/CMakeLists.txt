if(DEFINED IN_SOURCE_BUILD)
    add_llvm_tool(ivfg_pp augmentation.cpp)
else()
    #include_directories("nlohmann")
    add_compile_options("-fexceptions")
    add_compile_options("-frtti")
    link_directories("/home/shuibing/ptCode2vec/dataset/rustc-demangle/target/release/")
    add_executable(ivfg_pp ivfg_pp.cpp rustc_demangle.h)
#    set(Python3_EXECUTABLE "/home/shuibing/miniconda3/envs/code2vec/bin/python3.8")
#    set(Python3_INCLUDE_DIRS "/home/shuibing/miniconda3/envs/code2vec/include/python3.8")
#    set(Python3_LIBRARY_DIRS "/home/shuibing/miniconda3/envs/code2vec/lib")
#    set(Python3_LIBRARIES "/home/shuibing/miniconda3/envs/code2vec/lib/libpython3.8.so")
    #find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
    message("${CMAKE_MODULE_PATH}")
    include(FetchContent)

    FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
    FetchContent_MakeAvailable(json)

    target_link_libraries(ivfg_pp SvfLLVM ${llvm_libs} rustc_demangle nlohmann_json::nlohmann_json)

    set_target_properties(ivfg_pp PROPERTIES RUNTIME_OUTPUT_DIRECTORY
            ${CMAKE_BINARY_DIR}/bin)

endif()
