if(DEFINED IN_SOURCE_BUILD)
    add_llvm_tool(ivfg_augmentation augmentation.cpp)
else()
    link_directories("/home/shuibing/ptCode2vec/dataset/rustc-demangle/target/release/")

    add_executable(ivfg_augmentation augmentation.cpp rustc_demangle.h)

    target_link_libraries(ivfg_augmentation SvfLLVM ${llvm_libs} rustc_demangle)

    set_target_properties(ivfg_augmentation PROPERTIES RUNTIME_OUTPUT_DIRECTORY
            ${CMAKE_BINARY_DIR}/bin)

endif()
