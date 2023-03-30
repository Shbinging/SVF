if(DEFINED IN_SOURCE_BUILD)
    add_llvm_tool(ivfg_augmentation augmentation.cpp)
else()
    add_executable(ivfg_augmentation augmentation.cpp)

    target_link_libraries(ivfg_augmentation SvfLLVM ${llvm_libs})

    set_target_properties(ivfg_augmentation PROPERTIES RUNTIME_OUTPUT_DIRECTORY
            ${CMAKE_BINARY_DIR}/bin)
endif()
