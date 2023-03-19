if(DEFINED IN_SOURCE_BUILD)
    add_llvm_tool(ExtractFunctionName extract.cpp)
else()
    add_executable(ExtractFunctionName extract.cpp)

    target_link_libraries(ExtractFunctionName SvfLLVM ${llvm_libs})
    target_link_libraries(ExtractFunctionName rustc_demangle)
    set_target_properties(ExtractFunctionName PROPERTIES RUNTIME_OUTPUT_DIRECTORY
            ${CMAKE_BINARY_DIR}/bin)
endif()
