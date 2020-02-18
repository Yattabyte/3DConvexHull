###########
### SRC ###
###########
set(Module testProject)

# Configure and acquire files
set(FILES
    # Header files

    # Source files
    main.cpp
)

# Create Library using the supplied files
add_executable(${Module} ${FILES})
target_include_directories(${Module}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add library dependencies
target_compile_features(${Module} PRIVATE cxx_std_17)
target_link_libraries(${Module} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${Module} glfw)
target_link_libraries(${Module} OpenGL::GL)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    target_link_libraries(${Module} PRIVATE $<$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>:c++experimental stdc++fs>)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_link_libraries(${Module} PRIVATE stdc++fs)
endif()

# Set all project settings
target_compile_Definitions(${Module} PRIVATE $<$<CONFIG:DEBUG>:DEBUG>)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${Module})
set_target_properties(${Module} PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "$(SolutionDir)app"
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
    PDB_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
    VERSION ${PROJECT_VERSION}
)