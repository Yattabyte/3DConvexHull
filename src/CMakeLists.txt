###########################
### Convex Hull Example ###
###########################
set(Module convexHullEx)

# Configure and acquire files
set(FILES
    # Header files
    window.hpp
    hull.hpp

    # Source files
    window.cpp
    hull.cpp
    main.cpp
)

# Create Library using the supplied files
add_executable(${Module} ${FILES})
target_include_directories(${Module} 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} 
    PUBLIC ${PROJECT_SOURCE_DIR}/external/glfw/
    PUBLIC ${PROJECT_SOURCE_DIR}/external/MiniGFX/external
    PUBLIC ${PROJECT_SOURCE_DIR}/external/MiniGFX/src
)

# Add library dependencies
add_dependencies(${Module} MiniGFXCore)
target_link_libraries(${Module} PUBLIC MiniGFXCore)
target_compile_features(${Module} PRIVATE cxx_std_17)
target_link_libraries(${Module} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${Module} PUBLIC glfw OpenGL::GL)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND "${CXX_COMPILER_VERSION}" LESS_EQUAL "9.0")
    target_link_libraries(${Module} PRIVATE c++experimental stdc++fs>)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_link_libraries(${Module} PRIVATE stdc++fs)
endif()

# Set all project settings
target_compile_Definitions(${Module} PRIVATE $<$<CONFIG:DEBUG>:DEBUG>)
set_target_properties(${Module} PROPERTIES VERSION ${PROJECT_VERSION})