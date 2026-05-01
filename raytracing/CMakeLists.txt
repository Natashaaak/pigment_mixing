cmake_minimum_required(VERSION 3.20)
project(GLHydroSurface)

# --- Project Settings ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Policy for OpenGL preference
cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE "GLVND")

# --- Dependencies ---
find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# --- Definitions ---
add_definitions(-DDATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
add_definitions(-DEXTERNAL_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
add_definitions(-DEXTERNAL_SHADER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/shaders")

# --- External Subdirectories ---
add_subdirectory(external/spdlog)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_RELWITHDEBINFO_POSTFIX "rd")
set(CMAKE_MINSIZEREL_POSTFIX "ms")

# Fallback GLFW variables (required for some configurations)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE) 

# Add GLFW from the new location 
add_subdirectory(external/glfw)

if(TARGET glfw3 AND NOT TARGET glfw)
    add_library(glfw INTERFACE)
    target_link_libraries(glfw INTERFACE glfw3)
endif()

# --- Executable Definition ---
add_executable(GLHydroSurface
    pch.h
    src/state.cpp
    src/state.h
    src/PassTimer.cpp
    src/PassTimer.h
    src/main.cpp
    src/Shader.h
    src/Camera.h
    
    # ImGui source files (directly in project root) [cite: 3]
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_impl_glfw.cpp
    imgui/imgui_impl_opengl3.cpp
    
    # Simulation and Raymarching files [cite: 3, 4]
    src/mpm/MPMIntegrationSim.h
    src/mpm/MPMIntegrationSim.cpp
    src/raymarch/BinaryDensityGrid.cpp
    src/raymarch/BinaryDensityGrid.h
    src/raymarch/AABBc.cpp
    src/raymarch/AABBc.h
    src/raymarch/Classification.cpp
    src/raymarch/Classification.h
    src/raymarch/DepthProcessor.cpp
    src/raymarch/DepthProcessor.h
    src/raymarch/RayMarch.cpp
    src/raymarch/RayMarch.h
    src/raymarch/HDRLoader.cpp
    src/raymarch/HDRLoader.h
    src/raymarch/SpatulaMesh.cpp
    src/raymarch/SpatulaMesh.h
    src/mixbox/mixbox.h
    src/mixbox/mixbox.cpp
)

# --- Compiler & Linker Options ---
if(MSVC)
    target_compile_options(GLHydroSurface PRIVATE /O2 /fp:fast /LTCG)
    target_compile_features(GLHydroSurface PRIVATE cxx_std_20)
    target_link_options(GLHydroSurface PRIVATE /LTCG)
else()
    set(COMMON_RELEASE_FLAGS "-O3;-ffast-math;-flto")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(COMMON_RELEASE_FLAGS "${COMMON_RELEASE_FLAGS};-mcpu=native;-mtune=native")
    endif()

    target_compile_options(GLHydroSurface PRIVATE 
        ${COMMON_RELEASE_FLAGS}
        ${OpenMP_CXX_FLAGS}
    )
    target_link_options(GLHydroSurface PRIVATE -flto -O3)
endif()

# --- Include Directories ---
target_include_directories(GLHydroSurface PRIVATE 
    ${OPENGL_INCLUDE_DIRS} 
    OpenMP::OpenMP_CXX 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mixbox
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui                # ImGui headers [cite: 3]
    ${CMAKE_CURRENT_SOURCE_DIR}/external/glfw/include # GLFW headers
)

# --- Linking ---
target_link_libraries(GLHydroSurface PRIVATE 
    glad::glad 
    OpenGL::GL 
    OpenMP::OpenMP_CXX 
    glm::glm 
    glfw        # Link the target created by add_subdirectory(external/glfw) [cite: 5]
    spdlog::spdlog 
    matter_lib
    $<$<PLATFORM_ID:Linux>:m>
)

# Precompiled headers
target_precompile_headers(GLHydroSurface PRIVATE pch.h)