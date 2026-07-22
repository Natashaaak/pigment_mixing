cmake_minimum_required(VERSION 3.20)
project(pigment-mixing)

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

# Přidání mixboxu jako podadresáře, který se zkompiluje do vlastní knihovny
add_subdirectory(src/mixbox)

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
add_executable(pigment-mixing
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
    # mixbox.cpp a .h jsou nyní součástí cíle 'mixbox_library'
)

# --- Compiler & Linker Options ---
if(MSVC)
    target_compile_options(pigment-mixing PRIVATE /O2 /fp:fast)
    target_compile_features(pigment-mixing PRIVATE cxx_std_20)
else()
    set(COMMON_RELEASE_FLAGS "-O3;-ffast-math")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(COMMON_RELEASE_FLAGS "${COMMON_RELEASE_FLAGS};-mcpu=native;-march=native;-mtune=native")
    endif()

    target_compile_options(pigment-mixing PRIVATE 
        ${COMMON_RELEASE_FLAGS}
        ${OpenMP_CXX_FLAGS}
    )
    target_link_options(pigment-mixing PRIVATE -O3)
endif()

# --- Include Directories ---
target_include_directories(pigment-mixing PRIVATE 
    ${OPENGL_INCLUDE_DIRS} 
    OpenMP::OpenMP_CXX
    ${CMAKE_CURRENT_SOURCE_DIR}/src # src/mixbox bude přidán automaticky přes mixbox_library
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui                # ImGui headers [cite: 3]
    ${CMAKE_CURRENT_SOURCE_DIR}/external/glfw/include # GLFW headers
)

# --- Linking ---
target_link_libraries(pigment-mixing PRIVATE 
    glad::glad 
    OpenGL::GL 
    OpenMP::OpenMP_CXX 
    glm::glm 
    glfw        # Link the target created by add_subdirectory(external/glfw) [cite: 5]
    spdlog::spdlog 
    mixbox_library # Propojení s nově vytvořenou knihovnou mixbox
    matter_lib
    $<$<PLATFORM_ID:Linux>:m>
)

# Precompiled headers
target_precompile_headers(pigment-mixing PRIVATE pch.h)