cmake_minimum_required(VERSION 3.20)
project(GLHydroSurface)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE "GLVND")

find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/SPlisHSPlasH/CMake")

include_directories(${GLFW3_INCLUDE_DIRS})
link_directories(${GLFW3_LIBRARY_DIRS})
add_definitions(${GLFW3_CFLAGS_OTHER})
add_definitions(-DDATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
add_definitions(-DEXTERNAL_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
add_definitions(-DEXTERNAL_SHADER_PATH="${CMAKE_CURRENT_SOURCE_DIR}/shaders")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

set(USE_GUI OFF CACHE BOOL "" FORCE)
set(USE_AVX OFF CACHE BOOL "" FORCE)
set(USE_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(USE_DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
set(USE_OPENMP ON CACHE BOOL "" FORCE)

set(Build_FoamGenerator OFF CACHE BOOL "" FORCE)
set(Build_MeshSkinning OFF CACHE BOOL "" FORCE)
set(Build_ParameterParser OFF CACHE BOOL "" FORCE)
set(Build_PartioViewer OFF CACHE BOOL "" FORCE)
set(Build_SurfaceSampling OFF CACHE BOOL "" FORCE)
set(Build_VolumeSampling OFF CACHE BOOL "" FORCE)
set(Build_partio2vtk OFF CACHE BOOL "" FORCE)

add_subdirectory(external/spdlog)
add_subdirectory(external/SPlisHSPlasH)

add_executable(GLHydroSurface
        pch.h
        src/state.cpp
        src/state.h
        src/PassTimer.cpp
        src/PassTimer.h
        src/main.cpp
        src/Shader.h
        src/Camera.h
        imgui/imgui.cpp
        imgui/imgui_draw.cpp
        imgui/imgui_tables.cpp
        imgui/imgui_widgets.cpp
        imgui/imgui_impl_glfw.cpp
        imgui/imgui_impl_opengl3.cpp
        src/sph/SPHIntegrationSim.h
        src/sph/SPHIntegrationSim.cpp
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
)
if(MSVC)
    target_compile_options(GLHydroSurface PRIVATE $<$<CONFIG:Release>:/O2 /fp:fast /LTCG>)
    target_compile_features(GLHydroSurface PRIVATE cxx_std_20)
    target_link_options(GLHydroSurface PRIVATE $<$<CONFIG:Release>:/LTCG>)
else()
    set(COMMON_RELEASE_FLAGS "-O3;-ffast-math;-flto")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(COMMON_RELEASE_FLAGS "${COMMON_RELEASE_FLAGS};-mcpu=native;-mtune=native")
    endif()

    target_compile_options(GLHydroSurface PRIVATE
            $<$<CONFIG:Release>:${COMMON_RELEASE_FLAGS}>
            ${OpenMP_CXX_FLAGS}
    )

    target_link_options(GLHydroSurface PRIVATE
            $<$<CONFIG:Release>:-flto -O3>
    )
endif()

target_precompile_headers(GLHydroSurface PRIVATE pch.h)

target_include_directories(GLHydroSurface PRIVATE ${OPENGL_INCLUDE_DIRS} OpenMP::OpenMP_CXX ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(GLHydroSurface PRIVATE glad::glad OpenGL::GL ${GLFW3_LIBRARIES} OpenMP::OpenMP_CXX glm::glm glfw
        SPlisHSPlasH Utilities SimulatorBase $<$<PLATFORM_ID:Linux>:m> spdlog::spdlog)
