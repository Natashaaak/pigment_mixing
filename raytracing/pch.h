//----------------------------------------------------------------------------------------
/**
 * \file       pch.h
 * \brief      Precompiled header
 *
 *  Precompiled header with all used libraries
 *
 */
//----------------------------------------------------------------------------------------

#ifndef PCH_H
#define PCH_H

#pragma once
#define GLFW_INCLUDE_NONE
#include <glad/glad.h>
#include <GLFW/glfw3.h>
using uint = uint32_t;

#include "../imgui/imconfig.h"
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_opengl3.h"
#include "../imgui/imgui_impl_glfw.h"


#include <spdlog/spdlog.h>
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"


#include <omp.h>


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>


#include <iostream>
#include <fstream>
#include <sstream>


#include <chrono>


#include <vector>
#include <cstdlib>
#include <cmath>


#define OK 0

// #define WINDOW_WIDTH 1600.0f
// #define WINDOW_HEIGHT 900.0f
#define WINDOW_WIDTH 800.0f
#define WINDOW_HEIGHT 450.0f

#endif //PCH_H
