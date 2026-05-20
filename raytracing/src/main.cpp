//----------------------------------------------------------------------------------------
/**
 * \file       main.cpp
 * \brief      Main file
 *
 *  Main application file that controls application start/finish, main loop
 *  GUI rendering
 */
//----------------------------------------------------------------------------------------

#include "state.h"

#include "Camera.h"
#include "PassTimer.h"
#include "raymarch/BinaryDensityGrid.h"
#include "raymarch/RayMarch.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "mpm/MPMIntegrationSim.h"
#include "../../deps/json.hpp"
#include "mixbox/mixbox.h"
#include <string>
#include <algorithm>
#include <vector>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <chrono>
#include <cmath>
#include <iomanip>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

GLFWwindow* window;

RayMarch *rm;
Camera *camera;
MPMIntegrationSim *mpm = nullptr;
AABBc *a;

void loadPigmentConfig(const std::string& path) {
    std::string config_path = path;
    std::ifstream file(config_path);
    if (!file.is_open() && config_path == "../colors_config.json") {
        config_path = "colors_config.json"; // Fallback
        file.open(config_path);
    }

    state.g_available_pigments.clear();
    if (!file.is_open()) {
        spdlog::error("Could not open pigment config file: {}. Using fallback colors.", config_path);
        state.g_available_pigments = {
            {"Yellow", {0.959f, 0.802f, 0.035f}},
            {"Blue", {0.077f, 0.028f, 0.248f}},
            {"White", {0.995f, 0.999f, 0.997f}},
            {"Magenta", {0.506f, 0.012f, 0.184f}}
        };
    } else {
        try {
            nlohmann::json j;
            file >> j;
            if (j.contains("pigments")) {
                for (const auto& p : j["pigments"]) {
                    Pigment pigment;
                    pigment.name = p["name"].get<std::string>();
                    auto rgb = p["rgb"].get<std::vector<float>>();
                    if (rgb.size() == 3) {
                        pigment.rgb[0] = rgb[0];
                        pigment.rgb[1] = rgb[1];
                        pigment.rgb[2] = rgb[2];
                        state.g_available_pigments.push_back(pigment);
                    }
                }
            }
        } catch (const std::exception& e) {
            spdlog::error("Error parsing pigment config {}: {}", config_path, e.what());
        }
    }

    // Prepare names for ImGui and update initial g_colors
    state.g_pigment_names.clear();
    for (const auto& pigment : state.g_available_pigments) {
        state.g_pigment_names.push_back(pigment.name.c_str());
    }

    // Set initial colors based on default indices
    for (int i = 0; i < 4; ++i) {
        if (i < state.g_available_pigments.size()) {
            // Ensure default indices are valid
            state.g_selected_pigment_indices[i] = std::min(state.g_selected_pigment_indices[i], (int)state.g_available_pigments.size() - 1);
            const auto& pigment = state.g_available_pigments[state.g_selected_pigment_indices[i]];
            state.g_colors[i][0] = pigment.rgb[0];
            state.g_colors[i][1] = pigment.rgb[1];
            state.g_colors[i][2] = pigment.rgb[2];
        }
    }
}

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    camera->setAspect(width, height);
}

void saveImage(const std::string& directory, GLFWwindow* win) {
    int width, height;
    glfwGetFramebufferSize(win, &width, &height);

    std::vector<unsigned char> pixels(width * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    // OpenGL reads from bottom-to-top, flip vertically
    std::vector<unsigned char> flippedPixels(width * height * 3);
    for (int y = 0; y < height; ++y) {
        std::copy(pixels.begin() + y * width * 3, 
                  pixels.begin() + (y + 1) * width * 3, 
                  flippedPixels.begin() + (height - 1 - y) * width * 3);
    }

    static int frame_counter = 0;
    std::stringstream ss;
    ss << "render_" << std::setfill('0') << std::setw(5) << frame_counter++ << ".png";
    std::string filename = ss.str();

    try {
        if (!directory.empty()) {
            std::filesystem::create_directories(directory);
        }
        std::string filepath = directory.empty() ? filename : directory + "/" + filename;
        if (stbi_write_png(filepath.c_str(), width, height, 3, flippedPixels.data(), width * 3)) {
            state.screenshotsTaken = true;
        } else {
            debug("Failed to open file for image saving: ", filepath);
        }
    } catch (const std::exception& e) {
        debug("Error while saving image: ", e.what());
    }
}

void processInput(GLFWwindow *window, int key, int scancode, int action, int mods) {
    if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
        state.play = !state.play;
    if (key == GLFW_KEY_S && action == GLFW_PRESS) {
        camera->setPos(camera->cameraPos - (camera->camForward * 0.5f));
    }
    if (key == GLFW_KEY_W && action == GLFW_PRESS) {
        camera->setPos(camera->cameraPos + (camera->camForward * 0.5f));
    }
    if (key == GLFW_KEY_P && action == GLFW_PRESS) {
        state.takeScreenshot = !state.takeScreenshot;
        debug("Take screenshot: ", state.takeScreenshot);
    }
    if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        state.fullRender = !state.fullRender;
        debug("Full render: ", state.fullRender);
    }
}

void mouseButtonCallback(GLFWwindow* w, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            camera->rot = true;
            glfwGetCursorPos(w, &camera->lastX, &camera->lastY);
        } else if (action == GLFW_RELEASE) {
            camera->rot = false;
        }
    }
}

void cursorPosCallback(GLFWwindow* w, double x, double y) {
    if (!camera->rot)
        return;
    camera->yaw -= static_cast<float>(x - camera->lastX) * camera->sense;
    camera->pitch += static_cast<float>(y - camera->lastY) * camera->sense;
    camera->lastX = x;
    camera->lastY = y;
}

void glfw_error_callback(int error, const char* description) {
    std::cerr << "--- GLFW Error (" << error << ") ---" << std::endl;
    std::cerr << "Description: " << description << std::endl;
    std::cerr << "----------------------------------" << std::endl;
}

int createWindow() {
    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit()) {
        std::cerr << "ERROR: glfwInit failed" << std::endl;
        return -1;
    }

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // Přidání 4x MSAA
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Pigment Mixing", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "ERROR: glfw window not created" << std::endl;
        const char* description;
        int code = glfwGetError(&description);
        if (description) {
            std::cerr << "Error: " << description << " (code: " << code << ")" << std::endl;
        }
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetKeyCallback(window, processInput);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);


    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cerr << "ERROR: Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glEnable(GL_MULTISAMPLE); // Povolení MSAA

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImFontConfig ifc;
    ifc.SizePixels = 24.0f;
    io.Fonts->AddFontDefault(&ifc);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 450");
    return OK;
}

#ifdef MEASURE_TIME
static double frame_sim_time_ms = 0.0;
#endif

/**
 * Performs one simulation step if simulation is unpaused
 */
void sim() {
    if (state.play) {
#ifdef MEASURE_TIME
        auto start = std::chrono::high_resolution_clock::now();
        ctimer.start(0);
#endif
        mpm->simStep();
#ifdef MEASURE_TIME
        ctimer.end(0);
        auto end = std::chrono::high_resolution_clock::now();
        frame_sim_time_ms += std::chrono::duration<double, std::milli>(end - start).count();
        // ctimer.start(3);
        // ctimer.end(3);
#endif
    }
}

void clearWindow() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
}

/**
 * Reconstructs surface with according renderer
 */
void renderSpheres(bool firstRender = false) {
    if(!firstRender && !mpm->isTimeToRender() && state.play) return;

#ifdef MEASURE_TIME
    timer.update();

    if (state.play && !firstRender) {
        float gpu_prep_time = ctimer.rbh[1].last() + ctimer.rbh[3].last();
        float render_time = timer.rbh[0].last();
        std::cout << "Frame " << mpm->getFrame() << " simulated in : " << frame_sim_time_ms << " ms"
                  << ", Data prep: " << gpu_prep_time << " ms"
                  << ", Render: " << render_time << " ms" << std::endl;
        frame_sim_time_ms = 0.0;
    }
#else
    if (state.play && !firstRender){
        std::cout << "Frame " << mpm->getFrame() << " done." << std::endl;
    }
#endif

    clearWindow();

#ifdef MEASURE_TIME
    ctimer.start(3);
#endif
    mpm->recountParticles();
#ifdef MEASURE_TIME
    ctimer.end(3);
#endif

    int ww, wh;
    glfwGetFramebufferSize(window, &ww, &wh);
    auto& spheresData = mpm->getParticles();

#ifdef MEASURE_TIME
    timer.start(0);
#endif
    rm->march(ww, wh, mpm, camera);
#ifdef MEASURE_TIME
    timer.end(0);
#endif

    // Take screenshot only if enabled, simulation is running, and we are past the stabilization frames.
    if (state.takeScreenshot && state.play && !firstRender && mpm->getFrame() >= state.stabilizeFrames) {
        saveImage(state.outputDir, window);
    }
}

/**
 * Initializes MPM simulation
 */
void physicsInit() {
    mpm = new MPMIntegrationSim();
    mpm->setupScene(state.FPS, state.g_spatula_anim_path);
}

/**
 * Main menu gui
 * @param start if user selected to start the scene
 */
void guiStart(bool &start) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);

    ImGui::Begin("##MM", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

    float bw = vp->WorkSize.x * 0.5f;
    float bh = 80.0f;

    float totalH = (bh * 2) + 160.0f + (state.g_num_colors * 40.0f) + 80.0f; // Increased for preview

    ImGui::SetCursorPos(ImVec2((vp->WorkSize.x - bw) * 0.5f, (vp->WorkSize.y - totalH) * 0.5f));

    ImGui::BeginGroup();

    static int prev_num_colors = state.g_num_colors;
    ImGui::PushItemWidth(bw);
    
    // When the number of colors changes, each gets a basic minimum of 10%
    if (ImGui::SliderInt("Number of colors", &state.g_num_colors, 2, 4)) {
        if (state.g_num_colors != prev_num_colors) {
            for (int i = 0; i < state.g_num_colors; ++i) state.g_ratios[i] = 0.1f;
            for (int i = state.g_num_colors; i < 4; ++i) state.g_ratios[i] = 0.0f;
            prev_num_colors = state.g_num_colors;
        }
    }
    
    // Calculate unallocated free volume
    float current_sum = 0.0f;
    for (int i = 0; i < state.g_num_colors; ++i) current_sum += state.g_ratios[i];
    float unallocated = std::max(0.0f, 1.0f - current_sum);

    ImVec4 unallocatedColor = (unallocated > 0.005f) ? ImVec4(0.9f, 0.7f, 0.0f, 1.0f) : ImVec4(0.2f, 0.8f, 0.2f, 1.0f);
    ImGui::TextColored(unallocatedColor, "Remaining volume: %.2f", unallocated);
    ImGui::Dummy(ImVec2(0, 5));

    // Fixed maximum value for the visual range of the sliders (prevents them from jumping)
    float fixed_max = 1.0f - 0.1f * (state.g_num_colors - 1);

    for (int i = 0; i < state.g_num_colors; ++i) {
        ImGui::PushID(i);
        
        // Preview
        ImGui::ColorButton("##ColorPreview", ImVec4(state.g_colors[i][0], state.g_colors[i][1], state.g_colors[i][2], 1.0f));
        ImGui::SameLine();

        // Dropdown
        ImGui::PushItemWidth(150);
        if (ImGui::Combo("##Color", &state.g_selected_pigment_indices[i], state.g_pigment_names.data(), state.g_pigment_names.size())) {
            // When a new pigment is selected, just update the color for the current slot.
            // The logic to prevent duplicates has been removed.
            const auto& selected_pigment = state.g_available_pigments[state.g_selected_pigment_indices[i]];
            state.g_colors[i][0] = selected_pigment.rgb[0];
            state.g_colors[i][1] = selected_pigment.rgb[1];
            state.g_colors[i][2] = selected_pigment.rgb[2];
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();

        if (ImGui::SliderFloat("Ratio", &state.g_ratios[i], 0.1f, fixed_max, "%.2f")) {
            state.g_ratios[i] = roundf(state.g_ratios[i] * 100.0f) / 100.0f;
            
            // Calculate the actual physical limit for the given slider based on the other colors
            float sum_others = 0.0f;
            for (int j = 0; j < state.g_num_colors; ++j) {
                if (i != j) sum_others += state.g_ratios[j];
            }
            float true_max_allowed = std::max(0.1f, 1.0f - sum_others);
            
            // Do not allow the user to drag the value higher than physically possible
            state.g_ratios[i] = std::max(0.1f, std::min(state.g_ratios[i], true_max_allowed));
        }
        ImGui::PopID();
    }
    ImGui::PopItemWidth();
    ImGui::Dummy(ImVec2(0, 20));

    // Preview of the mixed color
    ImGui::Text("Expected mixed result:");
    float mixed_latent[MIXBOX_NUMLATENTS] = {0.0f};
    float total_ratio = 0.0f;
    for (int i = 0; i < state.g_num_colors; ++i) {
        total_ratio += state.g_ratios[i];
    }
    float preview_r = 0.0f, preview_g = 0.0f, preview_b = 0.0f;
    if (total_ratio > 0.001f) {
        for (int i = 0; i < state.g_num_colors; ++i) {
            float latent[MIXBOX_NUMLATENTS];
            mixbox_srgb32f_to_latent(state.g_colors[i][0], state.g_colors[i][1], state.g_colors[i][2], latent);
            float weight = state.g_ratios[i] / total_ratio;
            for (int j = 0; j < MIXBOX_NUMLATENTS; ++j) {
                mixed_latent[j] += latent[j] * weight;
            }
        }
        mixbox_latent_to_srgb32f(mixed_latent, &preview_r, &preview_g, &preview_b);
    }
    ImGui::ColorButton("##MixedPreview", ImVec4(preview_r, preview_g, preview_b, 1.0f), ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoInputs, ImVec2(bw, 40.0f));
    ImGui::Dummy(ImVec2(0, 20));

    bool can_start = (unallocated < 0.005f); // Allows for minor float deviation
    if (!can_start) {
        ImGui::TextColored(ImVec4(0.8f, 0.2f, 0.2f, 1.0f), "Remaining volume must be split before starting.");
        ImGui::BeginDisabled(true);
    }

    if (ImGui::Button("Start simulation", ImVec2(bw, bh))) {
        start = true;
    }

    if (!can_start) {
        ImGui::EndDisabled();
    }
    ImGui::Dummy(ImVec2(0, 60));

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
    if (ImGui::Button("EXIT", ImVec2(bw, bh))) { glfwSetWindowShouldClose(window, true); }
    ImGui::PopStyleColor();
    ImGui::EndGroup();

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

/**
 * GUI of the simulation
 */
void gui() {
    if (state.changeFontSize) {
        ImGuiIO& io = ImGui::GetIO();
        state.changeFontSize = false;
        io.Fonts->Clear();
        ImFontConfig ifc;
        ifc.SizePixels = state.fonts[state.fontChoice];
        io.Fonts->AddFontDefault(&ifc);
        ImGui_ImplOpenGL3_DestroyFontsTexture();
        ImGui_ImplOpenGL3_CreateFontsTexture();
    }
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Simulation");
    ImGuiIO& io = ImGui::GetIO();

    ImGui::BeginChild("Settings", ImVec2(28*state.fonts[state.fontChoice], 0), true);

    
    if (ImGui::CollapsingHeader("Iso Value", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Auto ISO:");
        if (ImGui::Checkbox("##AutoISO", &state.autoISO)) {
            if (state.autoISO) {
                float h = mpm->getSupportRadius();
                state.vdall = 0.8f * h;
                float q = state.vdall/h;
                state.iso = state.kern/(h*h*h)*pow(1.0f - q*q, 3);
            }
        }
        ImGui::Text("vdall:");
        if (ImGui::SliderFloat("##vdall", &state.vdall, 0.001f, 0.2f, "%.4f")) {
            if (state.autoISO) {
                float h = mpm->getSupportRadius();
                float q = state.vdall/h;
                state.iso = state.kern/(h*h*h)*pow(1.0f - q*q, 3);
            }
        }
        ImGui::BeginDisabled(state.autoISO);
        ImGui::Text("ISO:");
        ImGui::SliderFloat("##ISO", &state.iso, 0.01f, 5000);
        ImGui::EndDisabled();
    }
    ImGui::Spacing();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Ray Marching parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Max amount of steps for a ray:");
        if (ImGui::SliderInt("##Maxstepcount", &state.maxStepCount, 10, 1000)) {
            if (state.maxSkipCount > state.maxStepCount - state.stepsInside) {
                state.maxSkipCount = state.maxStepCount - state.stepsInside;
            }
        }

        ImGui::Text("Max amount of steps before skip to aggregation:");
        ImGui::SliderInt("##maxSkip", &state.maxSkipCount, 6, state.maxStepCount - state.stepsInside);

        ImGui::Text("Amount of steps inside possible cell:");
        ImGui::SliderInt("##Stepsinside ", &state.stepsInside, 2, 50);
    }
    ImGui::Spacing();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Binary Density Grid construction", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Factor a for BDG construction:");
        if (ImGui::SliderFloat("##afactor", &state.params.x, 0.02f, 1.0f, "%.2f")) {
            if (state.params.y > state.params.x - 0.01f) {
                state.params.y = state.params.x - 0.01f;
            }
        }

        ImGui::Text("Factor b for BDG construction:");
        ImGui::SliderFloat("##bfactor", &state.params.y, 0.01f, state.params.x-0.01f, "%.2f");
    }
    ImGui::Spacing();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Normals blending", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("A factor for normals blending:");
        ImGui::SliderFloat("##Afactor", &state.A, 0.00f, 10.0f);

        ImGui::Text("B factor for normals blending:");
        ImGui::SliderFloat("##Bfactor", &state.B, 0.00f, 10.0f);

    }
    ImGui::Spacing();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Depth Map Smoothing", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Filter Radius (R):");
        ImGui::SliderFloat("##FilterR", &state.R, 1.0f, 32.0f);

        ImGui::Text("Spatial weight (ks):");
        ImGui::SliderFloat("##FilterKs", &state.ks, 0.1f, 10.0f);

        ImGui::Text("Range weight (kr):");
        ImGui::SliderFloat("##FilterKr", &state.kr, 0.01f, 2.0f);
    }
    ImGui::Spacing();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Anisotropy for surface construction", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Anisotropy for rendering:");
        ImGui::Checkbox("##Anisotropy", &state.isAni);

        ImGui::Text("Auto scale factor for anisotropy:");
        ImGui::Checkbox("##AutoSfactor", &state.autoScaleS);

        ImGui::BeginDisabled(state.autoScaleS);
        ImGui::Text("Scale factor for anisotropy:");
        ImGui::SliderFloat("##Sfactor", &state.s, 1.0f, 2400.0f);
        ImGui::EndDisabled();

        ImGui::BeginDisabled(!state.isAni);
        ImGui::Text("Anisotropy threshold:");
        ImGui::SliderInt("##Anisotropythreshold", &state.aniso_threshold, 2, 25);
        ImGui::Text("Anisotropy k (clamping):");
        ImGui::SliderFloat("##AnisotropyK", &state.k, 0.1f, 10.0f);
        ImGui::EndDisabled();
    }
    ImGui::Spacing();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Max aggregated blocks one axis:");
        ImGui::Combo("##maxAggr", &state.currRes, " 1\0 2\0 4\0\0");

        ImGui::Text("Visible spheres:");
        ImGui::Checkbox("##seeSpheres", &state.seeSpheres);
    }
    ImGui::Spacing();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Show Diffusion (Red/Blue)", &state.showDiffusion);
        ImGui::SliderFloat("Sigma Color", &state.sigma_color, 0.01f, 1.0f);
        ImGui::SliderFloat("Sigma Spatial", &state.sigma_spatial, 0.01f, 10.0f);
        ImGui::Checkbox("Show Normals blending (Red/Blue)", &state.showNormals);
    }
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Font size:");
    if (ImGui::Combo("##Fontsize", &state.fontChoice, " 10\0 12\0 14\0 16\0 18\0 20\0 22\0 24\0\0")) {
        state.changeFontSize = true;
    }
    ImGui::EndChild();

    ImGui::SameLine();

    ImGui::BeginChild("---Statistics---", ImVec2(0,0), true);
#ifdef MEASURE_TIME
    timer.plotLine();
    ctimer.plotLine();
    ImGui::Separator();
#endif


    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Text("Particles: %3d", mpm->getParticleAmount());
    ImGui::Text("Particles radius: %.3f", mpm->getRadius());
    ImGui::Text("Cells size: %3d", static_cast<int>(rm->getA()->getSize()));
    ImGui::Text("There are %3d possible cells", state.count[1]);

    ImGui::Text("FPS: %.1f (%.3f ms/frame)", io.Framerate, 1000.0f/io.Framerate);

    ImGui::Separator();
    ImGui::InputText("Output Dir", state.outputDir, IM_ARRAYSIZE(state.outputDir));
    if (ImGui::Button("Save Rendered Image")) {
        state.takeScreenshot = true;
    }

    ImGui::EndChild();

    ImGui::End();

    ImGui::Render();
#ifdef MEASURE_TIME
    timer.start(1);
#endif
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#ifdef MEASURE_TIME
    timer.end(1);
#endif
}

int mainComputeLoop() {
    int fbW, fbH;
    glfwGetFramebufferSize(glfwGetCurrentContext(), &fbW, &fbH);
    glViewport(0, 0, fbW, fbH);
#ifdef MEASURE_TIME
    timer.init();
    ctimer.init();
#endif
    bool startScene = false;
    static bool recording_started_message = false;
    while(!glfwWindowShouldClose(window)) {
        if (!startScene) {
            clearWindow();

            guiStart(startScene);
            if (startScene) {
                std::cout << "--- Starting Simulation ---" << std::endl;
                for (int i = 0; i < state.g_num_colors; ++i) {
                    const char* color_name = state.g_pigment_names[state.g_selected_pigment_indices[i]];
                    std::cout << "  - {}: {:.2f}" << color_name << ": " << state.g_ratios[i] << std::endl;
                }
                std::cout << "---------------------------" << std::endl;
                state.simulationStartTime = std::chrono::high_resolution_clock::now();

                state.changeFontSize = true;
                physicsInit();
                a = new AABBc(mpm);
                rm = new RayMarch(mpm, a);
                rm->loadConfig("../render_config.json");

                renderSpheres(true);
            }
        }
        else {
            if (mpm->isFinished()) {
                glfwSetWindowShouldClose(window, true);
                continue;
            }

            if (!recording_started_message && mpm->getFrame() >= state.stabilizeFrames) {
                std::cout << "--- Stabilization finished. Starting to record frames. ---" << std::endl;
                state.takeScreenshot = true;
                recording_started_message = true;
            }

            sim();
            renderSpheres();
            gui();
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
#ifdef MEASURE_TIME
    timer.clear();
#endif

    // Calculate and print the final mixed color
    if (mpm) {
        mpm->calculateAndPrintFinalColor();
    }

    delete mpm;
    delete camera;
    delete rm;
    delete a;
    glfwTerminate();

    if (state.takeScreenshot || state.screenshotsTaken) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        std::string timestamp = ss.str();
        // Command to create an MP4 video with the H.264 codec, which is widely compatible.
        // -pix_fmt yuv420p is important for compatibility with most players.
        std::string cmd = "ffmpeg -y -loglevel quiet -framerate " + std::to_string(state.FPS) + " -i " + std::string(state.outputDir) + "/render_%05d.png -vf \"pad=ceil(iw/2)*2:ceil(ih/2)*2\" -c:v libx264 -pix_fmt yuv420p " + std::string(state.outputDir) + "/animation_" + timestamp + ".mp4";
        int ret = system(cmd.c_str());
        if (ret == 0) {
            int frame_count = 0;
            for (const auto& entry : std::filesystem::directory_iterator(state.outputDir)) {
                if (entry.path().extension() == ".png") {
                    frame_count++;
                    std::filesystem::remove(entry.path());
                }
            }
            std::cout << "Video created from " << frame_count << " frames and screenshots deleted." << std::endl;
        } else {
            std::cerr << "Failed to create video. Make sure ffmpeg is installed on your system and libx264 is available." << std::endl;
        }
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> total_duration = endTime - state.simulationStartTime;
        if (total_duration.count() > 0) {
            std::cout << "Total simulation and video generation time: " << std::fixed << std::setprecision(2) << total_duration.count() / 60.0 << " minutes." << std::endl;
        }
    }

    return 0;
}

int main() {
    omp_set_num_threads(6);
    auto cons_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    cons_sink->set_level(spdlog::level::err);
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("../log.log", true);
    file_sink->set_level(spdlog::level::trace);
    auto l = std::make_shared<spdlog::logger>("logger", spdlog::sinks_init_list{cons_sink, file_sink});
    spdlog::register_logger(l);
    spdlog::set_default_logger(l);
    l->set_level(spdlog::level::trace);
    spdlog::flush_every(std::chrono::seconds(3));

    loadPigmentConfig("../colors_config.json");

    if (createWindow() != OK) return -1;

    glm::vec3 camera_pos(0.0f, 1.5f, -2.0f);
    glm::vec3 camera_tgt(0.0f, 0.0f, 0.0f);

    std::string config_path = "../render_config.json";
    std::ifstream f(config_path);
    if (!f.is_open()) {
        config_path = "render_config.json"; // Fallback pokud program běží mimo build složku
        f.open(config_path);
    }

    if (f.is_open()) {
        try {
            nlohmann::json data;
            f >> data;
            if (data.contains("camera")) {
                auto& camera_data = data.at("camera");
                if (camera_data.contains("position")) {
                    auto& pos = camera_data.at("position");
                    camera_pos = glm::vec3(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>());
                }
                if (camera_data.contains("target")) {
                    auto& tgt = camera_data.at("target");
                    camera_tgt = glm::vec3(tgt[0].get<float>(), tgt[1].get<float>(), tgt[2].get<float>());
                }
            }
            if (data.contains("simulation")) {
                auto& sim = data.at("simulation");
                if (sim.contains("fps")) {
                    state.FPS = sim.at("fps").get<int>();
                }
                if (sim.contains("animation")) {
                    state.g_spatula_anim_path = sim.at("animation").get<std::string>();
                }
            }
            std::cout << "Camera position loaded from config: (" << camera_pos.x << ", " << camera_pos.y << ", " << camera_pos.z << ")" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Error parsing {}: {}. Using default camera settings." << config_path << ": " << e.what() << std::endl;
        }
    } else {
        std::cout << "Config file not found: " << config_path << std::endl;
        std::cout << "Using default camera settings." << std::endl;
    }

    camera = new Camera(camera_pos, camera_tgt);
    mainComputeLoop();
    spdlog::shutdown();
    return OK;
}
