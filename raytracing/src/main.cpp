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
#include <string>
#include <algorithm>
#include <vector>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <chrono>
#include <iomanip>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

GLFWwindow* window;

RayMarch *rm;
Camera *camera;
MPMIntegrationSim *mpm;
AABBc *a;
char outputDir[256] = "output_images";
bool takeScreenshot = false;
bool screenshotsTaken = false;

extern std::string g_spatula_anim_path;
int g_num_colors = 2;
// float g_colors[4][3] = {
//     {0.959123f, 0.802565f, 0.0356184f}, // Yellow
//     {0.0771705f, 0.0282698f, 0.24833f}, // Blue
//     {0.995181f, 0.999781f, 0.997048f}, // White
//     {0.506f, 0.012f, 0.184f}  // Magenta
// };
float g_colors[4][3] = {
    {0.982f, 0.655f, 0.001f}, // Yellow
    {0.003f, 0.015f, 0.076f}, // Blue
    {0.956f, 0.956f, 0.947f}, // White
    {0.218f, 0.001f, 0.027f}  // Magenta
};
float g_ratios[4] = { 0.5f, 0.5f, 0.0f, 0.0f };

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
            if (frame_counter % 60 == 1) debug("Saved image to ", filepath);
            screenshotsTaken = true;
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
        takeScreenshot = !takeScreenshot;
        debug("Take screenshot: ", takeScreenshot);
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
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "GLHydroSurface", nullptr, nullptr);
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

static double frame_sim_time_ms = 0.0;

/**
 * Performs one simulation step if simulation is unpaused
 */
void sim() {
    if (state.play) {
        auto start = std::chrono::high_resolution_clock::now();
        ctimer.start(0);
        mpm->simStep();
        ctimer.end(0);
        auto end = std::chrono::high_resolution_clock::now();
        frame_sim_time_ms += std::chrono::duration<double, std::milli>(end - start).count();
        // ctimer.start(3);
        // ctimer.end(3);
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

    if (state.play && !firstRender) {
        std::cout << "\nSnímek nasimulován za: " << frame_sim_time_ms << " ms" << std::endl;
        frame_sim_time_ms = 0.0;
    }

    clearWindow();
    mpm->recountParticles();

    int ww, wh;
    glfwGetFramebufferSize(window, &ww, &wh);
    auto& spheresData = mpm->getParticles();

    rm->march(ww, wh, mpm, camera);

    if (takeScreenshot && state.play) {
        saveImage(outputDir, window);
    }
}

/**
 * Initializes MPM simulation
 */
void physicsInit() {
    mpm = new MPMIntegrationSim();
    mpm->setupScene();
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

    float totalH = (bh * 2) + 160.0f + (g_num_colors * 40.0f);

    ImGui::SetCursorPos(ImVec2((vp->WorkSize.x - bw) * 0.5f, (vp->WorkSize.y - totalH) * 0.5f));

    ImGui::BeginGroup();

    static int spatula_anim_idx = 0;
    ImGui::PushItemWidth(bw);
    ImGui::Text("Spatula Animation:");
    ImGui::Combo("##SpatulaAnim", &spatula_anim_idx, "Mixing Blobs\0Squish\0Sweep\0Mixing\0Inf\0\0");
    ImGui::PopItemWidth();
    ImGui::Dummy(ImVec2(0, 10));

    ImGui::Separator();
    ImGui::Dummy(ImVec2(0, 10));

    static int prev_num_colors = g_num_colors;
    ImGui::PushItemWidth(bw);
    
    // Re-balance evenly if the user changes the number of active colors
    if (ImGui::SliderInt("Number of Colors", &g_num_colors, 2, 4)) {
        if (g_num_colors != prev_num_colors) {
            float val = 1.0f / g_num_colors;
            for (int i = 0; i < g_num_colors; ++i) g_ratios[i] = val;
            prev_num_colors = g_num_colors;
        }
    }
    
    for (int i = 0; i < g_num_colors; ++i) {
        ImGui::PushID(i);
        ImGui::ColorEdit3("##Color", g_colors[i], ImGuiColorEditFlags_NoInputs);
        ImGui::SameLine();
        
        float max_ratio = 1.0f - 0.1f * (g_num_colors - 1);
        if (ImGui::SliderFloat("Ratio", &g_ratios[i], 0.1f, max_ratio, "Vol Ratio: %.05f")) {
            g_ratios[i] = std::max(0.1f, std::min(g_ratios[i], max_ratio)); // Ensure strict bounds on manual input

            float rest_sum = 0.0f;
            for (int j = 0; j < g_num_colors; ++j) if (i != j) rest_sum += g_ratios[j];
            
            float target_rest_sum = 1.0f - g_ratios[i];
            float free_rest_sum = rest_sum - 0.1f * (g_num_colors - 1);
            float target_free_sum = target_rest_sum - 0.1f * (g_num_colors - 1);

            if (free_rest_sum > 0.0001f) { // Scale the free portions proportionally
                float scale = target_free_sum / free_rest_sum;
                for (int j = 0; j < g_num_colors; ++j) if (i != j) g_ratios[j] = 0.1f + (g_ratios[j] - 0.1f) * scale;
            } else { // If the others were fully depleted, distribute the new free remainder evenly
                float val = target_free_sum / (g_num_colors - 1);
                for (int j = 0; j < g_num_colors; ++j) if (i != j) g_ratios[j] = 0.1f + val;
            }
        }
        ImGui::PopID();
    }
    ImGui::PopItemWidth();
    ImGui::Dummy(ImVec2(0, 20));

    if (ImGui::Button("Start simulation", ImVec2(bw, bh))) {
        if (spatula_anim_idx == 0) g_spatula_anim_path = "../matter/animations/spatula_motion_blobs.bin";
        else if (spatula_anim_idx == 1) g_spatula_anim_path = "../matter/animations/spatula_motion_squish.bin";
        else if (spatula_anim_idx == 2) g_spatula_anim_path = "../matter/animations/spatula_motion_sweep.bin";
        else if (spatula_anim_idx == 3) g_spatula_anim_path = "../matter/animations/spatula_motion_mixing.bin";
        else if (spatula_anim_idx == 4) g_spatula_anim_path = "../matter/animations/spatula_motion_inf.bin";

        start = true;
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
        ImGui::SliderFloat("Sigma Color", &state.sigma_color, 0.01f, 2.0f);
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
    timer.plotLine();
    ctimer.plotLine();
    ImGui::Separator();


    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Text("Particles: %3d", mpm->getParticleAmount());
    ImGui::Text("Particles radius: %.3f", mpm->getRadius());
    ImGui::Text("Cells size: %3d", static_cast<int>(rm->getA()->getSize()));
    ImGui::Text("There are %3d possible cells", state.count[1]);

    ImGui::Text("FPS: %.1f (%.3f ms/frame)", io.Framerate, 1000.0f/io.Framerate);

    ImGui::Separator();
    ImGui::InputText("Output Dir", outputDir, IM_ARRAYSIZE(outputDir));
    if (ImGui::Button("Save Rendered Image")) {
        takeScreenshot = true;
    }

    ImGui::EndChild();

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

int mainComputeLoop() {
    int fbW, fbH;
    glfwGetFramebufferSize(glfwGetCurrentContext(), &fbW, &fbH);
    glViewport(0, 0, fbW, fbH);
    timer.init();
    ctimer.init();
    bool startScene = false;
    while(!glfwWindowShouldClose(window)) {
        if (startScene)
            timer.update();
        if (!startScene) {
            clearWindow();

            guiStart(startScene);
            if (startScene) {
                state.changeFontSize = true;
                physicsInit();
                a = new AABBc(mpm);
                rm = new RayMarch(mpm, a);
                rm->loadConfig("../render_config.json");

                renderSpheres(true);
            }
        }
        else {
            sim();
            timer.start();
            renderSpheres();
            gui();
            timer.end();
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    timer.clear();
    delete mpm;
    delete camera;
    delete rm;
    delete a;
    glfwTerminate();

    if (takeScreenshot || screenshotsTaken) {
        std::cout << "Creating video from screenshots..." << std::endl;
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        std::string timestamp = ss.str();
        // Příkaz pro vytvoření MP4 videa s kodekem H.264, který je široce kompatibilní.
        // -pix_fmt yuv420p je důležitý pro kompatibilitu s většinou přehrávačů.
        std::string cmd = "ffmpeg -y -framerate 30 -i " + std::string(outputDir) + "/render_%05d.png -vf \"pad=ceil(iw/2)*2:ceil(ih/2)*2\" -c:v libx264 -pix_fmt yuv420p " + std::string(outputDir) + "/animation_" + timestamp + ".mp4";
        int ret = system(cmd.c_str());
        if (ret == 0) {
            std::cout << "Video created successfully. Deleting PNG sequence..." << std::endl;
            for (const auto& entry : std::filesystem::directory_iterator(outputDir)) {
                if (entry.path().extension() == ".png") {
                    std::filesystem::remove(entry.path());
                }
            }
            std::cout << "Screenshots deleted." << std::endl;
        } else {
            std::cerr << "Failed to create video. Make sure ffmpeg is installed on your system and libx264 is available." << std::endl;
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
            spdlog::info("Loaded camera settings from {}: pos({}, {}, {})", config_path, camera_pos.x, camera_pos.y, camera_pos.z);
        } catch (const std::exception& e) {
            spdlog::warn("Error parsing {}: {}. Using default camera settings.", config_path, e.what());
        }
    } else {
        spdlog::info("render_config.json not found. Using default camera settings.");
    }

    camera = new Camera(camera_pos, camera_tgt);
    mainComputeLoop();
    spdlog::shutdown();
    return OK;
}
