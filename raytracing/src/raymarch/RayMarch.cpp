#include "RayMarch.h"

#include "PassTimer.h"
#include "HDRLoader.h"
#include "PassTimer.h"
#include <fstream>
#include <vector>
#include "../../../deps/json.hpp"

RayMarch::RayMarch(MPMIntegrationSim *mpm, AABBc *a) {
    glGenBuffers(1, &spheresSSBO);
    glGenBuffers(1, &pigmentsSSBO);
    glGenBuffers(1, &diffusionSSBO);
    depthMaps = new DepthProcessor();
    this->a = a;
    bdg = new BinaryDensityGrid(a, mpm);
    initShader();
    texQuadInit();
    createOutputTexture();
    generateBRDFLUT();
    initSkybox();

    // Default lights if not in config
    lightDirs.push_back(glm::normalize(glm::vec3(0.9096f, 0.4466f, 0.3660f)));
    lightColors.push_back(glm::vec3(1.0f));
    lightIntensities.push_back(2.0f);

    lightDirs.push_back(glm::normalize(glm::vec3(-0.2853f, 0.4466f, 0.9380f)));
    lightColors.push_back(glm::vec3(1.0f));
    lightIntensities.push_back(0.5f);


    spatulaMesh = new SpatulaMesh("data/spatula_handler.obj");
}

RayMarch::~RayMarch() {
    delete depthMaps;
    delete bdg;
    delete shader;
    delete texShader;
    delete spatulaShader;
    delete skyboxShader;
    delete bilateralShader;
    glDeleteBuffers(1, &spheresSSBO);
    glDeleteBuffers(1, &pigmentsSSBO);
    glDeleteBuffers(1, &diffusionSSBO);
    glDeleteTextures(1, &outputTex);
    glDeleteTextures(1, &postProcessTex);
    if (postProcessPingPongTex) {
        glDeleteTextures(1, &postProcessPingPongTex);
    }
    glDeleteTextures(1, &normalDepthTex);
    if (hdrTexture) {
        glDeleteTextures(1, &hdrTexture);
    }
    if (irradianceTexture) {
        glDeleteTextures(1, &irradianceTexture);
    }
    if (brdfLUTTexture) {
        glDeleteTextures(1, &brdfLUTTexture);
    }
    glDeleteBuffers(1, &quadVBO);
    glDeleteVertexArrays(1, &quadVAO);
    glDeleteVertexArrays(1, &skyboxVAO);
    glDeleteBuffers(1, &skyboxVBO);
    delete spatulaMesh;
}

void RayMarch::loadConfig(const std::string& path) {
    std::string config_path = path;
    std::ifstream file(config_path);
    if (!file.is_open() && config_path == "../render_config.json") {
        config_path = "render_config.json"; // Fallback
        file.open(config_path);
    }
    if (file.is_open()) {
        try {
            nlohmann::json j;
            file >> j;
            if (j.contains("materials")) {
                auto& mats = j["materials"];
                if (mats.contains("fluid")) {
                    fluidMat.albedo = glm::vec3(mats["fluid"]["albedo"][0].get<float>(), mats["fluid"]["albedo"][1].get<float>(), mats["fluid"]["albedo"][2].get<float>());
                    fluidMat.metallic = mats["fluid"]["metallic"].get<float>();
                    fluidMat.roughness = mats["fluid"]["roughness"].get<float>();
                }
                if (mats.contains("spatula_metal")) {
                    spatulaMetal.albedo = glm::vec3(mats["spatula_metal"]["albedo"][0].get<float>(), mats["spatula_metal"]["albedo"][1].get<float>(), mats["spatula_metal"]["albedo"][2].get<float>());
                    spatulaMetal.metallic = mats["spatula_metal"]["metallic"].get<float>();
                    spatulaMetal.roughness = mats["spatula_metal"]["roughness"].get<float>();
                }
                if (mats.contains("spatula_wood")) {
                    spatulaWood.albedo = glm::vec3(mats["spatula_wood"]["albedo"][0].get<float>(), mats["spatula_wood"]["albedo"][1].get<float>(), mats["spatula_wood"]["albedo"][2].get<float>());
                    spatulaWood.metallic = mats["spatula_wood"]["metallic"].get<float>();
                    spatulaWood.roughness = mats["spatula_wood"]["roughness"].get<float>();
                }
                if (mats.contains("floor")) {
                    floorMat.albedo = glm::vec3(mats["floor"]["albedo"][0].get<float>(), mats["floor"]["albedo"][1].get<float>(), mats["floor"]["albedo"][2].get<float>());
                    floorMat.metallic = mats["floor"]["metallic"].get<float>();
                    floorMat.roughness = mats["floor"]["roughness"].get<float>();
                }
            }
            if (j.contains("lights")) {
                lightDirs.clear();
                lightColors.clear();
                lightIntensities.clear();
                auto& lights_data = j["lights"];
                for (const auto& light : lights_data) {
                    if (lightDirs.size() >= MAX_LIGHTS) {
                        spdlog::warn("Maximum number of lights ({}) reached. Ignoring additional lights.", (int)MAX_LIGHTS);
                        break;
                    }
                    if (light.contains("direction") && light.contains("color") && light.contains("intensity")) {
                        auto dir_vec = light["direction"].get<std::vector<float>>();
                        lightDirs.push_back(glm::normalize(glm::vec3(dir_vec[0], dir_vec[1], dir_vec[2])));

                        auto col_vec = light["color"].get<std::vector<float>>();
                        lightColors.push_back(glm::vec3(col_vec[0], col_vec[1], col_vec[2]));

                        lightIntensities.push_back(light["intensity"].get<float>());
                    }
                }
            }
            
            std::string skyMapPath = "data/SkyMap.hdr";
            if (j.contains("environment")) {
                auto& env = j["environment"];
                if (env.contains("skymap")) {
                    skyMapPath = env["skymap"].get<std::string>();
                }
            }
            if (hdrTexture) glDeleteTextures(1, &hdrTexture);
            if (irradianceTexture) glDeleteTextures(1, &irradianceTexture);
            HDRLoader::loadHDRCubemap(skyMapPath, hdrTexture, irradianceTexture);
            
        } catch (const std::exception& e) {
            spdlog::error("Error parsing config {}: {}", config_path, e.what());
        }
    } else {
        spdlog::warn("Could not open config file: {}", config_path);
    }
}

void RayMarch::initShader() {
    shader = new Shader("../shaders/raymarching/raymarch.glsl");
    texShader = new Shader("../shaders/raymarching/shader.vert", "../shaders/raymarching/shader.frag");
    spatulaShader = new Shader("../shaders/raymarching/spatula.vert", "../shaders/raymarching/spatula.frag");
    bilateralShader = new Shader("../shaders/raymarching/bilateral.glsl");
}

void RayMarch::texQuadInit() {
    float quadVertices[] = {
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,

        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };

    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);

    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_COPY);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

    glBindVertexArray(0);
}

void RayMarch::initSkybox() {
    float skyboxVertices[] = {
        // positions          
        -1.0f,  1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,  1.0f,  1.0f, -1.0f, -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f, -1.0f,  1.0f,  1.0f, -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,  1.0f, -1.0f,  1.0f,  1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,  1.0f,  1.0f, -1.0f,  1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f, -1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,  1.0f, -1.0f,  1.0f, -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,  1.0f,  1.0f, -1.0f,  1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f, -1.0f,  1.0f,  1.0f, -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f,  1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f,  1.0f, -1.0f,  1.0f
    };
    
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);

    skyboxShader = new Shader("../shaders/raymarching/skybox.vert", "../shaders/raymarching/skybox.frag");
}

void RayMarch::renderSkybox(Camera* camera) {
    if (!hdrTexture) return;
    
    // Render the skybox behind all other elements
    glDepthFunc(GL_LEQUAL); 
    skyboxShader->use();

    // Remove translation from the view matrix so the camera only rotates and doesn't move out of the box
    glm::mat4 view = glm::mat4(glm::mat3(camera->getView())); 
    skyboxShader->setUniform("view", view);
    skyboxShader->setUniform("projection", camera->getProj());
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, hdrTexture);
    skyboxShader->setUniform("skybox", 0);
    
    glBindVertexArray(skyboxVAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
    
    glDepthFunc(GL_LESS);
}

void RayMarch::generateBRDFLUT() {
    glGenTextures(1, &brdfLUTTexture);
    glBindTexture(GL_TEXTURE_2D, brdfLUTTexture);
    // We use RG format because we only store parameters A and B (scale and bias for Fresnel)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, 512, 512, 0, GL_RG, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    GLuint captureFBO, captureRBO;
    glGenFramebuffers(1, &captureFBO);
    glGenRenderbuffers(1, &captureRBO);

    glBindFramebuffer(GL_FRAMEBUFFER, captureFBO);
    glBindRenderbuffer(GL_RENDERBUFFER, captureRBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 512, 512);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, brdfLUTTexture, 0);

    GLint last_viewport[4];
    glGetIntegerv(GL_VIEWPORT, last_viewport);
    glViewport(0, 0, 512, 512);
    
    // Use the same vertex shader for the full-screen quad as for regular texture rendering
    Shader brdfShader("../shaders/raymarching/shader.vert", "../shaders/raymarching/brdf.frag");
    brdfShader.use();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(last_viewport[0], last_viewport[1], last_viewport[2], last_viewport[3]);
    
    glDeleteFramebuffers(1, &captureFBO);
    glDeleteRenderbuffers(1, &captureRBO);
}

void RayMarch::resizeTexutres(GLint ww, GLint wh) {
    glBindTexture(GL_TEXTURE_2D, outputTex);
    GLint x, y;
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH,  &x);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &y);
    if (ww != x || wh != y) {
        spdlog::debug("Changing size of textures for output texture of ray tracing to {} {}", ww, wh);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        ww, wh, 0, GL_RGBA, GL_FLOAT, nullptr);

        glBindTexture(GL_TEXTURE_2D, normalDepthTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        ww, wh, 0, GL_RGBA, GL_FLOAT, nullptr);

        glBindTexture(GL_TEXTURE_2D, postProcessTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        ww, wh, 0, GL_RGBA, GL_FLOAT, nullptr);
        
        glBindTexture(GL_TEXTURE_2D, postProcessPingPongTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        ww, wh, 0, GL_RGBA, GL_FLOAT, nullptr);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
}


void RayMarch::createOutputTexture() {
    glGenTextures(1, &outputTex);
    glBindTexture(GL_TEXTURE_2D, outputTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        10, 10, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &postProcessTex);
    glBindTexture(GL_TEXTURE_2D, postProcessTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        10, 10, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &postProcessPingPongTex);
    glBindTexture(GL_TEXTURE_2D, postProcessPingPongTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        10, 10, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &normalDepthTex);
    glBindTexture(GL_TEXTURE_2D, normalDepthTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        10, 10, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);
}

AABBc *RayMarch::getA() const {
    return a;
}

void RayMarch::bindSpheres(MPMIntegrationSim *mpm) {
    static bool isAllocated = false;
    size_t numParticles = mpm->getParticleAmount();

    if (!isAllocated) {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, spheresSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, numParticles * sizeof(glm::vec4), nullptr, GL_STREAM_DRAW);
        
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, pigmentsSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, numParticles * sizeof(std::array<float, 7>), nullptr, GL_STREAM_DRAW);
        
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, diffusionSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, numParticles * sizeof(float), nullptr, GL_STREAM_DRAW);
        
        isAllocated = true;
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, spheresSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, numParticles * sizeof(glm::vec4), mpm->getParticles().data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start, spheresSSBO);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, pigmentsSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, numParticles * sizeof(std::array<float, 7>), mpm->getPigments().data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start + 8, pigmentsSSBO);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, diffusionSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, numParticles * sizeof(float), mpm->getDiffusionFactors().data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start + 9, diffusionSSBO);
}

void RayMarch::renderTex(Camera* camera) {
    // Enable Alpha Blending so the raymarched object is visible over the skybox
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Enable depth buffer writing for subsequent mesh composition
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_ALWAYS); // Always overwrite the values (raymarching is first in order above the background)

    texShader->use();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, state.fullRender ? postProcessTex : outputTex);
    texShader->setUniform("renderedImage", 0);

    // Pass normalDepthTex so the frag shader can read the position and calculate gl_FragDepth
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, normalDepthTex);
    texShader->setUniform("normalDepthTex", 1);

    // Matrices needed for computeDepth()
    texShader->setUniform("view", camera->getView());
    texShader->setUniform("proj", camera->getProj());

    texShader->setUniform("fullRender", state.fullRender);

    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
    glDepthFunc(GL_LESS); // Revert to standard depth testing
}


void RayMarch::march(GLint ww, GLint wh, MPMIntegrationSim *mpm, Camera *camera) {
    resizeTexutres(ww, wh);

    static bool first_run = true;
    bool update_physics = state.play || state.recalcMarchParams || first_run;

#ifdef MEASURE_TIME
    ctimer.start(1);
#endif

    if (update_physics) {
        a->fixedAABB(mpm);
        bdg->createVectors(a, mpm);
        bdg->fillBDG(a, mpm);
        // Pass 1: Set grid data so neighbors can be queried by Classification
        mpm->setGridData(bdg, a);
    }

    // Pass 2: Generate depth maps & calculate anisotropic matrices
    depthMaps->generateDepthMaps(mpm, ww, wh, camera);
    // Pass 3: Fill pigment grid using the newly computed anisotropic matrices
    if(state.fullRender && update_physics) {
        bdg->fillRenderGrid(mpm, a, depthMaps->getc()->getMatrices());
    }

    const float transparentClear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    glClearTexImage(outputTex, 0, GL_RGBA, GL_FLOAT, transparentClear);
    glClearTexImage(normalDepthTex, 0, GL_RGBA, GL_FLOAT, clearData);
    shader->use();
    bindSpheres(mpm);

    glBindImageTexture(0, outputTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    glBindImageTexture(1, normalDepthTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    bdg->bindBuffers(start+1);

    depthMaps->bindDepthMaps(2, shader);

    // Bind the HDR map to a free slot 5
    if (hdrTexture) {
        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_CUBE_MAP, hdrTexture);
        shader->setUniform("hdrMap", 5);
    }
    
    // Bind the Irradiance map to slot 6
    if (irradianceTexture) {
        glActiveTexture(GL_TEXTURE6);
        glBindTexture(GL_TEXTURE_CUBE_MAP, irradianceTexture);
        shader->setUniform("irradianceMap", 6);
    }

    // Bind the BRDF integration texture to slot 7
    if (brdfLUTTexture) {
        glActiveTexture(GL_TEXTURE7);
        glBindTexture(GL_TEXTURE_2D, brdfLUTTexture);
        shader->setUniform("brdfLUT", 7);
    }

    shader->setUniform("width", ww);
    shader->setUniform("height", wh);
    shader->setUniform("viewAngle", camera->viewAngle);
    shader->setUniform("view", camera->getView());
    shader->setUniform("proj", camera->getProj());
    shader->setUniform("invView", camera->getInvView());
    shader->setUniform("invProj", camera->getInvProj());
    shader->setUniform("maxStepCount", state.maxStepCount);
    shader->setUniform("maxSkipCount", state.maxSkipCount);
    shader->setUniform("gridStart", a->gridStart);
    shader->setUniform("voxelSize", a->voxelS);
    shader->setUniform("h", mpm->getSupportRadius());
    // shader->setUniform("DforRIJ", state.vdall);
    shader->setUniform("DforRIJ", mpm->getRadius());
    shader->setUniform("rad", mpm->getRadius());
    shader->setUniform("cellsSize", glm::ivec3(a->cellsX, a->cellsY, a->cellsZ));
    shader->setUniform("iso", state.iso);
    shader->setUniform("stepsInside", state.stepsInside);
    shader->setUniform("A", state.A);
    shader->setUniform("B", state.B);
    shader->setUniform("maxLevel", state.aa[state.currRes]);
    shader->setUniform("isAni", state.isAni);
    shader->setUniform("showDiffusion", state.showDiffusion);
    shader->setUniform("showNormals", state.showNormals);
    shader->setUniform("pigment_D_edge0", mpm->getPigmentDEdge0());
    shader->setUniform("pigment_D_edge1", mpm->getPigmentDEdge1());
    shader->setUniform("sigma_color", state.sigma_color);
    shader->setUniform("sigma_spatial", state.sigma_spatial);
    shader->setUniform("fullRender", state.fullRender);

    shader->setUniform("fluidMat.albedo", fluidMat.albedo);
    shader->setUniform("fluidMat.metallic", fluidMat.metallic);
    shader->setUniform("fluidMat.roughness", fluidMat.roughness);

    shader->setUniform("spatulaMat.albedo", spatulaMetal.albedo);
    shader->setUniform("spatulaMat.metallic", spatulaMetal.metallic);
    shader->setUniform("spatulaMat.roughness", spatulaMetal.roughness);

    shader->setUniform("floorMat.albedo", floorMat.albedo);
    shader->setUniform("floorMat.metallic", floorMat.metallic);
    shader->setUniform("floorMat.roughness", floorMat.roughness);

    // Bind spatula uniforms
    shader->setUniform("invSpatulaTransform", mpm->getSpatulaInvTransform());
    shader->setUniform("has_spatula", mpm->spatulaExists());
    shader->setUniform("spatulaDim", mpm->getSpatulaDim());
    
    first_run = false;
    state.recalcMarchParams = false;

    std::vector<glm::vec3> finalLightColors;
    finalLightColors.reserve(lightDirs.size());
    for (size_t i = 0; i < lightDirs.size(); ++i) {
        finalLightColors.push_back(lightColors[i] * lightIntensities[i]);
    }

    shader->setUniform("numLights", (int)lightDirs.size());
    shader->setUniform("lightDirs", lightDirs);
    shader->setUniform("lightColors", finalLightColors);

#ifdef MEASURE_TIME
    ctimer.end(1);
#endif
    GLuint groupCountX = (ww + state.groupSizeRayMarching.x - 1) / state.groupSizeRayMarching.x;
    GLuint groupCountY = (wh + state.groupSizeRayMarching.y - 1) / state.groupSizeRayMarching.y;
    // glBeginQuery(GL_TIME_ELAPSED, timer.queries[1]);
    glDispatchCompute(groupCountX, groupCountY, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
    
    if (state.fullRender) {
        // Bilateral blur Pass 1 (Horizontal)
        bilateralShader->use();
        bilateralShader->setUniform("width", ww);
        bilateralShader->setUniform("height", wh);
        bilateralShader->setUniform("blurDirection", glm::ivec2(1, 0));
        bilateralShader->setUniform("isFinalPass", false);
        
        glBindImageTexture(0, outputTex, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
        glBindImageTexture(1, normalDepthTex, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
        glBindImageTexture(2, postProcessPingPongTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
        
        glDispatchCompute(groupCountX, groupCountY, 1);
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
        
        // Bilateral blur Pass 2 (Vertical)
        bilateralShader->setUniform("blurDirection", glm::ivec2(0, 1));
        bilateralShader->setUniform("isFinalPass", true);
        
        glBindImageTexture(0, postProcessPingPongTex, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
        // normalDepthTex remains bound to binding 1
        glBindImageTexture(2, postProcessTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
        
        glDispatchCompute(groupCountX, groupCountY, 1);
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
        
        renderSkybox(camera);
    }
    
    renderTex(camera);            // Step 1: Render the quad and set gl_FragDepth
    if (mpm->spatulaExists() && spatulaMesh) {
        // Activate the spatula shader BEFORE setting textures and uniforms.
        // This ensures that all subsequent operations apply to the correct shader program.
        spatulaShader->use();

        // Set uniforms that were previously in SpatulaMesh::render
        spatulaShader->setUniform("invSpatulaTransform", mpm->getSpatulaInvTransform());
        spatulaShader->setUniform("view", camera->getView());
        spatulaShader->setUniform("projection", camera->getProj());
        spatulaShader->setUniform("camPos", camera->cameraPos);
        spatulaShader->setUniform("fullRender", state.fullRender);

        spatulaShader->setUniform("numLights", (int)lightDirs.size());
        spatulaShader->setUniform("lightDirs", lightDirs);
        spatulaShader->setUniform("lightColors", finalLightColors);
        // Bind PBR textures needed for IBL (Image-Based Lighting).
        // Without them, `texture(irradianceMap, ...)` would return black, causing black reflections.
        if (hdrTexture) { glActiveTexture(GL_TEXTURE5); glBindTexture(GL_TEXTURE_CUBE_MAP, hdrTexture); spatulaShader->setUniform("hdrMap", 5); }
        if (irradianceTexture) { glActiveTexture(GL_TEXTURE6); glBindTexture(GL_TEXTURE_CUBE_MAP, irradianceTexture); spatulaShader->setUniform("irradianceMap", 6); }
        if (brdfLUTTexture) { glActiveTexture(GL_TEXTURE7); glBindTexture(GL_TEXTURE_2D, brdfLUTTexture); spatulaShader->setUniform("brdfLUT", 7); }

        // Set materials as an array (0: metal, 1: wood)
        spatulaShader->setUniform("spatulaMaterials[0].albedo", spatulaMetal.albedo);
        spatulaShader->setUniform("spatulaMaterials[0].metallic", spatulaMetal.metallic);
        spatulaShader->setUniform("spatulaMaterials[0].roughness", spatulaMetal.roughness);
        spatulaShader->setUniform("spatulaMaterials[1].albedo", spatulaWood.albedo);
        spatulaShader->setUniform("spatulaMaterials[1].metallic", spatulaWood.metallic);
        spatulaShader->setUniform("spatulaMaterials[1].roughness", spatulaWood.roughness);
        
        // Before rendering the spatula mesh, we must pass the floor material to the shader,
        // because pbr_lighting.glsl requires it for correct reflections.
        spatulaShader->setUniform("floorMat.albedo", floorMat.albedo);
        spatulaShader->setUniform("floorMat.metallic", floorMat.metallic);
        spatulaShader->setUniform("floorMat.roughness", floorMat.roughness);

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS); // Render only what is in front of the fluid

        glBindVertexArray(spatulaMesh->getVAO());
        glDrawArrays(GL_TRIANGLES, 0, spatulaMesh->getVertexCount());
        glBindVertexArray(0);
    }
    // timer.end();
    // glEndQuery(GL_TIME_ELAPSED);
}
