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
    
    HDRLoader::loadHDRCubemap("data/SkyMap.hdr", hdrTexture, irradianceTexture);

    spatulaMesh = new SpatulaMesh("data/spatula_handler.obj"); // Nebo "../data/spatula_handler.obj" v závislosti na pracovním adresáři
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
    
    // Zkreslíme skybox za všemi ostatními prvky
    glDepthFunc(GL_LEQUAL); 
    skyboxShader->use();

    // Odstraníme translaci z matice pohledu, aby se kamera mohla jen otáčet a neutíkala ven z boxu
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
    // Používáme RG formát, protože ukládáme jen parametry A a B (scale a bias pro Fresnel)
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
    
    // Využijeme stejný vertex shader pro full-screen quad jako při běžném vykreslování textury
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
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, spheresSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        mpm->getParticleAmount() * sizeof(glm::vec4),
        nullptr,
        GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
        mpm->getParticleAmount() * sizeof(glm::vec4), mpm->getParticles().data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start, spheresSSBO);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, pigmentsSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        mpm->getParticleAmount() * sizeof(std::array<float, 7>),
        nullptr,
        GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
        mpm->getParticleAmount() * sizeof(std::array<float, 7>), mpm->getPigments().data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start + 8, pigmentsSSBO);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, diffusionSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        mpm->getParticleAmount() * sizeof(float),
        nullptr,
        GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
        mpm->getParticleAmount() * sizeof(float), mpm->getDiffusionFactors().data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start + 9, diffusionSSBO);
}

void RayMarch::renderTex(Camera* camera) {
    // Aktivujeme Alpha Blending, aby byl raymarched objekt vidět nad skyboxem
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // ZMĚNA: Zapneme zápis do depth bufferu pro následnou kompozici meshe
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_ALWAYS); // Vždy přepíšeme hodnoty (raymarching je první v pořadí nad pozadím)

    texShader->use();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, state.fullRender ? postProcessTex : outputTex);
    texShader->setUniform("renderedImage", 0);

    // Předáme normalDepthTex, aby si frag shader mohl přečíst pozici a spočítat gl_FragDepth
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, normalDepthTex);
    texShader->setUniform("normalDepthTex", 1);

    // Matice potřebné pro computeDepth()
    texShader->setUniform("view", camera->getView());
    texShader->setUniform("proj", camera->getProj());

    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
    glDepthFunc(GL_LESS); // Vrátíme zpět na klasické testování hloubky
}


void RayMarch::march(GLint ww, GLint wh, MPMIntegrationSim *mpm, Camera *camera) {
    resizeTexutres(ww, wh);
    // if (state.play) {
    ctimer.start(1);
    a->fixedAABB(mpm);
    bdg->createVectors(a, mpm);
    bdg->fillBDG(a, mpm);
    
    // Pass 1: Set grid data so neighbors can be queried by Classification
    mpm->setGridData(bdg, a);
    // Pass 2: Generate depth maps & calculate anisotropic matrices
    depthMaps->generateDepthMaps(mpm, ww, wh, camera);
    // Pass 3: Fill pigment grid using the newly computed anisotropic matrices
    bdg->fillRenderGrid(mpm, a, depthMaps->getc()->getMatrices());

    // }
    const float transparentClear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    glClearTexImage(outputTex, 0, GL_RGBA, GL_FLOAT, transparentClear);
    glClearTexImage(normalDepthTex, 0, GL_RGBA, GL_FLOAT, clearData);
    shader->use();
    bindSpheres(mpm);

    glBindImageTexture(0, outputTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    glBindImageTexture(1, normalDepthTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    bdg->bindBuffers(start+1);

    depthMaps->bindDepthMaps(2, shader);

    // Bindneme HDR mapu na volny slot 5
    if (hdrTexture) {
        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_CUBE_MAP, hdrTexture);
        shader->setUniform("hdrMap", 5);
    }
    
    // Bindneme Irradiance mapu na slot 6
    if (irradianceTexture) {
        glActiveTexture(GL_TEXTURE6);
        glBindTexture(GL_TEXTURE_CUBE_MAP, irradianceTexture);
        shader->setUniform("irradianceMap", 6);
    }

    // Bindneme BRDF integrační texturu na slot 7
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

    // Směrová světla pro PBR (D, F, G) a stíny
    shader->setUniform("lightDirs[0]", lightDirs[0]);
    shader->setUniform("lightDirs[1]", lightDirs[1]);
    shader->setUniform("lightColors[0]", lightColors[0]);
    shader->setUniform("lightColors[1]", lightColors[1]);

    GLuint groupCountX = (ww + state.groupSizeRayMarching.x - 1) / state.groupSizeRayMarching.x;
    GLuint groupCountY = (wh + state.groupSizeRayMarching.y - 1) / state.groupSizeRayMarching.y;
    // glBeginQuery(GL_TIME_ELAPSED, timer.queries[1]);
    glDispatchCompute(groupCountX, groupCountY, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
    
    if (state.fullRender) {
        // Bilateral blur pass pro vyhlazení stínů podložky (pouze v plném renderu)
        bilateralShader->use();
        bilateralShader->setUniform("width", ww);
        bilateralShader->setUniform("height", wh);
        glBindImageTexture(0, outputTex, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
        glBindImageTexture(1, normalDepthTex, 0, GL_FALSE, 0, GL_READ_ONLY, GL_RGBA32F);
        glBindImageTexture(2, postProcessTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
        glDispatchCompute(groupCountX, groupCountY, 1);
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
        
        renderSkybox(camera);
    }
    
    renderTex(camera);            // Krok 1: Vykreslí plochu a nastaví gl_FragDepth
    if (mpm->spatulaExists() && spatulaMesh) {
        SpatulaMaterial woodMat = {spatulaWood.albedo, spatulaWood.metallic, spatulaWood.roughness};
        SpatulaMaterial metalMat = {spatulaMetal.albedo, spatulaMetal.metallic, spatulaMetal.roughness};
        spatulaMesh->render(spatulaShader, mpm->getSpatulaInvTransform(), camera, state.fullRender, hdrTexture, irradianceTexture, brdfLUTTexture, woodMat, metalMat, lightDirs, lightColors);
    }
    // timer.end();
    // glEndQuery(GL_TIME_ELAPSED);
}
