#include "RayMarch.h"

#include "PassTimer.h"
#include "HDRLoader.h"

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
    initSkybox();
    
    HDRLoader::loadHDRCubemap("data/SkyMap.hdr", hdrTexture, irradianceTexture);
}

RayMarch::~RayMarch() {
    delete depthMaps;
    delete bdg;
    delete shader;
    delete texShader;
    delete skyboxShader;
    glDeleteBuffers(1, &spheresSSBO);
    glDeleteBuffers(1, &pigmentsSSBO);
    glDeleteBuffers(1, &diffusionSSBO);
    glDeleteTextures(1, &outputTex);
    glDeleteTextures(1, &normalDepthTex);
    if (hdrTexture) {
        glDeleteTextures(1, &hdrTexture);
    }
    if (irradianceTexture) {
        glDeleteTextures(1, &irradianceTexture);
    }
    glDeleteBuffers(1, &quadVBO);
    glDeleteVertexArrays(1, &quadVAO);
    glDeleteVertexArrays(1, &skyboxVAO);
    glDeleteBuffers(1, &skyboxVBO);
}

void RayMarch::initShader() {
    shader = new Shader("../shaders/raymarching/raymarch.glsl");
    texShader = new Shader("../shaders/raymarching/shader.vert", "../shaders/raymarching/shader.frag");
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

void RayMarch::renderTex() {
    // Aktivujeme Alpha Blending, aby byl raymarched objekt vidět nad skyboxem
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    texShader->use();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, outputTex);

    texShader->setUniform("renderedImage", 0);

    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
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

    // Bind spatula uniforms
    shader->setUniform("invSpatulaTransform", mpm->getSpatulaInvTransform());
    shader->setUniform("has_spatula", mpm->spatulaExists());
    shader->setUniform("spatulaDim", mpm->getSpatulaDim());

    GLuint groupCountX = (ww + state.groupSizeRayMarching.x - 1) / state.groupSizeRayMarching.x;
    GLuint groupCountY = (wh + state.groupSizeRayMarching.y - 1) / state.groupSizeRayMarching.y;
    // glBeginQuery(GL_TIME_ELAPSED, timer.queries[1]);
    glDispatchCompute(groupCountX, groupCountY, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
    
    if (state.fullRender) {
        renderSkybox(camera);
    }
    
    renderTex();
    // timer.end();
    // glEndQuery(GL_TIME_ELAPSED);
}
