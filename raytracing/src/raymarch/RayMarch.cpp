#include "RayMarch.h"

#include "PassTimer.h"


RayMarch::RayMarch(MPMIntegrationSim *mpm, AABBc *a) {
    glGenBuffers(1, &spheresSSBO);
    depthMaps = new DepthProcessor();
    this->a = a;
    bdg = new BinaryDensityGrid(a, mpm);
    initShader();
    texQuadInit();
    createOutputTexture();
}

RayMarch::~RayMarch() {
    delete depthMaps;
    delete bdg;
    delete shader;
    delete texShader;
    delete interpolation;
    glDeleteBuffers(1, &spheresSSBO);
    glDeleteTextures(1, &outputTex);
    glDeleteTextures(1, &normalDepthTex);
    glDeleteBuffers(1, &quadVBO);
    glDeleteVertexArrays(1, &quadVAO);
}

void RayMarch::initShader() {
    shader = new Shader("../shaders/raymarching/raymarch.glsl");
    texShader = new Shader("../shaders/raymarching/shader.vert", "../shaders/raymarching/shader.frag");
    interpolation = new Shader("../shaders/raymarching/Billinearinterpolation.glsl");
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
}

void RayMarch::renderTex() {
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    texShader->use();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, outputTex);

    texShader->setUniform("renderedImage", 0);

    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}


void RayMarch::march(GLint ww, GLint wh, MPMIntegrationSim *mpm, Camera *camera) {
    //TODO: count time for every step to test for bottlenecks
    resizeTexutres(ww, wh);
    // if (state.play) {
    ctimer.start(1);
    bdg->fillBDG(a, mpm);
    ctimer.end(1);
    depthMaps->generateDepthMaps(mpm, ww, wh, camera);
    // timer.start();
    // }
    glClearTexImage(outputTex, 0, GL_RGBA, GL_FLOAT, floorCol);
    glClearTexImage(normalDepthTex, 0, GL_RGBA, GL_FLOAT, clearData);
    shader->use();
    bindSpheres(mpm);

    glBindImageTexture(0, outputTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    glBindImageTexture(1, normalDepthTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    bdg->bindBuffers(start+1);

    depthMaps->bindDepthMaps(2, shader);
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
    // shader->setUniform("DforRIJ", state.seeSpheres ? spph->getRadius() : spph->getSupportRadius());
    shader->setUniform("rad", mpm->getRadius());
    shader->setUniform("cellsSize", glm::ivec3(a->cellsX, a->cellsY, a->cellsZ));
    shader->setUniform("iso", state.iso);
    shader->setUniform("stepsInside", state.stepsInside);
    shader->setUniform("A", state.A);
    shader->setUniform("B", state.B);
    shader->setUniform("debugMode", state.debugMode);
    shader->setUniform("maxLevel", state.aa[state.currRes]);
    shader->setUniform("isAni", state.isAni);


    GLuint groupCountX = (ww + state.groupSizeRayMarching.x - 1) / state.groupSizeRayMarching.x;
    GLuint groupCountY = (wh + state.groupSizeRayMarching.y - 1) / state.groupSizeRayMarching.y;
    // glBeginQuery(GL_TIME_ELAPSED, timer.queries[1]);
    shader->setUniform("stride", 1);
    glDispatchCompute(groupCountX, groupCountY, 1);
    if (!state.testFullRes) {
        shader->setUniform("stride", 2);
    glDispatchCompute((groupCountX + 2 - 1) / 2, (groupCountY + 2 - 1) / 2, 1);
    shader->setUniform("stride", 4);
    glDispatchCompute((groupCountX + 4 - 1) / 4, (groupCountY + 4 - 1) / 4, 1);
    }


    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
    if (!state.debugMode || !state.testFullRes) {
        interpolation->use();
        interpolation->setUniform("width", ww);
        interpolation->setUniform("height", wh);
        interpolation->setUniform("maxdv", mpm->getSupportRadius() * 2);
        glBindImageTexture(0, outputTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
        glBindImageTexture(1, normalDepthTex, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
        interpolation->setUniform("stride", 4);
        depthMaps->bindDall(interpolation);
        glDispatchCompute(groupCountX, groupCountY, 1);
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
        interpolation->setUniform("stride", 2);
        glDispatchCompute(groupCountX, groupCountY, 1);
        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
    }
    renderTex();
    // timer.end();
    // glEndQuery(GL_TIME_ELAPSED);
}


