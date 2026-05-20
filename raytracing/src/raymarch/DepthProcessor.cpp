#include "DepthProcessor.h"

#include "PassTimer.h"

DepthProcessor::DepthProcessor() {
    genBuffers();
    genFbo();
    initShader();
    c = new Classification();
}

DepthProcessor::~DepthProcessor() {
    glDeleteFramebuffers(1, &fboDagg);
    glDeleteFramebuffers(1, &fboDall);
    glDeleteTextures(1, &Dagg);
    glDeleteTextures(1, &Dall);
    glDeleteTextures(1, &DallSmooth);
    glDeleteTextures(1, &DallTMP);
    glDeleteTextures(1, &Nscreen);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &VBOMat);
    glDeleteBuffers(1, &EBODagg);
    glDeleteBuffers(1, &EBODall);
    glDeleteVertexArrays(1, &VAODagg);
    glDeleteVertexArrays(1, &VAODall);
    glDeleteRenderbuffers(1, &rdepth);
    delete shader;
    delete gaussBilateral;
    delete compN;
}

void DepthProcessor::genBuffers() {
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &VBOMat);
    glGenBuffers(1, &EBODagg);
    glGenBuffers(1, &EBODall);
    glGenVertexArrays(1, &VAODagg);

    //VAO DAGG
    glBindVertexArray(VAODagg);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE,
        sizeof(glm::vec4), (void*)0);
    //binding matrix
    glBindBuffer(GL_ARRAY_BUFFER, VBOMat);
    for (int i = 0; i < 3; ++i) {
        glEnableVertexAttribArray(1 + i);
        glVertexAttribPointer(1 + i, 3, GL_FLOAT, GL_FALSE,
            sizeof(glm::mat3), (void*)(i*sizeof(glm::vec3)));
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBODagg);

    //VAO DALL
    glGenVertexArrays(1, &VAODall);
    glBindVertexArray(VAODall);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE,
        sizeof(glm::vec4), (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, VBOMat);
    for (int i = 0; i < 3; ++i) {
        glEnableVertexAttribArray(1 + i);
        glVertexAttribPointer(1 + i, 3, GL_FLOAT, GL_FALSE,
            sizeof(glm::mat3), (void*)(i*sizeof(glm::vec3)));
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBODall);

    glBindVertexArray(0);

    //Nscreen normals
    glGenTextures(1, &Nscreen);
    glBindTexture(GL_TEXTURE_2D, Nscreen);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
        10, 10, 0, GL_RGBA, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    //Smooth Dall
    glGenTextures(1, &DallSmooth);
    glBindTexture(GL_TEXTURE_2D, DallSmooth);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F,
        10, 10, 0, GL_RED, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    //TMP texture to store values for separable kernel
    glGenTextures(1, &DallTMP);
    glBindTexture(GL_TEXTURE_2D, DallTMP);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F,
        10, 10, 0, GL_RED, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void DepthProcessor::genFbo() {
    glGenRenderbuffers(1, &rdepth);

    glGenTextures(1, &Dagg);
    glBindTexture(GL_TEXTURE_2D, Dagg);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F,
        10, 10, 0, GL_RED, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenFramebuffers(1, &fboDagg);
    glBindFramebuffer(GL_FRAMEBUFFER, fboDagg);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        Dagg, 0);
    GLenum d = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, &d);
    glBindRenderbuffer(GL_RENDERBUFFER, rdepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, 10, 10);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER, rdepth);
    glReadBuffer(GL_NONE);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        spdlog::error("Framebuffer is not complete!");

    glGenTextures(1, &Dall);
    glBindTexture(GL_TEXTURE_2D, Dall);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F,
        10, 10, 0, GL_RED, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenFramebuffers(1, &fboDall);
    glBindFramebuffer(GL_FRAMEBUFFER, fboDall);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        Dall, 0);
    glDrawBuffers(1, &d);
    glBindRenderbuffer(GL_RENDERBUFFER, rdepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, 10, 10);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER, rdepth);
    glReadBuffer(GL_NONE);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        spdlog::error("Framebuffer 2 is not complete!");

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void DepthProcessor::initShader() {
    shader = new Shader("../shaders/raymarching/DepthMap.vert", "../shaders/raymarching/DepthMap.frag");
    gaussBilateral = new Shader("../shaders/raymarching/GaussBilateral.glsl");
    compN = new Shader("../shaders/raymarching/NScreen.glsl");
}

void DepthProcessor::bindDall(Shader *sh) {
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, Dall);
    sh->setUniform("Dall", 3);
}


void DepthProcessor::bindDepthMaps(int start, Shader* sh) const {
    glActiveTexture(GL_TEXTURE0 + start);
    glBindTexture(GL_TEXTURE_2D, Dagg);
    sh->setUniform("Dagg", start);

    glActiveTexture(GL_TEXTURE0 + start + 1);
    glBindTexture(GL_TEXTURE_2D, Dall);
    sh->setUniform("Dall", start + 1);

    glActiveTexture(GL_TEXTURE0 + start + 2);
    glBindTexture(GL_TEXTURE_2D, Nscreen);
    sh->setUniform("Nscreen", start + 2);

    c->bindBuffers(6);
}

void DepthProcessor::smoothDall(MPMIntegrationSim *mpm, GLint ww, GLint wh, Camera *camera) {
    gaussBilateral->use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, Dall);
    gaussBilateral->setUniform("Di", 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, Dall);
    gaussBilateral->setUniform("Dall", 1);

    glBindImageTexture(2, DallTMP, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);

    gaussBilateral->setUniform("width", ww);
    gaussBilateral->setUniform("height", wh);
    gaussBilateral->setUniform("r", state.seeSpheres ? mpm->getRadius() : mpm->getSupportRadius());
    gaussBilateral->setUniform("fov", camera->viewAngle);
    gaussBilateral->setUniform("ks", state.ks);
    gaussBilateral->setUniform("sigma_r", state.kr * (state.seeSpheres ? mpm->getRadius() : mpm->getSupportRadius()));
    gaussBilateral->setUniform("filterR", state.R);
    gaussBilateral->setUniform("dir", glm::ivec2(1, 0));

    int gx = (ww + state.groupSizeRayMarching.x - 1) / state.groupSizeRayMarching.x;
    int gy = (wh + state.groupSizeRayMarching.y - 1) / state.groupSizeRayMarching.y;

    glDispatchCompute(gx, gy, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);

    //Second pass through y

    gaussBilateral->setUniform("dir", glm::ivec2(0, 1));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, DallTMP);
    gaussBilateral->setUniform("Di", 0);

    glBindImageTexture(2, DallSmooth, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
    glDispatchCompute(gx, gy, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
}

void DepthProcessor::computeNScreen(GLint ww, GLint wh, Camera *camera) {
    compN->use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, DallSmooth);
    compN->setUniform("Dsmooth", 0);
    glBindImageTexture(1, Nscreen, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    compN->setUniform("width", ww);
    compN->setUniform("height", wh);
    compN->setUniform("invProj", camera->getInvProj());
    int gx = (ww + state.groupSizeRayMarching.x - 1) / state.groupSizeRayMarching.x;
    int gy = (wh + state.groupSizeRayMarching.y - 1) / state.groupSizeRayMarching.y;
    glDispatchCompute(gx, gy, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
}

void DepthProcessor::generateDepthMaps(MPMIntegrationSim *mpm, GLint ww, GLint wh, Camera *camera) {
    idsDagg.clear();
    idsDall.clear();
#ifdef MEASURE_TIME
    ctimer.start(2);
#endif
    c->countDensities(mpm, idsDagg, idsDall);
#ifdef MEASURE_TIME
    ctimer.end(2);
#endif
    buffers(mpm);
    resizeTextures(ww, wh);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_BLEND);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    shader->use();
    shader->setUniform("view", camera->getView());
    shader->setUniform("proj", camera->getProj());
    shader->setUniform("wh", wh);
    //using h instead of radius to be able to get particles density
    shader->setUniform("h", state.seeSpheres ? mpm->getRadius() : mpm->getSupportRadius());
    shader->setUniform("isAni", state.isAni);

    glBindFramebuffer(GL_FRAMEBUFFER, fboDagg);
    glViewport(0, 0, ww, wh);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBindVertexArray(VAODagg);
    glDrawElements(GL_POINTS, idsDagg.size(), GL_UNSIGNED_INT, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, fboDall);
    glViewport(0, 0, ww, wh);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBindVertexArray(VAODall);
    glDrawElements(GL_POINTS, idsDall.size(), GL_UNSIGNED_INT, 0);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
    glBindVertexArray(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    smoothDall(mpm, ww, wh, camera);
    computeNScreen(ww, wh, camera);
}

void DepthProcessor::resizeTextures(GLint ww, GLint wh) {
    glBindTexture(GL_TEXTURE_2D, Dagg);
    GLint x, y;
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH,  &x);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &y);
    if (ww != x || wh != y) {
        spdlog::debug("Changing size of textures for depth maps to {} {}", ww, wh);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F,
            ww, wh, 0, GL_RED, GL_FLOAT, nullptr);

        glBindTexture(GL_TEXTURE_2D, Dall);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F,
            ww, wh, 0, GL_RED, GL_FLOAT, nullptr);

        glBindTexture(GL_TEXTURE_2D, DallSmooth);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F,
            ww, wh, 0, GL_RED, GL_FLOAT, nullptr);

        glBindTexture(GL_TEXTURE_2D, DallTMP);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F,
                    ww, wh, 0, GL_RED, GL_FLOAT, nullptr);

        glBindTexture(GL_TEXTURE_2D, Nscreen);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F,
                    ww, wh, 0, GL_RGBA, GL_FLOAT, nullptr);

        glBindRenderbuffer(GL_RENDERBUFFER, rdepth);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, ww, wh);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
}

void DepthProcessor::buffers(MPMIntegrationSim *mpm) {
    //VBO
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, mpm->getParticleAmount() * sizeof(glm::vec4),
        nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
        mpm->getParticleAmount() * sizeof(glm::vec4),mpm->getParticles().data());

    //VBOMat
    glBindBuffer(GL_ARRAY_BUFFER, VBOMat);
    glBufferData(GL_ARRAY_BUFFER, mpm->getParticleAmount() * sizeof(glm::mat3),
        nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
        mpm->getParticleAmount() * sizeof(glm::mat3), c->getMatricesInv().data());

    //EBO DAGG
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBODagg);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idsDagg.size() * sizeof(unsigned),
        nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
        idsDagg.size() * sizeof(unsigned), idsDagg.data());

    //EBO DALL
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBODall);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idsDall.size() * sizeof(unsigned),
        nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
        idsDall.size() * sizeof(unsigned), idsDall.data());

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

}
