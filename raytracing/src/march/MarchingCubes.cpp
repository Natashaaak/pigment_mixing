#include "MarchingCubes.h"

#include "raymarch/AABBc.h"

MarchingCubes::MarchingCubes() {
    initShader();
    createBind();
}

MarchingCubes::~MarchingCubes() {
    delete densityShader;
    delete marchShader;
    delete tr;
    glDeleteTextures(1, &tex);
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &meshSSBO);
    glDeleteBuffers(1, &counterBuf);
    glDeleteBuffers(1, &spheresSSBO);
}

void MarchingCubes::countAABB(const std::vector<glm::vec4> &spheresData) {
    glm::vec3 maxG(-1e20f), minG(1e20f);
    voxelS = spheresData[0].w;
    for (auto sphere : spheresData) {
        minG = glm::min(minG, glm::vec3(sphere.x, sphere.y, sphere.z) - glm::vec3(sphere.w)); //left_down corner
        maxG = glm::max(maxG, glm::vec3(sphere.x, sphere.y, sphere.z) + glm::vec3(sphere.w)); //right_up corner
    }
    glm::vec3 extent = maxG - minG; //diagonal
    float maxExtent = std::max(extent.x, std::max(extent.y, extent.z));
    //voxelSize = maxExtent / gridSize;
    voxelS = std::min(voxelS, maxExtent / std::min(state.gridSize, static_cast<int>(state.maxGridResolution)));
    glm::vec3 rawOrigin = minG;
    gridStart = glm::floor(rawOrigin / voxelS) * voxelS;

    float a = std::max(std::max(extent.x/voxelS, extent.y/voxelS), extent.z/voxelS);
    if (a > state.maxGridResolution) {
        voxelS *= a/state.maxGridResolution;
    }
    cellsX = std::ceil(extent.x / voxelS) + 10;
    cellsY = std::ceil(extent.y / voxelS) + 25;
    cellsZ = std::ceil(extent.z / voxelS) + 10;
}

void MarchingCubes::create3DTexture() {
    GLint x, y, z;
    glBindTexture(GL_TEXTURE_3D, tex);
    glGetTexLevelParameteriv(GL_TEXTURE_3D, 0, GL_TEXTURE_WIDTH,  &x);
    glGetTexLevelParameteriv(GL_TEXTURE_3D, 0, GL_TEXTURE_HEIGHT, &y);
    glGetTexLevelParameteriv(GL_TEXTURE_3D, 0, GL_TEXTURE_DEPTH,  &z);
    if (x != cellsX+1 || y != cellsY+1 || z != cellsZ+1) {
        glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, cellsX+1, cellsY+1, cellsZ+1, 0,
         GL_RGBA, GL_FLOAT, nullptr);
    }

}

void MarchingCubes::computeDensity(const std::vector<glm::vec4> &spheresData, int numSpheres, SPHIntegrationSim *spph) {
    if (matrices.empty())
        matrices.resize(spph->getParticleAmount());
    anisotropicKernelCount(spheresData, numSpheres, spph);
    densityShader->use();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, spheresSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    numSpheres * sizeof(aniso),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, numSpheres * sizeof(aniso), matrices.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, spheresSSBO);

    densityShader->setUniform("spheresCount", numSpheres);
    densityShader->setUniform("cells", glm::ivec3(cellsX+1, cellsY+1, cellsZ+1));
    densityShader->setUniform("voxelS", voxelS);
    densityShader->setUniform("gridStart", gridStart);
    densityShader->setUniform("h", spph->getSupportRadius());

    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 2, counterBuf);

    GLuint gcX = (cellsX + 1 + state.localGroupSize.x - 1) / state.localGroupSize.x;
    GLuint gcY = (cellsY + 1 + state.localGroupSize.y - 1) / state.localGroupSize.y;
    GLuint gcZ = (cellsZ + 1 + state.localGroupSize.z - 1) / state.localGroupSize.z;
    create3DTexture();
    glBindImageTexture(0, tex, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    glDispatchCompute(gcX, gcY, gcZ);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void MarchingCubes::march(Camera *camera) {
    marchShader->use();

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, meshSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, cellsX * cellsY * cellsZ * 8 * 3 * sizeof(glm::vec4), nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, meshSSBO);

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counterBuf);
    GLuint zero = 0;
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(GLuint), &zero, GL_STREAM_DRAW);
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 1, counterBuf);

    marchShader->setUniform("cells", glm::ivec3(cellsX, cellsY, cellsZ));
    marchShader->setUniform("iso", state.iso);
    marchShader->setUniform("voxelS", voxelS);
    marchShader->setUniform("gridStart", gridStart);

    glBindImageTexture(0, tex, 0, GL_TRUE, 0, GL_READ_ONLY, GL_RGBA32F);

    GLuint gcX = (cellsX + state.localGroupSize.x - 1) / state.localGroupSize.x;
    GLuint gcY = (cellsY + state.localGroupSize.y - 1) / state.localGroupSize.y;
    GLuint gcZ = (cellsZ + state.localGroupSize.z - 1) / state.localGroupSize.z;
    glDispatchCompute(gcX, gcY, gcZ);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    glGetBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(GLuint), &state.triCount);

    drawResult(camera);
}

void MarchingCubes::drawResult(Camera *camera) {
    tr->use();
    glm::mat4 P = camera->getProj();

    glm::mat4 V = camera->getView();

    glm::mat4 M = glm::mat4(1.0f);

    tr->setUniform("proj", P);
    tr->setUniform("model", M);
    tr->setUniform("view", V);
    glBindVertexArray(VAO);

    glDrawArrays(GL_TRIANGLES, 0, state.triCount * 3);
    glBindVertexArray(0);
}

void MarchingCubes::createBind() {
    glGenBuffers(1, &spheresSSBO);
    glGenBuffers(1, &meshSSBO);
    glGenBuffers(1, &counterBuf);

    glGenTextures(1, &tex);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, meshSSBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), nullptr);
    glBindVertexArray(0);
}

void MarchingCubes::anisotropicKernelCount(const std::vector<glm::vec4> &spheresData, int numSpheres, SPHIntegrationSim *spph)
{
    float h = spph->getSupportRadius();
#pragma omp parallel for schedule(static)
    for (int i = 0; i < numSpheres; ++i) {
        glm::vec4 xi = spheresData[i];
        glm::vec3 xic(0);
        float wsum = 0;
        glm::mat3 B (1.0f);
        float detB;
        if (state.useAnisotropicKer){
            std::vector<unsigned> neighpos;
            spph->neighborsByIndex(i, neighpos);
            if (neighpos.size() <= state.aniso_threshold) {
                B *= 1.0f / h;
                detB = determinant(B);
                matrices[i] = aniso(glm::vec4(B[0],1), glm::vec4(B[1],1),
                               glm::vec4(B[2],1), glm::vec4(detB), xi);
                continue;
            }
            std::vector<float> wijs;
            wijs.reserve(neighpos.size());
            //Collecting weighs
            for (int j = 0; j < neighpos.size(); ++j) { // we have assurance that all particles are inside h of curr part
                glm::vec3 xj = glm::vec3(spheresData[neighpos[j]]);
                glm::vec3 r = glm::vec3(xi) - xj;
                float l = length(r);
                float wij = 0.0f;
                wij = pow(1.0f - (l / h), 3);
                wsum += wij;
                xic += wij * xj;
                wijs.push_back(wij);
            }
            xic /= wsum;
            glm::mat3 C(0);
            //Constructing covariance matrix
            for (int j = 0; j < neighpos.size(); ++j) {
                wijs[j] /= wsum; //normalizing elements so their sum is 1
                glm::vec3 d = glm::vec3(spheresData[neighpos[j]]) - xic;
                C += wijs[j] * glm::outerProduct(d, d);
            }

            C *= state.s;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(Eigen::Map<Eigen::Matrix3f>(glm::value_ptr(C)));
            Eigen::Vector3f eigenVals = solver.eigenvalues();
            Eigen::Matrix3f eigenV = solver.eigenvectors();

            float maxVal = eigenVals.maxCoeff();
            //Clamping the difference and making it square root
            for(int j = 0; j < 3; ++j) {
                eigenVals[j] = 1.0f / std::sqrt(std::max(eigenVals[j], maxVal / state.k));
            }
            Eigen::Matrix3f B_i = (1.0f / h) * eigenV * eigenVals.asDiagonal() * eigenV.transpose();
            float detB = abs(B_i.determinant());
            if ((state.autoScaleS || !state.useAnisotropicKer) && detB > 0.0f) {
                float td = 1.0f / (h*h*h);
                float s = cbrt(td / detB);
                B_i *= s;
                detB *= s;
            }
            B = glm::make_mat3(B_i.data());
            detB = abs(determinant(B));
        }
        else{
            B *= 1.0f / h;
            detB = determinant(B);
        }
        matrices[i] = aniso(glm::vec4(B[0], 1), glm::vec4(B[1], 1),
            glm::vec4(B[2], 1), glm::vec4(detB), xi);
    }

}

void MarchingCubes::initShader() {
    densityShader = new Shader("../shaders/march/density.glsl");
    marchShader = new Shader("../shaders/march/marchingCubes.glsl");
    tr = new Shader("../shaders/march/tri.vert", "../shaders/march/tri.frag");
}


