#include "Classification.h"

Classification::~Classification() {
    glDeleteBuffers(1, &SSBOmat);
    glDeleteBuffers(1, &SSBOdet);
}

Classification::Classification() {
    glGenBuffers(1, &SSBOmat);
    glGenBuffers(1, &SSBOdet);
}

void Classification::countDensities(MPMIntegrationSim *mpm,  std::vector<unsigned> &idsDagg, std::vector<unsigned> &idsDall) {
    if (matrices.empty()) {
        matrices.resize(mpm->getParticleAmount());
        matricesInv.resize(mpm->getParticleAmount());
        determinants.resize(mpm->getParticleAmount(), 0);
        densities.resize(mpm->getParticleAmount(), 0);
    }
#pragma omp parallel for schedule(static)
    for (int i = 0; i < mpm->getParticleAmount(); ++i) {
        countMatrix(mpm, i);
    }
#pragma omp parallel for schedule(static)
    for (int i = 0; i < mpm->getParticleAmount(); ++i) {
        densities[i] = countCurrDens(mpm, i);
    }
    for (unsigned i = 0; i < densities.size(); ++i) {
        idsDall.push_back(i);
        if (densities[i] > state.iso)
            idsDagg.push_back(i);
    }
}

void Classification::countMatrix(MPMIntegrationSim *mpm, unsigned id) {
    auto &parts = mpm->getParticles();
    glm::vec4 xi = parts[id];
    glm::vec3 xic(0);
    float wsum = 0.0f;
    float h = mpm->getSupportRadius();
    std::vector<unsigned> neighpos;
    mpm->neighborsByIndex(id, neighpos);

    std::vector<float> wijs;
    wijs.reserve(neighpos.size());
    //Collecting weights
    for (int j = 0; j < neighpos.size(); ++j) { // we have assurance that all particles are inside h of curr part
        glm::vec3 xj = glm::vec3(parts[neighpos[j]]);
        glm::vec3 r = glm::vec3(xi) - xj;
        float l = length(r);
        float ri = 2 * h;
        float q = l / ri;
        float wij = 0.0f;
        if (l < ri)
            wij = pow(1.0f - q, 3.0f);
        wsum += wij;
        xic += wij * xj;
        wijs.push_back(wij);
    }
    if (neighpos.size() < state.aniso_threshold || wsum < 1e-12) {
        glm::mat3 B (1.0f/h);
        float detB = determinant(B);
        matrices[id] = glm::mat4(B);
        matricesInv[id] = inverse(B);
        determinants[id] = detB;
        return;
    }
    xic /= wsum;
    glm::mat3 C(0);
    //Constructing covariance matrix
    for (int j = 0; j < neighpos.size(); ++j) {
        wijs[j] /= wsum; //normalizing elements so their sum is 1
        glm::vec3 d = glm::vec3(parts[neighpos[j]]) - xic;
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
    if ((state.autoScaleS || !state.isAni) && detB > 0.0f) {
        float td = 1.0f / (h*h*h);
        float s = cbrt(td / detB);
        B_i *= s;
        detB *= s;
    }
    Eigen::Matrix3f B_iInv = B_i.inverse();
    matrices[id] = glm::mat4(glm::make_mat3(B_i.data()));
    matricesInv[id] = glm::make_mat3(B_iInv.data());
    determinants[id] = abs(glm::determinant(matrices[id]));
}

float Classification::countCurrDens(MPMIntegrationSim *mpm, unsigned id) {
    auto &parts = mpm->getParticles();
    glm::vec4 xi = parts[id];
    std::vector<unsigned> neighpos;
    mpm->neighborsByIndex(id, neighpos);
    float density = 0.0f;
    for (unsigned neigh : neighpos) {
        glm::vec4 xj = parts[neigh];
        glm::mat3 curr_mat = glm::mat3(matrices[neigh]);
        glm::vec3 l = curr_mat * glm::vec3(xi - xj);
        float l2 = glm::dot(l, l);
        if (l2 > 1.0f) continue;
        density += state.kern * abs(determinant(matrices[neigh])) * pow(1.0f - l2, 3.0f);
    }
    return density;
}

std::vector<glm::mat3> &Classification::getMatricesInv() {
    return matricesInv;
}

void Classification::bindBuffers(int start) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, SSBOmat);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        matrices.size() * sizeof(glm::mat4),
        nullptr,
        GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
        matrices.size() * sizeof(glm::mat4), matrices.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start, SSBOmat);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, SSBOdet);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        determinants.size() * sizeof(float),
        nullptr,
        GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
        determinants.size() * sizeof(float), determinants.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+1, SSBOdet);
}

