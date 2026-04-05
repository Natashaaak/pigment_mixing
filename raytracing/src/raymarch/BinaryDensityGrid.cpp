#include "BinaryDensityGrid.h"

BinaryDensityGrid::BinaryDensityGrid(AABBc *a, MPMIntegrationSim *mpm) {
    createVectors(a, mpm);
    genBuffers();
}


void BinaryDensityGrid::createVectors(AABBc *a, MPMIntegrationSim *mpm) {
    cells.resize(a->getSize(), {0, 0});
    ids.resize(mpm->getParticleAmount());
    M.resize(a->getSize(), 0);
    M2.resize((a->cellsX+1)/2 * (a->cellsY+1)/2 * (a->cellsZ+1)/2, 0);
    M4.resize((a->cellsX+4-1)/4 * (a->cellsY+4-1)/4 * (a->cellsZ+4-1)/4, 0);
}

void BinaryDensityGrid::fillCells(MPMIntegrationSim *mpm, AABBc *a) {
    const auto &spheresData = mpm->getParticles();
    std::vector<glm::uvec2>(cells.size(), {0, 0}).swap(cells); //setting to zero
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mpm->getParticleAmount(); ++i) { //counting particles in each cell
        glm::vec3 cell = glm::floor((glm::vec3(spheresData[i]) - a->gridStart) / a->voxelS);

        if (cell.x < 0 || cell.y < 0 || cell.z < 0 || cell.x >= a->cellsX || cell.y >= a->cellsY || cell.z >= a->cellsZ)
            continue; //checking boundaries

        unsigned id = cell.x + a->cellsX * (cell.z * a->cellsY + cell.y); //finding according cell
        #pragma omp atomic
        cells[id].x++;
    }
    //counting offsets
    for (int i = 1; i < cells.size(); ++i) {
        cells[i].y += cells[i-1].x + cells[i-1].y;
    }
    std::vector<unsigned> appeared(cells.size(), 0);
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mpm->getParticleAmount(); ++i) {
        glm::vec3 cell = glm::floor((glm::vec3(spheresData[i]) - a->gridStart) / a->voxelS);
        if (cell.x < 0 || cell.y < 0 || cell.z < 0 || cell.x >= a->cellsX || cell.y >= a->cellsY || cell.z >= a->cellsZ)
            continue; //checking boundaries
        unsigned id = cell.x + a->cellsX * (cell.z * a->cellsY + cell.y); //finding according cell
        unsigned local_appeared;
        #pragma omp atomic capture
        {
            local_appeared = appeared[id];
            appeared[id]++;
        }
        unsigned index = cells[id].y + local_appeared;
        ids[index] = i;
    }
}

void BinaryDensityGrid::generateNc(AABBc *a, std::vector<float> &nc) {
    //paper 4.1 equations 4 and 5
    std::vector<float> nx(cells.size());
    std::vector<float> ny(cells.size());
    //x
    for (int x = 0; x < a->cellsX; ++x) {
        for (int y = 0; y < a->cellsY; ++y) {
            for (int z = 0; z < a->cellsZ; ++z) {
                float sum = 0.0f;
                for (int dx = -1; dx < 2; ++dx) {
                    unsigned cov_x = std::clamp(x + dx, 0, static_cast<int>(a->cellsX - 1));
                    float k = dx == 0 ? state.params.x : state.params.y;
                    sum += k * cells[cov_x + a->cellsX * (y + z * a->cellsY)].x;
                }
                nx[x + a->cellsX * (y + z * a->cellsY)] = sum;
            }
        }
    }
    //y
    for (int x = 0; x < a->cellsX; ++x) {
        for (int y = 0; y < a->cellsY; ++y) {
            for (int z = 0; z < a->cellsZ; ++z) {
                float sum = 0.0f;
                for (int dy = -1; dy < 2; ++dy) {
                    unsigned cov_y = std::clamp(y + dy, 0, static_cast<int>(a->cellsY - 1));
                    float k = dy == 0 ? state.params.x : state.params.y;
                    sum += k * nx[x + a->cellsX * (cov_y + z * a->cellsY)];
                }
                ny[x + a->cellsX * (y + z * a->cellsY)] = sum;
            }
        }
    }
    //nc
    for (int x = 0; x < a->cellsX; ++x) {
        for (int y = 0; y < a->cellsY; ++y) {
            for (int z = 0; z < a->cellsZ; ++z) {
                float sum = 0.0f;
                for (int dz = -1; dz < 2; ++dz) {
                    unsigned cov_z = std::clamp(z + dz, 0, static_cast<int>(a->cellsZ - 1));
                    float k = dz == 0 ? state.params.x : state.params.y;
                    sum += k * ny[x + a->cellsX * (y + cov_z * a->cellsY)];
                }
                nc[x + a->cellsX * (y + z * a->cellsY)] = sum;
            }
        }
    }

}

void BinaryDensityGrid::fillBDG(AABBc *a, MPMIntegrationSim *mpm) {
    float kern = 315.0f/(64.0f * M_PI * pow(mpm->getSupportRadius(), 3));
    fillCells(mpm, a);
    std::vector<float> nc(cells.size());
    generateNc(a, nc);
    state.count[0] = 0; state.count[1] = 0;
    uint cellsX2 = (a->cellsX+1)/2;
    uint cellsY2 = (a->cellsY+1)/2;
    uint cellsX4 = (a->cellsX+4 - 1)/4;
    uint cellsY4 = (a->cellsY+4 - 1)/4;
    std::ranges::fill(M2, 0u);
    std::ranges::fill(M4, 0u);
    std::ranges::fill(M, 0u);
    //paper 4.1 equuation 1a nd 3, no mass because mass is the same for every particle.
    for (int i = 0; i < M.size(); ++i) {
        int num = (nc[i] * kern >= state.iso || state.testAllFilled) ? 1 : 0;
        M[i] = num;
        state.count[num]++;

        uint currX = i % a->cellsX;
        uint currY = i / a->cellsX % a->cellsY;
        uint currZ = i / (a->cellsX * a->cellsY);

        uint x2 = currX / 2;
        uint y2 = currY / 2;
        uint z2 = currZ / 2;

        uint x4 = currX / 4;
        uint y4 = currY / 4;
        uint z4 = currZ / 4;
        if (x2 + cellsX2 * (y2 + z2 * cellsY2) >= M2.size() || x4 + cellsX4 * (y4 + z4 * cellsY4) >= M4.size())
            spdlog::critical("Id out of bounds id: {} u got {}, M2: {}", i, x2 + cellsX2 * (y2 + z2 * cellsY2), M2.size());
        M2[x2 + cellsX2 * (y2 + z2 * cellsY2)] |= num;
        M4[x4 + cellsX4 * (y4 + z4 * cellsY4)] |= num;
    }
}

void BinaryDensityGrid::bindBuffers(unsigned start) {
    //MSSBO: 0
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, MSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    M.size() * sizeof(unsigned),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    M.size() * sizeof(unsigned),
    M.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start, MSSBO);

    //cellsSSBO: 1
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, cellsSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    cells.size() * sizeof(glm::uvec2),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    cells.size() * sizeof(glm::uvec2),
    cells.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+1, cellsSSBO);

    //idsSSBO: 2
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, idsSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    ids.size() * sizeof(unsigned),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    ids.size() * sizeof(unsigned),
    ids.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+2, idsSSBO);

    //M2SSBO: 3
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, M2SSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    M2.size() * sizeof(unsigned),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    M2.size() * sizeof(unsigned),
    M2.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+3, M2SSBO);

    //M4SSBO: 4
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, M4SSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    M4.size() * sizeof(unsigned),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    M4.size() * sizeof(unsigned),
    M4.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+4, M4SSBO);
}

void BinaryDensityGrid::genBuffers() {
    glGenBuffers(1, &MSSBO);
    glGenBuffers(1, &M2SSBO);
    glGenBuffers(1, &M4SSBO);
    glGenBuffers(1, &cellsSSBO);
    glGenBuffers(1, &idsSSBO);
}

BinaryDensityGrid::~BinaryDensityGrid() {
    glDeleteBuffers(1, &MSSBO);
    glDeleteBuffers(1, &M2SSBO);
    glDeleteBuffers(1, &M4SSBO);
    glDeleteBuffers(1, &cellsSSBO);
    glDeleteBuffers(1, &idsSSBO);
}
