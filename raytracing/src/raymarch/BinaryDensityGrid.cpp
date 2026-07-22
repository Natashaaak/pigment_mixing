#include "BinaryDensityGrid.h"
#include <glm/common.hpp>

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
    size_t pigX = a->cellsX * 4;
    size_t pigY = a->cellsY * 4;
    size_t pigZ = a->cellsZ * 4;
    gridPigments.resize(pigX * pigY * pigZ * 8, 0); // 8 components for each voxel
}

void BinaryDensityGrid::fillCells(MPMIntegrationSim *mpm, AABBc *a) {
    const auto &spheresData = mpm->getParticles();
    std::vector<glm::uvec2>(cells.size(), {0, 0}).swap(cells); // set to zero
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mpm->getParticleAmount(); ++i) { // count particles in each cell
        glm::vec3 cell = glm::floor((glm::vec3(spheresData[i]) - a->gridStart) / a->voxelS);

        if (cell.x < 0 || cell.y < 0 || cell.z < 0 || cell.x >= a->cellsX || cell.y >= a->cellsY || cell.z >= a->cellsZ)
            continue; // check boundaries

        unsigned id = cell.x + a->cellsX * (cell.z * a->cellsY + cell.y); // find corresponding cell
        #pragma omp atomic
        cells[id].x++;
    }
    // count offsets
    for (int i = 1; i < cells.size(); ++i) {
        cells[i].y += cells[i-1].x + cells[i-1].y;
    }
    std::vector<unsigned> appeared(cells.size(), 0);
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < mpm->getParticleAmount(); ++i) {
        glm::vec3 cell = glm::floor((glm::vec3(spheresData[i]) - a->gridStart) / a->voxelS);
        if (cell.x < 0 || cell.y < 0 || cell.z < 0 || cell.x >= a->cellsX || cell.y >= a->cellsY || cell.z >= a->cellsZ)
            continue; // check boundaries
        unsigned id = cell.x + a->cellsX * (cell.z * a->cellsY + cell.y); // find corresponding cell
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

static float mixboxDistance(const std::array<float, 7>& a, const std::array<float, 7>& b) {
    float sum = 0.0f;
    for (int c = 0; c < 4; ++c) {
        float diff = a[c] - b[c];
        sum += diff * diff;
    }
    return std::sqrt(sum); // Euclidean norm of distance 7D vectors
}

void BinaryDensityGrid::fillRenderGrid(MPMIntegrationSim *mpm, AABBc *a, const std::vector<glm::mat4>& ani_matrices) {
    // 1. Clear the grid (8 floats per voxel: 7 for latent, 1 for weight)
    std::fill(gridPigments.begin(), gridPigments.end(), 0.0f);

    const auto& particles_pos = mpm->getParticles(); // spheresData
    const auto& particles_pig = mpm->getPigments(); // PigmentsBuffer

    float voxelS = a->voxelS / 4.0f;
    size_t pigX = a->cellsX * 4;
    size_t pigY = a->cellsY * 4;
    size_t pigZ = a->cellsZ * 4;

    float sigma_color = state.sigma_color; // Sensitivity threshold (tunable parameter)
    
    // FIX 1: Do not multiply by voxelS! The anisotropic matrix G inherently scales the physical 
    // difference into a normalized space where 1.0 is the boundary.
    float sigma_spatial = std::max(state.sigma_spatial, 0.001f);
    sigma_color = std::max(sigma_color, 0.001f);

    // FIX 2: Calculate a much wider search extent. G scales by ~ 1/h. 
    // As a safe heuristic to cover stretched anisotropic ellipsoids, we search up to 3x the support radius.
    float max_search_dist = mpm->getSupportRadius() * 3.0f * sigma_spatial;
    int ext = std::min(12, (int)std::ceil(max_search_dist / voxelS));

    // Temporary grid for Pass 1 (spatial average to find C_voxel)
    std::vector<float> tempGrid(gridPigments.size(), 0.0f);

    // Pass 1: Compute the spatial average color (C_voxel) for each cell
    for (int p = 0; p < mpm->getParticleAmount(); ++p) {
        glm::vec3 pos = glm::vec3(particles_pos[p]);
        glm::ivec3 base_cell = glm::ivec3(glm::floor((pos - a->gridStart) / voxelS - 0.5f));
        glm::mat3 G = glm::mat3(ani_matrices[p]);

        for (int dz = -ext; dz <= ext; ++dz) {
            for (int dy = -ext; dy <= ext; ++dy) {
                for (int dx = -ext; dx <= ext; ++dx) {
                    glm::ivec3 cell = base_cell + glm::ivec3(dx, dy, dz);

                    // boundary check
                    if (cell.x < 0 || cell.y < 0 || cell.z < 0 || 
                        cell.x >= (int)pigX || cell.y >= (int)pigY || cell.z >= (int)pigZ) continue;

                    size_t idx = (cell.z * pigY * pigX + cell.y * pigX + cell.x) * 8;
                    
                    // Calculate Spatial Weight based on distance to voxel center
                    glm::vec3 voxelCenter = a->gridStart + (glm::vec3(cell) + 0.5f) * voxelS;
                    glm::vec3 diff = pos - voxelCenter;
                    
                    // Apply anisotropy
                    glm::vec3 l = G * diff;
                    float spatialDistSq = glm::dot(l, l);
                    
                    // Optimization: early exit if distance is safely far beyond sigma standard deviation
                    if (spatialDistSq > 9.0f * sigma_spatial * sigma_spatial) continue;

                    float w_spatial = std::exp(-spatialDistSq / (sigma_spatial * sigma_spatial));

                    for (int c = 0; c < 7; ++c) {
                        tempGrid[idx + c] += particles_pig[p][c] * w_spatial;
                    }
                    tempGrid[idx + 7] += w_spatial;
                }
            }
        }
    }

    // Normalize Pass 1 to get final C_voxel
    for (size_t i = 0; i < tempGrid.size(); i += 8) {
        float w = tempGrid[i + 7];
        if (w > 0.0f) {
            for (int c = 0; c < 7; ++c) tempGrid[i + c] /= w;

            // Normalize pigments for robustness
            float pigment_sum = tempGrid[i] + tempGrid[i+1] + tempGrid[i+2] + tempGrid[i+3];
            // Since we are working only with opaque pigments, their sum should always be 1.
            if (pigment_sum > 1e-6f) {
                for (int c = 0; c < 4; ++c) tempGrid[i + c] /= pigment_sum;
            }
        }
    }

    // Pass 2: Calculate Bilateral weights and accumulate final voxel colors
    for (int p = 0; p < mpm->getParticleAmount(); ++p) {
        glm::vec3 pos = glm::vec3(particles_pos[p]);
        glm::ivec3 base_cell = glm::ivec3(glm::floor((pos - a->gridStart) / voxelS - 0.5f));
        glm::mat3 G = glm::mat3(ani_matrices[p]);

        for (int dz = -ext; dz <= ext; ++dz) {
            for (int dy = -ext; dy <= ext; ++dy) {
                for (int dx = -ext; dx <= ext; ++dx) {
                    glm::ivec3 cell = base_cell + glm::ivec3(dx, dy, dz);

                    if (cell.x < 0 || cell.y < 0 || cell.z < 0 || 
                        cell.x >= (int)pigX || cell.y >= (int)pigY || cell.z >= (int)pigZ) continue;

                    size_t idx = (cell.z * pigY * pigX + cell.y * pigX + cell.x) * 8;

                    std::array<float, 7> p_pig = particles_pig[p];
                    std::array<float, 7> v_pig;
                    for (int c = 0; c < 7; ++c) v_pig[c] = tempGrid[idx + c]; // Access C_voxel from Pass 1
                    
                    // Get the current color in the voxel for comparison (if any already exists)
                    glm::vec3 voxelCenter = a->gridStart + (glm::vec3(cell) + 0.5f) * voxelS;
                    glm::vec3 diff = pos - voxelCenter;
                    
                    // Apply anisotropy
                    glm::vec3 l = G * diff;
                    float spatialDistSq = glm::dot(l, l);
                    
                    // Optimization: early exit if distance is safely far beyond sigma standard deviation
                    if (spatialDistSq > 9.0f * sigma_spatial * sigma_spatial) continue;

                    float w_spatial = std::exp(-spatialDistSq / (sigma_spatial * sigma_spatial));

                    // Calculate W_total = W_spatial * exp(-mixbox_dist^2 / sigma^2)
                    float colorDist = mixboxDistance(p_pig, v_pig);
                    float w_total = w_spatial * std::exp(-(colorDist * colorDist) / (sigma_color * sigma_color));

                    // Accumulation
                    for (int c = 0; c < 7; ++c) {
                        gridPigments[idx + c] += p_pig[c] * w_total;
                    }
                    gridPigments[idx + 7] += w_total;
                }
            }
        }
    }

    // Normalization: Divide colors by the sum of weights
    for (size_t i = 0; i < gridPigments.size(); i += 8) {
        float w = gridPigments[i + 7];
        if (w > 0.0f) {
            for (int c = 0; c < 7; ++c) gridPigments[i + c] /= w;

            // Final normalization of pigments before sending to the shader
            float pigment_sum = gridPigments[i] + gridPigments[i+1] + gridPigments[i+2] + gridPigments[i+3];
            // Since we are working only with opaque pigments, their sum should always be 1.
            if (pigment_sum > 1e-6f) {
                for (int c = 0; c < 4; ++c) gridPigments[i + c] /= pigment_sum;
            }
        }
    }
}

void BinaryDensityGrid::bindBuffers(unsigned start) {
    // MSSBO: 0
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, MSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    M.size() * sizeof(unsigned),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    M.size() * sizeof(unsigned),
    M.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start, MSSBO);

    // cellsSSBO: 1
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, cellsSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    cells.size() * sizeof(glm::uvec2),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    cells.size() * sizeof(glm::uvec2),
    cells.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+1, cellsSSBO);

    // idsSSBO: 2
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, idsSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    ids.size() * sizeof(unsigned),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    ids.size() * sizeof(unsigned),
    ids.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+2, idsSSBO);

    // M2SSBO: 3
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, M2SSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    M2.size() * sizeof(unsigned),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    M2.size() * sizeof(unsigned),
    M2.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+3, M2SSBO);

    // M4SSBO: 4
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, M4SSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
    M4.size() * sizeof(unsigned),
    nullptr,
    GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
    M4.size() * sizeof(unsigned),
    M4.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, start+4, M4SSBO);

    // pigmentsSSBO: 5
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, pigmentsSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, gridPigments.size() * sizeof(float), gridPigments.data(), GL_STREAM_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 10, pigmentsSSBO);
}

void BinaryDensityGrid::genBuffers() {
    glGenBuffers(1, &MSSBO);
    glGenBuffers(1, &M2SSBO);
    glGenBuffers(1, &M4SSBO);
    glGenBuffers(1, &cellsSSBO);
    glGenBuffers(1, &idsSSBO);
    glGenBuffers(1, &pigmentsSSBO);
}

BinaryDensityGrid::~BinaryDensityGrid() {
    glDeleteBuffers(1, &MSSBO);
    glDeleteBuffers(1, &M2SSBO);
    glDeleteBuffers(1, &M4SSBO);
    glDeleteBuffers(1, &cellsSSBO);
    glDeleteBuffers(1, &idsSSBO);
    glDeleteBuffers(1, &pigmentsSSBO);
}
