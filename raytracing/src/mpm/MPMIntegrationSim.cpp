#include "MPMIntegrationSim.h"
#include "../raymarch/BinaryDensityGrid.h"
#include "../raymarch/AABBc.h"
#include <algorithm>
#include <iostream>
#include "mixbox.h"

MPMIntegrationSim::MPMIntegrationSim(){
    sim = new Simulation();
}

MPMIntegrationSim::~MPMIntegrationSim(){
    delete sim;
}

void MPMIntegrationSim::setupScene(){
    ratios.clear();
    std::vector<Eigen::Matrix<float, 7, 1>> initial_pigments;
    
    for (int i = 0; i < g_num_colors; ++i) {
        ratios.push_back(g_ratios[i]);

        Eigen::Matrix<float, 7, 1> p = Eigen::Matrix<float, 7, 1>::Zero();
        mixbox_srgb32f_to_latent(g_colors[i][0], g_colors[i][1], g_colors[i][2], p.data());
        initial_pigments.push_back(p);

        // print the rgb colors and latent pigments for debugging
        std::cout << "Color " << i << ": RGB(" << g_colors[i][0] << ", " << g_colors[i][1] << ", " << g_colors[i][2] << ") -> Latent(";
        for (int c = 0; c < 7; ++c) {
            std::cout << p(c);
            if (c < 6) std::cout << ", ";   
        }
        std::cout << ")" << std::endl;
    }

    float fps = 30;
    sim->initializeBasic("mpm_integration_test");
    sim->setupScene(fps, ratios, initial_pigments);
    sim->prepareSimulation();
    particles.resize(getParticleAmount());
    pigments.resize(getParticleAmount());
    diffusion_factors.resize(getParticleAmount());
}

void MPMIntegrationSim::simStep(){
    sim->step();
}

bool MPMIntegrationSim::isTimeToRender() {
    return sim->frameFinished();
}

bool MPMIntegrationSim::isFinished() const {
    return sim->isFinished();
}

void MPMIntegrationSim::calculateAndPrintFinalColor() const {
    if (sim == nullptr || sim->Np == 0) {
        std::cout << "No particles to sample." << std::endl;
        return;
    }

    Eigen::Matrix<float, 7, 1> average_pigment = Eigen::Matrix<float, 7, 1>::Zero();
    int sampled_particles_count = 0;

    const TV center(0.0, 0.1, 0.0); // Center of an infinitely tall cylinder
    const T radius = 0.2; // Cylinder radius (diameter 0.5)
    const T radius_sq = radius * radius;

    const Particles& sim_particles = sim->particles;

    for (unsigned int i = 0; i < sim->Np; ++i) {
        if (!sim_particles.active[i]) {
            continue;
        }

        const TV& pos = sim_particles.x[i];
        T dist_xz_sq = (pos(0) - center(0)) * (pos(0) - center(0)) + (pos(2) - center(2)) * (pos(2) - center(2));

        if (dist_xz_sq <= radius_sq) {
            average_pigment += sim_particles.pigments[i];
            sampled_particles_count++;
        }
    }

    if (sampled_particles_count > 0) {
        average_pigment /= (float)sampled_particles_count;

        float final_rgb[3];
        mixbox_latent_to_srgb32f(average_pigment.data(), &final_rgb[0], &final_rgb[1], &final_rgb[2]);

        std::cout << "\n--------------------------------------------------" << std::endl;
        std::cout << "Final Mixed Color (from " << sampled_particles_count << " particles sampled):" << std::endl;
        std::cout << "R: " << final_rgb[0] << ", G: " << final_rgb[1] << ", B: " << final_rgb[2] << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;
    } else {
        std::cout << "No particles found in the sampling cylinder." << std::endl;
    }
}

float MPMIntegrationSim::getRadius() {
    return sim->dx * 0.5f;  // set particle radius to be half of grid cell size
}

float MPMIntegrationSim::getSupportRadius() {
    return sim->dx * 2.0f;  // with using SPLINEDEG 2 kernel, the support radius is 2 times the grid cell size
}

std::vector<glm::vec4> &MPMIntegrationSim::recountParticles() {
    Particles &sim_particles = sim->particles;
    for (unsigned int i = 0; i < sim->Np; i++) {
        // store position in .xyz
        if (sim->dim == 3) {
            this->particles[i] = glm::vec4(sim_particles.x[i](0), sim_particles.x[i](1), sim_particles.x[i](2), 1.0f);
        } else {
            // make z coordinate random
            this->particles[i] = glm::vec4(sim_particles.x[i](0), sim_particles.x[i](1), 0.0f, 1.0f);
        }
        for (int c = 0; c < 7; ++c)
            this->pigments[i][c] = sim_particles.pigments[i](c);
        this->diffusion_factors[i] = sim_particles.diffusion_factor[i];
    }

    return this->particles;
}


std::vector<glm::vec4> &MPMIntegrationSim::getParticles() {
    return particles;
}

std::vector<std::array<float, 7>>& MPMIntegrationSim::getPigments() {
    return pigments;
}

std::vector<float>& MPMIntegrationSim::getDiffusionFactors() {
    return diffusion_factors;
}

unsigned MPMIntegrationSim::getParticleAmount() {
    return sim->Np;
}

void MPMIntegrationSim::setGridData(BinaryDensityGrid* bdg, AABBc* a) {
    this->bdg = bdg;
    this->aabb = a;
}

bool MPMIntegrationSim::spatulaExists() const {
    return sim->getSpatulaObject() != nullptr;
}

glm::mat4 MPMIntegrationSim::getSpatulaInvTransform() const
{
    if(!sim->getSpatulaObject())
        return glm::mat4(1.0f);

    auto transform = sim->getSpatulaObject()->invTransform.matrix();
    // convert to glm::mat4
    glm::mat4 glmTransform;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            // Eigen (row, col) -> GLM [col][row]
            glmTransform[col][row] = (float)transform(row, col); 
        }
    }
    return glmTransform;
}

glm::vec3 MPMIntegrationSim::getSpatulaDim() const
{
    if (!sim->getSpatulaObject())
        return glm::vec3(0.0f);
        
    auto spatula = sim->getSpatulaObject();
    return glm::vec3((float)spatula->halfWidth, (float)spatula->halfThickness, (float)spatula->halfLength);
}

float MPMIntegrationSim::getPigmentDEdge0() const
{
    return sim->pigment_D_edge0;
}

float MPMIntegrationSim::getPigmentDEdge1() const
{
    return sim->pigment_D_edge1;
}

void MPMIntegrationSim::neighborsByIndex(unsigned i, std::vector<unsigned int> &out) {
    out.clear();
    // Pre-allocate to prevent reallocation overhead during neighbor search.
    out.reserve(64); 
    float searchRadius = getSupportRadius();
    float searchRadiusSq = searchRadius * searchRadius;
    glm::vec3 posI = glm::vec3(particles[i]);

    if (bdg && aabb) {
        glm::vec3 cell = glm::floor((posI - aabb->gridStart) / aabb->voxelS);
        
        int minX = std::max(0, (int)cell.x - 1);
        int maxX = std::min((int)aabb->cellsX - 1, (int)cell.x + 1);
        int minY = std::max(0, (int)cell.y - 1);
        int maxY = std::min((int)aabb->cellsY - 1, (int)cell.y + 1);
        int minZ = std::max(0, (int)cell.z - 1);
        int maxZ = std::min((int)aabb->cellsZ - 1, (int)cell.z + 1);

        const auto& cells = bdg->getCells();
        const auto& ids = bdg->getIds();

        for (int z = minZ; z <= maxZ; ++z) {
            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    unsigned cellId = x + aabb->cellsX * (z * aabb->cellsY + y);
                    glm::uvec2 cellData = cells[cellId];
                    unsigned count = cellData.x;
                    unsigned offset = cellData.y;
                    
                    for (unsigned c = 0; c < count; ++c) {
                        unsigned j = ids[offset + c];
                        glm::vec3 posJ = glm::vec3(particles[j]);
                        glm::vec3 diff = posI - posJ;
                        if (glm::dot(diff, diff) <= searchRadiusSq) {
                            out.push_back(j);
                        }
                    }
                }
            }
        }
    } else {
        for (unsigned int j = 0; j < particles.size(); ++j) {
            glm::vec3 posJ = glm::vec3(particles[j]);
            glm::vec3 diff = posI - posJ;
            if (glm::dot(diff, diff) <= searchRadiusSq) {
                out.push_back(j);
            }
        }
    }
}

std::pair<glm::vec3, glm::vec3> MPMIntegrationSim::getGridBoundaries() {
    auto boundaries = sim->getGridBoundaries();
    return std::make_pair(
        glm::vec3(boundaries.first[0], 
                  boundaries.first[1], 
                  boundaries.first[2]), 
        glm::vec3(boundaries.second[0], 
                  boundaries.second[1], 
                  boundaries.second[2]));
}
