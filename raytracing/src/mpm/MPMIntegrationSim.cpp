#include "MPMIntegrationSim.h"
#include "../raymarch/BinaryDensityGrid.h"
#include "../raymarch/AABBc.h"
#include <algorithm>

MPMIntegrationSim::MPMIntegrationSim(){
    sim = new Simulation();
}

MPMIntegrationSim::~MPMIntegrationSim(){
    delete sim;
}

void MPMIntegrationSim::setupScene(){
    // temp setting colors
    colors.push_back(glm::vec3(1.0f, 1.0f, 0.0f));
    colors.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
    colors.push_back(glm::vec3(0.0f, 1.0f, 0.0f));

    ratios.push_back(1.0f/3.0f);
    ratios.push_back(2.0f/3.0f);

    float fps = 60;
    sim->initializeBasic("mpm_integration_test");
    sim->setupScene(fps, ratios);
    sim->prepareSimulation();
    particles.resize(getParticleAmount());
}

void MPMIntegrationSim::simStep(){
    sim->step();
}

bool MPMIntegrationSim::isTimeToRender() {
    return sim->frameFinished();
}

float MPMIntegrationSim::getRadius() {
    return sim->dx * 0.5f;  // set particle radius to be half of grid cell size
}

float MPMIntegrationSim::getSupportRadius() {
    return sim->dx * 1.5f;  // with using SPLINEDEG 2 kernel, the support radius is 1.5 times the grid cell size
}

std::vector<glm::vec4> &MPMIntegrationSim::recountParticles() {
    Particles &particles = sim->particles;
    for (unsigned int i = 0; i < sim->Np; i++) {
        // store position in .xyz and color index in .w
        if (sim->dim == 3) {
            this->particles[i] = glm::vec4(particles.x[i](0), particles.x[i](1), particles.x[i](2), static_cast<float>(particles.color[i]));
        } else {
            // make z coordinate random
            this->particles[i] = glm::vec4(particles.x[i](0), particles.x[i](1), 0.0f, static_cast<float>(particles.color[i]));
        }
    }

    return this->particles;
}


const std::vector<glm::vec4> &MPMIntegrationSim::getParticles() {
    return particles;
}

const std::vector<glm::vec3> &MPMIntegrationSim::getColors() {
    return colors;
}

unsigned MPMIntegrationSim::getParticleAmount() {
    return sim->Np;
}

void MPMIntegrationSim::setGridData(BinaryDensityGrid* bdg, AABBc* a) {
    this->bdg = bdg;
    this->aabb = a;
}

std::vector<float> MPMIntegrationSim::getSpatulaBuffer() const
{
    if (!sim->getVdbObject())
        return {};

    return sim->getVdbObject()->buffer;
}

glm::vec3 MPMIntegrationSim::getSpatulaDimensions() const
{
    if (!sim->getVdbObject())
        return glm::vec3(0.0f);

    auto dim = sim->getVdbObject()->dim;
    return glm::vec3(dim.x(), dim.y(), dim.z());
}

glm::mat4 MPMIntegrationSim::getSpatulaTransform() const
{
    if(!sim->getVdbObject())
        return glm::mat4(1.0f);

    auto vdbObj = sim->getVdbObject();
    auto offset = vdbObj->offset;
    float scale = vdbObj->scale;
    
    // Coordinates of the spatula bounding box
    openvdb::CoordBBox bbox = vdbObj->grid->evalActiveVoxelBoundingBox();
    openvdb::Vec3d minIndexSpace(bbox.min().x(), bbox.min().y(), bbox.min().z());
    
    // transfer coordinates into world space
    openvdb::Vec3d minWorld = vdbObj->grid->indexToWorld(minIndexSpace);

    glm::mat4 transform(1.0f);
    
    #ifdef THREEDIM
    transform = glm::translate(transform, glm::vec3(offset.x(), offset.y(), offset.z()));
    #else
    transform = glm::translate(transform, glm::vec3(offset.x(), offset.y(), 0.0f));
    #endif
    transform = glm::scale(transform, glm::vec3(scale));
    transform = glm::translate(transform, glm::vec3(minWorld.x(), minWorld.y(), minWorld.z()));
    
    float vdbVoxelSize = vdbObj->grid->voxelSize()[0];
    transform = glm::scale(transform, glm::vec3(vdbVoxelSize));
    return transform;
}

void MPMIntegrationSim::moveSpatulaY(T deltaY)
{
    if (!sim->getVdbObject())
        return;

    auto vdbObj = sim->getVdbObject();
    vdbObj->offset.y() += deltaY;
}

void MPMIntegrationSim::neighborsByIndex(unsigned i, std::vector<unsigned int> &out) {
    out.clear();
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
