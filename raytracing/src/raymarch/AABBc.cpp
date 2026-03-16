#include "AABBc.h"

#include <glm/common.hpp>


AABBc::AABBc(MPMIntegrationSim *mpm) {
    fixedAABB(mpm);
}


void AABBc::fixedAABB(MPMIntegrationSim *mpm) {
    voxelS = mpm->getSupportRadius();
    glm::vec3 max(1);
    glm::vec3 min(1);

    auto boundaries = mpm->getGridBoundaries();
    min = glm::vec3(boundaries.first);
    max = glm::vec3(boundaries.second);

    extent = max - min;
    gridStart = glm::floor(min / voxelS) * voxelS;
    cellsX = std::ceil(extent.x / voxelS) + 1;
    cellsY = std::ceil(extent.y / voxelS) + 1;
    cellsZ = std::ceil(extent.z / voxelS) + 1;
}

glm::uint AABBc::getSize() const {
    return cellsX * cellsY * cellsZ;
}
