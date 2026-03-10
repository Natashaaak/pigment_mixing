#include "AABBc.h"

#include <glm/common.hpp>


AABBc::AABBc(SPHIntegrationSim *spph) {
    fixedAABB(spph);
}


void AABBc::fixedAABB(SPHIntegrationSim *spph) {
    voxelS = spph->getSupportRadius();
    glm::vec3 max(1);
    glm::vec3 min(1);

    // TODO: change based on number of selected colors
    min = {-1.35, -0.2, -0.85};
    max = {1.35, 3.1, 0.85};

    extent = max - min;
    gridStart = glm::floor(min / voxelS) * voxelS;
    cellsX = std::ceil(extent.x / voxelS) + 1;
    cellsY = std::ceil(extent.y / voxelS) + 1;
    cellsZ = std::ceil(extent.z / voxelS) + 1;
}

glm::uint AABBc::getSize() const {
    return cellsX * cellsY * cellsZ;
}
