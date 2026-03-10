#include "AABBc.h"

#include <glm/common.hpp>


AABBc::AABBc(SPHIntegrationSim *spph, int scene) {
    fixedAABB(spph, scene);
}


void AABBc::fixedAABB(SPHIntegrationSim *spph, int scene) {
    voxelS = spph->getSupportRadius();
    glm::vec3 max(1);
    glm::vec3 min(1);
    switch (scene) {
        case 1:
            min = {-1.35, -0.2, -0.85};
            max = {1.35, 3.1, 0.85};
            break;
        case 2:
            min = {-1.65, -0.2, -1.65};
            max = {1.65, 3.2, 1.65};
            break;
        case 3:
            min = {-1.1, -0.2, -1.1};
            max = {1.1, 2.1, 1.1};
            break;
        default:
            break;
    }
    extent = max - min;
    gridStart = glm::floor(min / voxelS) * voxelS;
    cellsX = std::ceil(extent.x / voxelS) + 1;
    cellsY = std::ceil(extent.y / voxelS) + 1;
    cellsZ = std::ceil(extent.z / voxelS) + 1;
}

glm::uint AABBc::getSize() const {
    return cellsX * cellsY * cellsZ;
}
