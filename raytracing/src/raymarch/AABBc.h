//----------------------------------------------------------------------------------------
/**
 * \file       AABBc.h
 * \brief      Stores aabb bounding box
 *
 *  Stores and chooses aabb bounding box and size of the grid, its start and voxel size
 *
 */
//----------------------------------------------------------------------------------------

#ifndef AABBC_H
#define AABBC_H
#include <glm/vec3.hpp>

#include "sph/SPHIntegrationSim.h"


class AABBc {
public:
    /**
     * Creates boundary box object of the current scene, with computing voxels amount and size, needed for BDG
     * @param spph simulation
     * @param scene scene number from 3 predifined scenes
     */
    AABBc(SPHIntegrationSim *spph);
    float voxelS = 0;
    glm::vec3 gridStart = glm::vec3(0);
    glm::uint cellsX = 0, cellsY = 0, cellsZ = 0;
    glm::vec3 extent;

    /**
     * returns cellsX * cellsY * cellsZ;
     * @return
     */
    [[nodiscard]] glm::uint getSize() const;
private:
    /**
     * Chooses what boundary to use depending on the scene and computes all the parameters
     * @param spph simulation
     */
    void fixedAABB(SPHIntegrationSim *spph);
};



#endif //AABBC_H
