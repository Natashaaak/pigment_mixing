//----------------------------------------------------------------------------------------
/**
 * \file       MarchingCubes.h
 * \brief      Marching cubes pipeline
 *
 *  Full marching cubes pipeline, including density computations and anisotropy matrices.
 *
 */
//----------------------------------------------------------------------------------------

#ifndef MARCHINGCUBES_H
#define MARCHINGCUBES_H

#include "Camera.h"
#include "Shader.h"
#include "sph/SPHIntegrationSim.h"
#include "state.h"
#include "raymarch/AABBc.h"

struct aniso {
    glm::vec4 col1;
    glm::vec4 col2;
    glm::vec4 col3;
    glm::vec4 bDet;
    glm::vec4 sphereData;
    aniso() = default;
    aniso(glm::vec4 col1, glm::vec4 col2, glm::vec4 col3, glm::vec4 bDet, glm::vec4 sphereData) :
        col1(col1), col2(col2), col3(col3), bDet(bDet), sphereData(sphereData) {}
};


class MarchingCubes {
public:
    MarchingCubes();
    ~MarchingCubes();

    /**
     * Recounts boundaries for the current state of the simulation based on the particles
     * @param spheresData link to spheres vector
     */
    void countAABB(const std::vector<glm::vec4> &spheresData);

    /**
     * Creates 3D texture for density compuatation
     */
    void create3DTexture();

    /**
     * Computes density for the whole grid
     * @param spheresData link to spheres vector
     * @param numSpheres amount of spheres
     * @param spph simulation object
     */
    void computeDensity(const std::vector<glm::vec4> &spheresData, int numSpheres, SPHIntegrationSim *spph);

    /**
     * Performs marching cubes algorithm, generating triangles for the whole grid.
     * @param camera Camera object
     */
    void march(Camera *camera);

private:
    /**
     * Creates ssbos and textures
     */
    void createBind();

    /**
     * Computes the anisotropic matrix for particles that considered to be at aggregations.
     * @param spheresData link to spheres vector
     * @param numSpheres amount of spheres
     * @param spph simulation object
     */
    void anisotropicKernelCount(const std::vector<glm::vec4> &spheresData, int numSpheres, SPHIntegrationSim *spph);

    /**
     * Renders the final result
     * @param camera Camera object
     */
    void drawResult(Camera *camera);

    /**
     * Creates shader objects for density, marching cubes and final surface rendering shaders.
     */
    void initShader();

    float voxelS = 0;
    glm::vec3 gridStart = glm::vec3(0);
    uint cellsX = 0, cellsY = 0, cellsZ = 0;
    GLuint tex = 0, spheresSSBO = 0, meshSSBO, counterBuf, VAO;
    Shader *densityShader, *marchShader, *tr;
    std::vector<aniso> matrices;
};



#endif //MARCHINGCUBES_H
