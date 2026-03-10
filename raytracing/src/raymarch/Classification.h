//----------------------------------------------------------------------------------------
/**
 * \file       Classification.h
 * \brief      Classifies particles
 *
 *  Classifies particles based on the anisotropy kernels and computed density. Stores anisotropy matrices and binds them
 *  to shaders
 */
//----------------------------------------------------------------------------------------

#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H
#include "sph/SPHIntegrationSim.h"
#include "state.h"

class Classification {
public:
    /**
     * Computes densities for particles in order to classify them
     * @param spph simulation
     * @param idsDagg vector that will store ids of particles classified as aggregations.
     * @param idsDall vector that will store ids of all particles.
     */
    void countDensities(SPHIntegrationSim *spph, std::vector<unsigned> &idsDagg, std::vector<unsigned> &idsDall);
    ~Classification();
    Classification();

    /**
     * Returns vector of inverted anisotropy matrices for particles for Depth map generation
     * @return std::vector<glm::mat3>& matricesInv
     */
    std::vector<glm::mat3>& getMatricesInv();
    void bindBuffers(int start);
private:
    /**
     * Computes anisotropy matrix for the needed particle
     * @param spph simulation
     * @param id particle id
     */
    void countMatrix(SPHIntegrationSim *spph, unsigned id);

    /**
     * Computes density value for the current particle
     * @param spph simulation
     * @param id particle id
     * @return float density
     */
    float countCurrDens(SPHIntegrationSim *spph, unsigned id);
    std::vector<glm::mat4> matrices;
    std::vector<glm::mat3> matricesInv;
    std::vector<float> determinants;
    std::vector<float> densities;
    GLuint SSBOmat = 0, SSBOdet = 0;
};



#endif //CLASSIFICATION_H
