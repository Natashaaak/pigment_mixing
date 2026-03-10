//----------------------------------------------------------------------------------------
/**
 * \file       BinaryDensityGrid.h
 * \brief      Computes and binds binary density grid
 *
 *  Computes binary density grid and all ssbos needed to access it fast in the shader
 *
 */
//----------------------------------------------------------------------------------------

#ifndef BINARYDENSITYGRID_H
#define BINARYDENSITYGRID_H


#include "AABBc.h"
#include "sph/SPHIntegrationSim.h"
#include "state.h"

class BinaryDensityGrid {
public:
    /**
     * Fills in the binary density grid, including all the texels called M2 and M4
     * @param a boundaries of the simulation scene
     * @param spph simulation
     */
    void fillBDG(AABBc *a, SPHIntegrationSim *spph);

    /**
     * Binds all SSBOs from the starting point in uniforms
     * @param start starting index in shader for all ssbos in this class
     */
    void bindBuffers(unsigned start = 0);
    BinaryDensityGrid(AABBc *a, SPHIntegrationSim *spph);
    ~BinaryDensityGrid();

private:
    /**
     * Creates and resize all vectors for the data needed
     * @param a boundary
     * @param spph simulation
     */
    void createVectors(AABBc *a, SPHIntegrationSim *spph);

    /**
     * Fills cells with particles ids that are in these cells, and all the supporting vectors
     * @param spph simulation
     * @param a boundary
     */
    void fillCells(SPHIntegrationSim *spph, AABBc *a);
    std::vector<glm::uvec2> cells; //x is count, y is offset
    std::vector<unsigned> ids;
    std::vector<unsigned> M;
    std::vector<unsigned> M2;
    std::vector<unsigned> M4;

    /**
     * Estimates the density value for every cell in M in three consequent steps
     * @param a
     * @param nc
     */
    void generateNc(AABBc *a, std::vector<float> &nc);
    void genBuffers();

    GLuint cellsSSBO = 0, idsSSBO = 0, MSSBO = 0, M2SSBO = 0, M4SSBO;
};



#endif //BINARYDENSITYGRID_H
