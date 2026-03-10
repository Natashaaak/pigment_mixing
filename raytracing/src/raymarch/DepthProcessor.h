//----------------------------------------------------------------------------------------
/**
 * \file       DepthProcessor.h
 * \brief      Processes all the pipeline steps connected to depth.
 *
 *  Processes Dall, Dagg, DallSmooth, Nscreen, DepthVar pipeline steps.
 *
 */
//----------------------------------------------------------------------------------------

#ifndef DEPTHMAPS_H
#define DEPTHMAPS_H
#include "Camera.h"
#include "Classification.h"
#include "Shader.h"
#include "sph/SPHIntegrationSim.h"


class DepthProcessor {
public:
    ~DepthProcessor();
    DepthProcessor();

    /**
     * Generates all the depth maps and calls for other pipeline steps connected to depth.
     * @param spph simulation
     * @param ww framebuffer width
     * @param wh framebuffer height
     * @param camera camera object
     */
    void generateDepthMaps(SPHIntegrationSim *spph, GLint ww, GLint wh, Camera *camera);

    /**
     * Binds depth maps, screen space normals and depth variance to shader
     * @param start starting position
     * @param sh shader to bind to
     */
    void bindDepthMaps(int start, Shader *sh) const;

    /**
     * Binds Dall and depthVar to interpolation shader
     * @param sh shader
     */
    void bindDall(Shader *sh);
private:
    void genBuffers();
    void initShader();
    void buffers(SPHIntegrationSim *spph);
    void genFbo();
    void resizeTextures(GLint ww, GLint wh);

    /**
     * Smoothes dall by calling Gaussian Bilateral Filter shader.
     * @param spph simulation
     * @param ww framebuffer width
     * @param wh framebuffer height
     * @param camera camera object
     */
    void smoothDall(SPHIntegrationSim *spph, GLint ww, GLint wh, Camera *camera);

    /**
     * Computes screen spaces normals using DallSmooth
     * @param ww framebuffer width
     * @param wh framebuffer height
     * @param camera camera object
     */
    void computeNScreen(GLint ww, GLint wh, Camera *camera);

    /**
     * Computes depth variance for tiles based on Dall depth map
     * @param ww framebuffer width
     * @param wh framebuffer height
     */
    void computeDepthVar(GLint ww, GLint wh);

    GLuint Dagg = 0, Dall = 0, fboDagg = 0, fboDall = 0, VBO = 0, EBODagg = 0, EBODall = 0,
    VAODagg = 0, VAODall = 0, rdepth = 0, DallSmooth = 0, DallTMP = 0, Nscreen = 0, Variance = 0, VBOMat = 0;
    Shader *shader, *gaussBilateral, *compN, *depthVarShader;

    std::vector<unsigned> idsDagg;
    std::vector<unsigned> idsDall;
    Classification *c;
};



#endif //DEPTHMAPS_H
