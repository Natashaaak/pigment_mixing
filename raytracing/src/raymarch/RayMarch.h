//----------------------------------------------------------------------------------------
/**
 * \file       RayMarch.h
 * \brief      Ray marching pipeline class
 *
 *  Calls all other classes connected to ray marching pipeline and dispatches ray marching shader
 *
 */
//----------------------------------------------------------------------------------------

#ifndef RAYMARCH_H
#define RAYMARCH_H
#include "AABBc.h"
#include "BinaryDensityGrid.h"
#include "DepthProcessor.h"
#include "SpatulaMesh.h"
#include <string>
#include <vector>

class RayMarch {
public:
    RayMarch(MPMIntegrationSim *mpm, AABBc *a);
    ~RayMarch();
    AABBc* getA() const;

    /**
     * Starts the pipeline, calls all other steps, then performs ray marching and billinear interpolation
     * @param ww framebuffer width
     * @param wh framebuffer height
     * @param mpm simulation
     * @param camera camera object
     */
    void march(GLint ww, GLint wh, MPMIntegrationSim *mpm, Camera* camera);

    void loadConfig(const std::string& path);

private:
    void initShader();
    void texQuadInit();
    void bindSpheres(MPMIntegrationSim *mpm);
    void createOutputTexture();

    /**
     * Resizes textures to new framebuffer sizes
     * @param ww framebuffer width
     * @param wh framebuffer height
     */
    void resizeTexutres(GLint ww, GLint wh);

    /**
     * Renders final texture
     */
    void renderTex(Camera* camera);
    void initSkybox();
    void renderSkybox(Camera* camera);
    void generateBRDFLUT();
    
    DepthProcessor *depthMaps;
    BinaryDensityGrid *bdg;
    AABBc *a;
    Shader *shader, *texShader, *interpolation, *spatulaShader;
    Shader *skyboxShader;
    GLuint spheresSSBO = 0, pigmentsSSBO = 0, diffusionSSBO = 0, outputTex = 0, quadVAO = 0, quadVBO = 0, normalDepthTex = 0;
    GLuint skyboxVAO = 0, skyboxVBO = 0;
    SpatulaMesh* spatulaMesh = nullptr;
    GLuint hdrTexture = 0, irradianceTexture = 0, prefilterTexture = 0, brdfLUTTexture = 0;
    int start = 0;
    const float floorCol[4] = {0.375f, 0.35f, 0.325f, 1.0f};
    const float clearData[4] = {1000.0f, 1000.0f, 1000.0f, 1000.0f};

    struct MaterialConfig {
        glm::vec3 albedo;
        float metallic;
        float roughness;
    };
    MaterialConfig fluidMat = {glm::vec3(1.0f), 0.0f, 0.4f};
    MaterialConfig spatulaMetal = {glm::vec3(1.0f), 0.8f, 0.1f};
    MaterialConfig spatulaWood = {glm::vec3(0.59f, 0.29f, 0.0f), 0.0f, 0.8f};
    MaterialConfig floorMat = {glm::vec3(0.85f), 0.0f, 0.6f};
};



#endif //RAYMARCH_H
