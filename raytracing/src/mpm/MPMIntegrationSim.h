#ifndef MPMINTEGRATIONSIM_H
#define MPMINTEGRATIONSIM_H

#include "../../../matter/src/simulation/simulation.hpp"

class BinaryDensityGrid;
class AABBc;

class MPMIntegrationSim {
public:
    MPMIntegrationSim();
    ~MPMIntegrationSim();

    void simStep();
    bool isTimeToRender();
    void setupScene();

    /**
     * Returns radius of a single particle, every particle shares the same radius
     * @return float rad
     */
    float getRadius();

    /**
     * Returns support particle radius, every particle has it the same
     * @return float h
     */
    float getSupportRadius();

    /**
     * Returns the amount of particles in the simulation
     * @return unsigned num;
     */
    unsigned getParticleAmount();

    /**
     * Returns indices of all particles that are inside support SPH radius from the particles
     * @param i Particle id
     * @param out vector where to store neighboring particles ids
     */
    void neighborsByIndex(unsigned i, std::vector<unsigned>& out);

    /**
     * Returns the link to the vector with all the particles
     * @return std::vector<glm::vec4>& parts
     */
    std::vector<glm::vec4>& getParticles();

    const std::vector<glm::vec3> &getColors();

    /**
     * Recounts current positions of all particles after each simulation step
     * @return std::vector<glm::vec4>& parts
     */
    std::vector<glm::vec4>& recountParticles();

    /**
     * Returns the boundaries of the simulation grid
     * @return std::pair<glm::vec3, glm::vec3> grid_boundaries
     */
    std::pair<glm::vec3, glm::vec3> getGridBoundaries();

    void setGridData(BinaryDensityGrid* bdg, AABBc* a);

    bool spatulaExists() const;

    glm::mat4 getSpatulaInvTransform() const;

    glm::vec3 getSpatulaDim() const;

    void moveSpatulaY(T deltaY);

private:
    Simulation* sim;
    
    std::vector<glm::vec4> particles;
    std::vector<glm::vec3> colors;
    // colors ratio (cdf), first one is ommited bcs always = 0
    std::vector<float> ratios;

    BinaryDensityGrid* bdg = nullptr;
    AABBc* aabb = nullptr;
};

#endif //MPMINTEGRATIONSIM_H