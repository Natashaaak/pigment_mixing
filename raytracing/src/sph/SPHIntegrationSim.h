//----------------------------------------------------------------------------------------
/**
 * \file       SPHIntegrationSim.h
 * \brief      SPlisHSPlasH simulation controller
 *
 *  Controls and retrieves data of the SPH simulation.
 *
 */
//----------------------------------------------------------------------------------------

#ifndef SPHINTEGRATIONSIM_H
#define SPHINTEGRATIONSIM_H

#include <Simulator/SimulatorBase.h>
#include <SPlisHSPlasH/Simulation.h>
#include <SPlisHSPlasH/FluidModel.h>
#include <SPlisHSPlasH/TimeManager.h>
#include <SPlisHSPlasH/TimeStep.h>

#include <Utilities/Logger.h>
#include <Utilities/Timing.h>
#include <Utilities/Counting.h>

// INIT_LOGGING;
// INIT_TIMING;
// INIT_COUNTING;

using namespace SPH;


class SPHIntegrationSim {
public:
    /**
     * Loads simulation from json file
     * @param scene path to the scene file
     */
    void loadSimFromJson(const std::string& scene);

    /**
     * Performs one simulation step
     */
    void simStep();

    /**
     * Returns radius of a single particle, every particle shares the same radius
     * @return float rad
     */
    float getRadius();

    /**
     * Returns support SPH radius, every particle has it the same
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
    SPHIntegrationSim();
    ~SPHIntegrationSim();

    /**
     * Returns the link to the vector with all the particles
     * @return std::vector<glm::vec4>& parts
     */
    const std::vector<glm::vec4>& getParticles();

    /**
     * Recounts current positions of all particles after each simulation step
     * @return std::vector<glm::vec4>& parts
     */
    std::vector<glm::vec4>& recountParticles();
private:
    /**
     * Returns pointer from the SPlisHSPlasH to the particle data
     * @return Vector3r * parts
     */
    Vector3r *getParticlePointer();
    SimulatorBase base;
    std::vector<glm::vec4> particles;
};

#endif //SPHINTEGRATIONSIM_H
