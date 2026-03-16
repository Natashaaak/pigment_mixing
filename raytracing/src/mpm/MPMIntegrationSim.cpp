#include "MPMIntegrationSim.h"

MPMIntegrationSim::MPMIntegrationSim(){
    sim = new Simulation();
}

MPMIntegrationSim::~MPMIntegrationSim(){
    delete sim;
}

void MPMIntegrationSim::setupScene(){
    float fps = 60;
    sim->initializeBasic("mpm_integration_test");
    sim->setupScene(fps);
    sim->prepareSimulation();
    particles.resize(getParticleAmount());
}

void MPMIntegrationSim::simStep(){
    sim->step();
}

bool MPMIntegrationSim::isTimeToRender() {
    return sim->frameFinished();
}

float MPMIntegrationSim::getRadius() {
    return sim->dx * 0.5f;  // set particle radius to be half of grid cell size
}

float MPMIntegrationSim::getSupportRadius() {
    return sim->dx * 1.5f;  // with using SPLINEDEG 2 kernel, the support radius is 1.5 times the grid cell size
}

std::vector<glm::vec4> &MPMIntegrationSim::recountParticles() {
    Particles &particles = sim->particles;
    for (unsigned int i = 0; i < sim->Np; i++) {
        // store position in .xyz and volume in .w
        if (sim->dim == 3) {
            this->particles[i] = glm::vec4(particles.x[i](0), particles.x[i](1), particles.x[i](2), sim->particle_volume);
        } else {
            // make z coordinate random
            this->particles[i] = glm::vec4(particles.x[i](0), particles.x[i](1), 0.0f, sim->particle_volume);
        }
    }

    return this->particles;
}


const std::vector<glm::vec4> &MPMIntegrationSim::getParticles() {
    return particles;
}

unsigned MPMIntegrationSim::getParticleAmount() {
    return sim->Np;
}

void MPMIntegrationSim::neighborsByIndex(unsigned i, std::vector<unsigned int> &out) {
    // TODO: implement
}

std::pair<glm::vec3, glm::vec3> MPMIntegrationSim::getGridBoundaries() {
    auto boundaries = sim->getGridBoundaries();
    return std::make_pair(
        glm::vec3(boundaries.first[0], 
                  boundaries.first[1], 
                  boundaries.first[2]), 
        glm::vec3(boundaries.second[0], 
                  boundaries.second[1], 
                  boundaries.second[2]));
}
