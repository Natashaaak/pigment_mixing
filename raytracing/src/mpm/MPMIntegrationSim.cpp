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
        this->particles[i] = glm::vec4(particles.x[i](0), particles.x[i](1), particles.x[i](2), sim->particle_volume);
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
