#include "SPHIntegrationSim.h"

SPHIntegrationSim::SPHIntegrationSim() = default;

float SPHIntegrationSim::getRadius() {
    auto * sim = Simulation::getCurrent();
    return sim->getParticleRadius();
}

float SPHIntegrationSim::getSupportRadius() {
    auto *sim = Simulation::getCurrent();
    return sim->getSupportRadius();
}

void SPHIntegrationSim::loadSimFromJson(const std::string &scene) {
    std::vector<std::string> args = { "GLHydroSurface", "--no-gui", scene};
    base.init(args, "aaa");
    base.initSimulation();
    base.runSimulation();
    particles.resize(getParticleAmount());
}

void SPHIntegrationSim::simStep() {
    auto *sim = Simulation::getCurrent();
    auto *t = sim->getTimeStep();
    t->step();

}

std::vector<glm::vec4> &SPHIntegrationSim::recountParticles() {
    auto a = getParticlePointer();
    float r = getRadius();
    for (int i = 0; i < getParticleAmount(); ++i) {
        glm::vec4 curr;
        curr.x = a[i].x();
        curr.y = a[i].y();
        curr.z = a[i].z();
        curr.w = r;
        particles[i] = curr;
    }
    return particles;
}

const std::vector<glm::vec4> &SPHIntegrationSim::getParticles() {
    return particles;
}

unsigned SPHIntegrationSim::getParticleAmount() {
    auto *sim = Simulation::getCurrent();
    auto *fluid = sim->getFluidModel(0);
    return fluid->numActiveParticles();
}

Vector3r *SPHIntegrationSim::getParticlePointer() {
    auto *sim = Simulation::getCurrent();
    auto *fluid = sim->getFluidModel(0);
    return &fluid->getPosition(0);
}

SPHIntegrationSim::~SPHIntegrationSim() {
    base.cleanup();
}

void SPHIntegrationSim::neighborsByIndex(unsigned i, std::vector<unsigned int> &out) {
    auto* sim = Simulation::getCurrent();
    auto* fm  = sim->getFluidModel(0);
    if (!fm || i >= fm->numActiveParticles()) return;

    const unsigned ps  = fm->getPointSetIndex();
    const unsigned cnt = sim->numberOfNeighbors(ps, ps, i);
    out.reserve(cnt);
    for (unsigned k = 0; k < cnt; ++k)
        out.emplace_back(sim->getNeighbor(ps, ps, i, k));
}
