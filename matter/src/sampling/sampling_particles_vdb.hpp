// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#ifndef SAMPLING_PARTICLES_VDB_HPP
#define SAMPLING_PARTICLES_VDB_HPP

#include "../tools.hpp"
#include "../data_structures.hpp"
#include "../../deps/tph_poisson-0.3/thinks/poisson_disk_sampling/poisson_disk_sampling.h"

#include "../objects/object_vdb.hpp"

template <typename S>
#ifdef THREEDIM
void sampleParticlesFromVdb(S& sim, ObjectVdb& obj, T kRadius, T ppc = 8)
#else // TWODIM
void sampleParticlesFromVdb(S& sim, ObjectVdb& obj, T kRadius, T ppc = 6)
#endif
{

    debug("Sampling particles from VDB...");

    std::uint32_t kAttempts = 30;
    std::uint32_t kSeed = 42;

    TV min_corner, max_corner;
    obj.bounds(min_corner, max_corner);
    TV L = max_corner - min_corner;

    #ifdef THREEDIM
        debug("    Min corner: ", min_corner(0), ", ", min_corner(1), ", ", min_corner(2));
        debug("    Max corner: ", max_corner(0), ", ", max_corner(1), ", ", max_corner(2));

        std::array<T, 3> kXMin = std::array<T, 3>{{min_corner(0), min_corner(1), min_corner(2)}};
        std::array<T, 3> kXMax = std::array<T, 3>{{max_corner(0), max_corner(1), max_corner(2)}};
        std::vector<std::array<T, 3>> square_samples = thinks::PoissonDiskSampling(kRadius, kXMin, kXMax, kAttempts, kSeed);

        sim.dx = std::cbrt(ppc / T(square_samples.size()) * L(0)*L(1)*L(2));
        sim.particle_volume = sim.dx * sim.dx * sim.dx / ppc;
        sim.particle_mass = sim.rho * sim.particle_volume;

        sim.Lx = L(0);
        sim.Ly = L(1);
        sim.Lz = L(2);
    #else // TWODIM
        debug("    Min corner: ", min_corner(0), ", ", min_corner(1));
        debug("    Max corner: ", max_corner(0), ", ", max_corner(1));

        std::array<T, 2> kXMin = std::array<T, 2>{{min_corner(0), min_corner(1)}};
        std::array<T, 2> kXMax = std::array<T, 2>{{max_corner(0), max_corner(1)}};
        std::vector<std::array<T, 2>> square_samples = thinks::PoissonDiskSampling(kRadius, kXMin, kXMax, kAttempts, kSeed);

        sim.dx = std::sqrt(ppc / T(square_samples.size()) * L(0)*L(1));
        sim.particle_volume = sim.dx * sim.dx / ppc;
        sim.particle_mass = sim.rho * sim.particle_volume;

        sim.Lx = L(0);
        sim.Ly = L(1);
    #endif

    debug("    Number of square samples: ", square_samples.size());
    debug("    dx set to ", sim.dx);

    std::vector<TV> samples;
    for(int p = 0; p < square_samples.size(); p++){

        #ifdef THREEDIM
            TV point(square_samples[p][0], square_samples[p][1], square_samples[p][2]);
        #else // TWODIM
            TV point(square_samples[p][0], square_samples[p][1]);
        #endif

        if ( obj.inside(point) ){
            samples.push_back(point);
        }
    }

    sim.Np = samples.size();
    debug("    Number of particles samples: ", sim.Np);

    sim.particles = Particles(sim.Np);
    sim.particles.x = samples;

} // end sampleParticles


template <typename S>
#ifdef THREEDIM
void sampleParticlesFromVdb(S& sim, std::vector<ObjectVdb*>& objects, std::vector<uint8_t>& colors, T kRadius, T ppc = 8)
#else // TWODIM
void sampleParticlesFromVdb(S& sim, std::vector<ObjectVdb*>& objects, std::vector<uint8_t>& colors, T kRadius, T ppc = 6)
#endif
{
    debug("Sampling particles from multiple VDB objects...");

    std::uint32_t kAttempts = 30;
    std::uint32_t kSeed = 42;
    std::vector<TV> all_final_samples;
    std::vector<uint8_t> all_final_colors;
    
    // Total volume/area accumulator for dx calculation
    T total_volume = 0;

    for (size_t i = 0; i < objects.size(); ++i) {
        ObjectVdb& obj = *objects[i];
        TV min_corner, max_corner;
        obj.bounds(min_corner, max_corner);
        TV L = max_corner - min_corner;

        #ifdef THREEDIM
            total_volume += L(0) * L(1) * L(2);
            std::array<T, 3> kXMin = {{min_corner(0), min_corner(1), min_corner(2)}};
            std::array<T, 3> kXMax = {{max_corner(0), max_corner(1), max_corner(2)}};
            // Use a different seed for each object to avoid structured patterns
            auto square_samples = thinks::PoissonDiskSampling(kRadius, kXMin, kXMax, kAttempts, kSeed + i);
        #else
            total_volume += L(0) * L(1);
            std::array<T, 2> kXMin = {{min_corner(0), min_corner(1)}};
            std::array<T, 2> kXMax = {{max_corner(0), max_corner(1)}};
            auto square_samples = thinks::PoissonDiskSampling(kRadius, kXMin, kXMax, kAttempts, kSeed + i);
        #endif

        // Perform inside test for this specific object
        for (const auto& s : square_samples) {
            #ifdef THREEDIM
                TV point(s[0], s[1], s[2]);
            #else
                TV point(s[0], s[1]);
            #endif

            if (obj.inside(point)) {
                all_final_samples.push_back(point);
            }
        }

        // Assign colors for this object's samples
        debug("    Object ", i, ": ", square_samples.size(), " square samples, ", all_final_samples.size() - all_final_colors.size(), colors[i % colors.size()]);
        all_final_colors.insert(all_final_colors.end(), all_final_samples.size() - all_final_colors.size(), colors[i % colors.size()]);
    }

    // --- Physical properties calculation based on global stats ---
    sim.Np = all_final_samples.size();
    
    #ifdef THREEDIM
        // Adjust dx based on total samples across all objects
        sim.dx = std::cbrt(ppc / T(sim.Np) * total_volume);
        sim.particle_volume = (sim.dx * sim.dx * sim.dx) / ppc;
    #else
        sim.dx = std::sqrt(ppc / T(sim.Np) * total_volume);
        sim.particle_volume = (sim.dx * sim.dx) / ppc;
    #endif

    sim.particle_mass = sim.rho * sim.particle_volume;
    sim.particles = Particles(sim.Np);
    sim.particles.x = all_final_samples;
    sim.particles.color = all_final_colors;

    debug("  Total particles sampled: ", sim.Np);
    debug("  Final dx: ", sim.dx);
}


#endif  // SAMPLING_PARTICLES_VDB_HPP
