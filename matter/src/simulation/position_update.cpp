// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"


void Simulation::positionUpdate(){

    #pragma omp parallel for schedule(static) num_threads(n_threads)
    for(int p=0; p<Np; p++){
        if (!particles.active[p]) continue;

        //// Position is updated according to PIC velocities
        particles.x[p] = particles.x[p] + dt * particles.pic[p];

        if (use_musl == false){
            //// Velicity is updated
            if (flip_ratio < -1){ // APIC
                particles.v[p] = particles.pic[p];
            } else if (flip_ratio < 0){ // AFLIP
                particles.v[p] = (-flip_ratio) * ( particles.v[p] + particles.flip[p] ) + (1 - (-flip_ratio)) * particles.pic[p];
            } else{ // PIC-FLIP
                particles.v[p] =   flip_ratio  * ( particles.v[p] + particles.flip[p] ) + (1 -   flip_ratio)  * particles.pic[p];
            }
        }

        // Ochrana proti NaN a nekonečnu (pokud výpočet exploduje, NaN by ignorovalo limity a shodilo by to paměť rendereru)
        bool is_invalid = false;
        for (unsigned int d = 0; d < dim; ++d) {
            if (std::isnan(particles.x[p](d)) || std::isinf(particles.x[p](d)) ||
                std::isnan(particles.v[p](d)) || std::isinf(particles.v[p](d))) {
                is_invalid = true;
                break;
            }
        }
        if (is_invalid) {
            particles.active[p] = false;
            for (unsigned int d = 0; d < dim; ++d) {
                particles.v[p](d) = 0.0;
                particles.x[p](d) = use_particle_boundaries ? particle_boundary_min(d) : 0.0;
            }
            continue;
        }

        // If periodic boundary conditions (PBC)
        if (pbc){
            if (particles.x[p](0) > Lx){
                particles.x[p](0) = particles.x[p](0) - Lx;
            }
            else if (particles.x[p](0) < 0){
                particles.x[p](0) = Lx + particles.x[p](0);
            }
        }

        // Pojistka: částice nesmí propadnout pod podložku (tvrdý limit na y=0).
        // Jistota pro případ, že numerická chyba při velkém stlačení posune částici dolů.
        if (particles.x[p](1) < 0.0) {
            particles.x[p](1) = 0.0;
            if (particles.v[p](1) < 0.0) {
                particles.v[p](1) = 0.0;
            }
        }

        // Change particle positions (and velocities)
        // Can be used as a crude PBC if there are no boundary interactions
        // To be hard-coded depending on application
        if (change_particle_positions){
            if (particles.x[p](0) > 0.8){
                particles.x[p](0)  = -0.1;
                particles.x[p](1) +=  0.27;
            }
        }

        // Enforce boundary limits to prevent unstable particles from expanding the grid infinitely
        if (use_particle_boundaries) {
            for (unsigned int d = 0; d < dim; ++d) {
                if (particles.x[p](d) < particle_boundary_min(d)) {
                    particles.x[p](d) = particle_boundary_min(d);
                    particles.active[p] = false;
                    particles.v[p](d) = 0.0;
                } else if (particles.x[p](d) > particle_boundary_max(d)) {
                    particles.x[p](d) = particle_boundary_max(d);
                    particles.active[p] = false;
                    particles.v[p](d) = 0.0;
                }
            }
            if (!particles.active[p]) {
                for (unsigned int d = 0; d < dim; ++d) particles.v[p](d) = 0.0;
            }
        }
    } // end loop over particles
}
