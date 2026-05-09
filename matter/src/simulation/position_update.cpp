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

        // --- KOREKCE ČÁSTIC HLUBOKO VE ŠPACHTLI ---
        // Pokud mřížka nedokázala částici zastavit a ta pronikla do geometrie špachtle
        if (spatula_ptr != nullptr) {
            Eigen::Vector4f worldPos;
#ifdef THREEDIM
            worldPos << (float)particles.x[p](0), (float)particles.x[p](1), (float)particles.x[p](2), 1.0f;
#else
            worldPos << (float)particles.x[p](0), 0.0f, (float)particles.x[p](1), 1.0f;
#endif
            Eigen::Vector3f localPos = (spatula_ptr->invTransform.matrix() * worldPos).head<3>();
            T sd = spatula_ptr->sdSpatula(localPos);

            // Pokud je částice hluboko uvnitř (např. víc než 0.2 * dx)
            if (sd < (T)(-0.2 * dx)) {
                TV n = spatula_ptr->normal(particles.x[p]);
                
                // Fyzický posun částice přesně na povrch špachtle
                particles.x[p] -= sd * n;

                // Vynulování akumulovaného pružného stresu zamezí obří odpudivé explozi
                particles.F[p].setIdentity();

                // Úprava rychlosti, aby nesměřovala dovnitř (promítnutí do tečné roviny)
                TV v_obj = spatula_ptr->velocity(particles.x[p]);
                TV v_rel = particles.v[p] - v_obj;
                T dot = v_rel.dot(n);
                if (dot < 0) { // Pokud složka rychlosti míří dovnitř špachtle
                    particles.v[p] = v_rel - dot * n + v_obj;
                }
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
