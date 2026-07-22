// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"

void Simulation::updateDt(){

    T max_speed_sq = 0.0;
    #pragma omp parallel for reduction(max:max_speed_sq) num_threads(n_threads)
    for(int p = 0; p < Np; p++) {
        if (!particles.active[p]) continue;
        T speed_sq = particles.v[p].squaredNorm();
        if (speed_sq > max_speed_sq) {
            max_speed_sq = speed_sq;
        }
    }
    T max_speed = std::sqrt(max_speed_sq);

    // if (max_speed >= wave_speed){
    //     debug("               FYI the particle speed ", max_speed, " is larger than elastic wave speed ", wave_speed);
    // }

#ifdef WARNINGS
    debug("               dt_max = ", dt_max);
#endif

    if (std::abs(max_speed) > 1e-10){
        T dt_cfl = cfl * dx / max_speed;
#ifdef WARNINGS
        debug("               dt_cfl = ", dt_cfl);
#endif
        dt = std::min(dt_cfl, dt_max);
    } else {
        dt = dt_max;
#ifdef WARNINGS
        debug("               dt_cfl = not computed, max_speed too low");
#endif
    }

    dt = std::min(dt, frame_dt*(frame+1) - time);
    // dt = std::min(dt, final_time         - time);
    dt = std::max(dt, min_dt);

#ifdef WARNINGS
    debug("               dt     = ", dt);
#endif


    // Here one may hard-code a special gravity evolution with time
    // Example here: linear gravity increase until "gravity_time"
    if (gravity_special){

        if (time < gravity_time){
            gravity = gravity_final * time/gravity_time;
        }
        else{
            gravity = gravity_final;
            // if (no_liftoff){
            //     for(int p=0; p<Np; p++){
            //         if (particles.x[p](0) > 0.5*Lx){
            //             particles.x[p](0) -= 0.5*Lx;
            //             particles.x[p](1) -= Ly+10*dx;
            //         }
            //     }
            //     no_liftoff = false;
            }

    } // end gravity_special



} // end updateDt
