// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"
#include <omp.h>

// Deformation gradient is updated based on the NEW GRID VELOCITIES and the OLD PARTICLE POSITIONS
void Simulation::deformationUpdate(){

    #ifdef WARNINGS
        debug("deformationUpdate");
    #endif

    std::fill( particles.delta_gamma.begin(), particles.delta_gamma.end(), 0.0 );
    unsigned int plastic_count = 0;

    #pragma omp parallel for reduction(+:plastic_count) num_threads(n_threads)
    for(int p=0; p<Np; p++){

        TM sum = TM::Zero();
        TV xp = particles.x[p];

        const auto &pn = p_neighbors[p];
        int count = 0;

        for(int i = pn.base_index[0]; i < pn.base_index[0]+4; i++){
            T xi = grid.x[i];
            for(int j = pn.base_index[1]; j < pn.base_index[1]+4; j++){
                T yi = grid.y[j];
    #ifdef THREEDIM
                for(int k = pn.base_index[2]; k < pn.base_index[2]+4; k++){
                    T zi = grid.z[k];
                    sum += grid.v[ind(i,j,k)] * pn.grads[count++].transpose();
                } // end loop k
    #else
                sum += grid.v[ind(i,j)] * pn.grads[count++].transpose();
    #endif
            } // end loop i
        } // end loop j

        TM Fe_trial = particles.F[p];
        Fe_trial = Fe_trial + dt * sum * Fe_trial;
        particles.F[p] = Fe_trial;

        plasticity(p, plastic_count, Fe_trial);

    } // end loop over particles

    if (!reduce_verbose)
        debug("               Proj: ", plastic_count, " / ", Np);

} // end deformationUpdate
