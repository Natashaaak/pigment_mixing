// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"
#include <omp.h>

void Simulation::MUSL(){

    #ifdef WARNINGS
        debug("MUSL");
    #endif


    #pragma omp parallel for schedule(static) num_threads(n_threads)
    for(int p=0; p<Np; p++){
        if (!particles.active[p]) continue;

        //// Velicity is updated
        if (flip_ratio < -1){ // APIC
            particles.v[p] = particles.pic[p];
        } else if (flip_ratio < 0){ // AFLIP
            particles.v[p] = (-flip_ratio) * ( particles.v[p] + particles.flip[p] ) + (1 - (-flip_ratio)) * particles.pic[p];
        } else{ // PIC-FLIP
            particles.v[p] =   flip_ratio  * ( particles.v[p] + particles.flip[p] ) + (1 -   flip_ratio)  * particles.pic[p];
        }

    } // end loop over particles


    std::fill( grid.v.begin(), grid.v.end(), TV::Zero() );

    #pragma omp parallel num_threads(n_threads)
    {
        std::vector<TV> grid_v_local(grid_nodes, TV::Zero() );

        #pragma omp for nowait
        for(int p = 0; p < Np; p++){
            if (!particles.active[p]) continue;

            TV xp = particles.x[p];
            const auto &pn = p_neighbors[p];
            int count = 0;
        #ifdef THREEDIM
        #endif

            for(int i = pn.base_index[0]; i < pn.base_index[0]+4; i++){
                T xi = grid.x[i];
                for(int j = pn.base_index[1]; j < pn.base_index[1]+4; j++){
                    T yi = grid.y[j];
        #ifdef THREEDIM
                    for(int k = pn.base_index[2]; k < pn.base_index[2]+4; k++){
                        T zi = grid.z[k];
                        T weight = pn.weights[count++];
                        if (weight > 1e-25){
                            grid_v_local[ind(i,j,k)] += particles.v[p] * weight;
                            if (flip_ratio < 0){ // APIC
                                TV posdiffvec = TV::Zero();
                                posdiffvec(0) = xi-xp(0);
                                posdiffvec(1) = yi-xp(1);
                                posdiffvec(2) = zi-xp(2);
                                grid_v_local[ind(i,j,k)] += particles.Bmat[p] * posdiffvec * apicDinverse * weight;
                            }
                        }
                    } // end for k
        #else
                    T weight = pn.weights[count++];
                    if (weight > 1e-25){
                        grid_v_local[ind(i,j)]     += particles.v[p] * weight;
                        if (flip_ratio < 0){ // APIC
                            TV posdiffvec = TV::Zero();
                            posdiffvec(0) = xi-xp(0);
                            posdiffvec(1) = yi-xp(1);
                            grid_v_local[ind(i,j)] += particles.Bmat[p] * posdiffvec * apicDinverse * weight;
                        }
                    }
        #endif
                } // end for j
            } // end for i
        } // end for p

        #pragma omp critical
        {
            for (int l = 0; l<grid_nodes; l++){
                grid.v[l] += grid_v_local[l];
            } // end for l
        } // end omp critical

    } // end omp parallel


    for (int l = 0; l<grid_nodes; l++){
        T mi = grid.mass[l];
        if (mi > 0)
            grid.v[l] /= (mi/particle_mass);
        else
            grid.v[l].setZero();
    }

} // end P2G_MUSL
