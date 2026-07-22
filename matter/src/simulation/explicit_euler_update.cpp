// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"
#include <omp.h>


void Simulation::explicitEulerUpdate(){

    // Global force field - zeroed
    std::vector<TV> grid_force(grid_nodes, TV::Zero());

    #pragma omp parallel for num_threads(n_threads)
    for(int p = 0; p < Np; p++){
        if (!particles.active[p]) continue;

        const auto &pn = p_neighbors[p];
        int count = 0;

        TM Fe = particles.F[p];
        TM dPsidF = (elastic_model == ElasticModel::NeoHookean) ? NeoHookeanPiola(Fe) : HenckyPiola(Fe);
        TM tau = dPsidF * Fe.transpose();

        for(int i = pn.base_index[0]; i < pn.base_index[0]+4; i++){
            for(int j = pn.base_index[1]; j < pn.base_index[1]+4; j++){
               
                #ifdef THREEDIM
                for(int k = pn.base_index[2]; k < pn.base_index[2]+4; k++){
                    unsigned int index = ind(i, j, k);
                    const TV& grad = pn.grads[count++];
                    if (grid.mass[index] > 0){
                        TV f_p = tau * grad;
                        #pragma omp atomic
                        grid_force[index](0) += f_p(0);
                        #pragma omp atomic
                        grid_force[index](1) += f_p(1);
                        #pragma omp atomic
                        grid_force[index](2) += f_p(2);
                    }
                }
                #else
                unsigned int index = ind(i, j);
                const TV& grad = pn.grads[count++]; // Precomputed gradient

                if (grid.mass[index] > 0){ // Force contribution from particle
                    TV f_p = tau * grad; // Force contribution from particle
                    // Atomic write by components (prevents race condition)
                    #pragma omp atomic
                    grid_force[index](0) += f_p(0);
                    #pragma omp atomic
                    grid_force[index](1) += f_p(1);
                }
                #endif
            }
        }
    }

    //////////// if external grid gravity: //////////////////
    // std::pair<TMX, TMX> external_gravity_pair = createExternalGridGravity();
    ////////////////////////////////////////////////////////

    T dt_particle_volume = dt * particle_volume;
    TV dt_gravity = dt * gravity;

    #pragma omp parallel for collapse(DIMENSION) num_threads(n_threads)
    for(int i = 0; i < Nx; i++){
        for(int j = 0; j < Ny; j++){
#ifdef THREEDIM
            for(int k = 0; k < Nz; k++){
                unsigned int index = ind(i,j,k);
#else
                unsigned int index = ind(i,j);
#endif
                T mi = grid.mass[index];
                if (mi > 0){

                    TV velocity_increment = -dt_particle_volume * grid_force[index] / mi + dt_gravity;

                    //////////// if external grid gravity: //////////////////
                    // T external_gravity = external_gravity_pair.first(i,j);
                    // T external_gravity = external_gravity_pair.second(i,j);
                    // velocity_increment_x += dt * external_gravity(0);
                    // velocity_increment_y += dt * external_gravity(1);
                    ////////////////////////////////////////////////////////

                    TV old_vi = grid.v[index];
                    TV new_vi = old_vi + velocity_increment;

#ifdef THREEDIM
                    TV Xi(grid.x[i], grid.y[j], grid.z[k]);
#else
                    TV Xi(grid.x[i], grid.y[j]);
#endif

                    boundaryCollision(index, Xi, new_vi);

                    // TODO:
                    // boundaryCorrection(new_xi, new_yi, new_vxi, new_vyi);

                    // Only if impose velocity on certain grid nodes:
                    // overwriteGridVelocity(grid.x[i], grid.y[j], new_vi);

                    grid.v[index]    = new_vi;
                    grid.flip[index] = new_vi - old_vi;

                } // end if non-zero grid mass
#ifdef THREEDIM
            } // end for k
#endif
        } // end for j
    } // end for i

} // end explicitEulerUpdate
