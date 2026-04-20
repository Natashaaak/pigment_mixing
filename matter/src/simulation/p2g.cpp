// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"
#include <omp.h>

void Simulation::P2G(){

    #ifdef WARNINGS
        debug("P2G");
    #endif

    #pragma omp parallel num_threads(n_threads)
    {
        std::vector<TV> grid_v_local(grid_nodes, TV::Zero() );
        std::vector<T> grid_mass_local(grid_nodes);
        std::vector<T> grid_friction_local(grid_nodes);
        std::vector<Eigen::Vector4f> grid_pigments_local(grid_nodes, Eigen::Vector4f::Zero());
        std::vector<Eigen::Vector4f> grid_div_flux_local(grid_nodes, Eigen::Vector4f::Zero());

        #pragma omp for nowait
        for(int p = 0; p < Np; p++){
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
                        unsigned int index = ind(i, j, k);
                        T weight = pn.weights[count];
                        const TV& grad = pn.grads[count];
                        count++;
                        if (weight > 1e-25){
                            grid_mass_local[index]  += weight;
                            grid_v_local[index]     += particles.v[p] * weight;
                            grid_pigments_local[index] += particles.pigments[p] * weight;
                            for (int c = 0; c < 4; ++c) {
                                grid_div_flux_local[index](c) += particles.flux[p][c].dot(grad);
                            }
                            if (flip_ratio < 0){ // APIC
                                TV posdiffvec = TV::Zero();
                                posdiffvec(0) = xi-xp(0);
                                posdiffvec(1) = yi-xp(1);
                                posdiffvec(2) = zi-xp(2);
                                grid_v_local[index] += particles.Bmat[p] * posdiffvec * apicDinverse * weight;
                            }
                            if (use_mibf)
                                grid_friction_local[index] += particles.muI[p] * weight;
                        }
                    } // end for k
        #else
                    unsigned int index = ind(i, j);
                    T weight = pn.weights[count];
                    const TV& grad = pn.grads[count];
                    count++;
                    if (weight > 1e-25){
                        grid_mass_local[index]  += weight;
                        grid_v_local[index]     += particles.v[p] * weight;
                        grid_pigments_local[index] += particles.pigments[p] * weight;
                        for (int c = 0; c < 4; ++c) {
                            grid_div_flux_local[index](c) += particles.flux[p][c].dot(grad);
                        }
                        if (flip_ratio < 0){ // APIC
                            TV posdiffvec = TV::Zero();
                            posdiffvec(0) = xi-xp(0);
                            posdiffvec(1) = yi-xp(1);
                            grid_v_local[index] += particles.Bmat[p] * posdiffvec * apicDinverse * weight;
                        }
                        if (use_mibf)
                            grid_friction_local[index] += particles.muI[p] * weight;
                    }
        #endif
                } // end for j
            } // end for i
        } // end for p

        #pragma omp critical
        {
            for (int l = 0; l<grid_nodes; l++){
                grid.mass[l]          += grid_mass_local[l];
                grid.v[l]             += grid_v_local[l];
                grid.pigments[l]      += grid_pigments_local[l];
                grid.div_flux[l]      += grid_div_flux_local[l];
                if (use_mibf)
                    grid.friction[l]  += grid_friction_local[l];
            } // end for l
        } // end omp critical

    } // end omp parallel

    ///////////////////////////////////////////////////////////
    // At this point in time grid.mass is equal to m_i / m_p //
    ///////////////////////////////////////////////////////////

    #pragma omp parallel for num_threads(n_threads)
    for (int l = 0; l<grid_nodes; l++){
        T mi = grid.mass[l];
        if (mi > 0) {
            grid.v[l] /= mi;
            grid.pigments[l] /= mi;
            grid.div_flux[l] /= mi;
        } else {
            grid.v[l].setZero();
            grid.pigments[l].setZero();
            grid.div_flux[l].setZero();
        }
        //grid.v[l] = (mi > 0) ? grid.v[l]/mi : TV::Zero(); // condition ? result_if_true : result_if_false
        if (use_mibf)
            grid.friction[l] /= mi;
    }

    for(auto&& m: grid.mass){
        m *= particle_mass;
    }

} // end P2G
