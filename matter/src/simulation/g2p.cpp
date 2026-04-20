// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"
#include <omp.h>


void Simulation::G2P(){

    #ifdef WARNINGS
        debug("G2P");
    #endif

    std::fill( particles.pic.begin(),  particles.pic.end(),  TV::Zero() );
    std::fill( particles.flip.begin(), particles.flip.end(), TV::Zero() );
    std::fill( particles.Bmat.begin(), particles.Bmat.end(), TM::Zero() );
    std::fill( particles.flux.begin(), particles.flux.end(), std::array<TV, 4>{{TV::Zero(), TV::Zero(), TV::Zero(), TV::Zero()}} );

    #pragma omp parallel num_threads(n_threads)
    {

        #pragma omp for nowait
        for(int p = 0; p < Np; p++){
            const auto &pn = p_neighbors[p];
            int count = 0;
            TV xp = particles.x[p];
            TV vp    = TV::Zero();
            TV flipp = TV::Zero();
            TM Bp    = TM::Zero();
            std::array<TV, 4> flux_p = {TV::Zero(), TV::Zero(), TV::Zero(), TV::Zero()};
            Eigen::Vector4f pigments_gain = Eigen::Vector4f::Zero();

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
                        vp += grid.v[index] * weight;
                        if (flip_ratio < 0){ // APIC
                            TV posdiffvec = TV::Zero();
                            posdiffvec(0) = xi-xp(0);
                            posdiffvec(1) = yi-xp(1);
                            posdiffvec(2) = zi-xp(2);
                            Bp += grid.v[index] * posdiffvec.transpose() * weight;
                        }
                        if (flip_ratio >= -1){ // PIC-FLIP or AFLIP
                            flipp += grid.flip[index] * weight;
                        }
                        for (int c = 0; c < 4; ++c) {
                            flux_p[c] += grid.pigments[index](c) * grad;
                        }
                        pigments_gain += grid.div_flux[index] * weight;
                    } // end loop k
        #else
                    unsigned int index = ind(i, j);
                    T weight = pn.weights[count];
                    const TV& grad = pn.grads[count];
                    count++;
                    vp += grid.v[index] * weight;
                    if (flip_ratio < 0){ // APIC
                        TV posdiffvec = TV::Zero();
                        posdiffvec(0) = xi-xp(0);
                        posdiffvec(1) = yi-xp(1);
                        Bp += grid.v[index] * posdiffvec.transpose() * weight;
                    }
                    if (flip_ratio >= -1){ // PIC-FLIP or AFLIP
                        flipp += grid.flip[index] * weight;
                    }
                    for (int c = 0; c < 4; ++c) {
                        flux_p[c] += grid.pigments[index](c) * grad;
                    }
                    pigments_gain += grid.div_flux[index] * weight;
        #endif
                } // end loop j
            } // end loop i
            particles.pic[p] = vp;
            if (flip_ratio < 0){ // APIC
                particles.Bmat[p] = Bp;
            }
            if (flip_ratio >= -1){ // PIC-FLIP or AFLIP
                particles.flip[p] = flipp;
            }

            for (int c = 0; c < 4; ++c) {
                particles.flux[p][c] = -pigment_D * flux_p[c];
            }

            particles.pigments[p] += dt * pigments_gain;
            particles.pigments[p] = particles.pigments[p].cwiseMax(0.0f).cwiseMin(1.0f);
        } // end loop p

    } // end omp paralell

} // end G2P
