// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"
#include <omp.h>

inline float smoothStep(float edge0, float edge1, float x) {
    float t = std::max(0.0f, std::min(1.0f, (x - edge0) / (edge1 - edge0)));
    return t * t * (3.0f - 2.0f * t);
}

void Simulation::G2P(){

    #ifdef WARNINGS
        debug("G2P");
    #endif

    std::fill( particles.pic.begin(),  particles.pic.end(),  TV::Zero() );
    std::fill( particles.flip.begin(), particles.flip.end(), TV::Zero() );
    std::fill( particles.Bmat.begin(), particles.Bmat.end(), TM::Zero() );

    #pragma omp parallel num_threads(n_threads)
    {

        #pragma omp for nowait
        for(int p = 0; p < Np; p++){
            if (!particles.active[p]) continue;

            const auto &pn = p_neighbors[p];
            int count = 0;
            TV xp = particles.x[p];
            TV vp    = TV::Zero();
            TV flipp = TV::Zero();
            TM Bp    = TM::Zero();
            TM L     = TM::Zero();
            Eigen::Matrix<float, 7, 1> grid_pigment_p = Eigen::Matrix<float, 7, 1>::Zero();

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
                        L += grid.v[index] * grad.transpose();
                        grid_pigment_p += grid.pigments[index] * weight;
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
                    L += grid.v[index] * grad.transpose();
                    grid_pigment_p += grid.pigments[index] * weight;
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

            // Compute the symmetric Rate of Strain tensor S
            TM S = 0.5f * (L + L.transpose());
            float shear_intensity = S.norm();
            float mix_factor = smoothStep(pigment_D_edge0, pigment_D_edge1, shear_intensity);
            particles.diffusion_factor[p] = mix_factor;
            // Přidáno násobení dt pro konzistentní rychlost difúze v čase
            mix_factor = std::min(std::max(pigment_D_max * mix_factor * (float)dt, 0.0f), 1.0f);

            particles.pigments[p] = (1.0f - mix_factor) * particles.pigments[p] + mix_factor * grid_pigment_p;
        } // end loop p

    } // end omp paralell

} // end G2P
