// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"
#include <omp.h>
#include <algorithm>

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
            T grid_shear_intensity_p = 0.0;

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
                        grid_shear_intensity_p += grid.shear_intensity[index] * weight;
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
                    grid_shear_intensity_p += grid.shear_intensity[index] * weight;
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

            // --- Symmetric Mixing Logic ---

            // 1. Vypočítáme novou intenzitu smyku pro tuto částici z gradientu rychlosti.
            //    Tato hodnota se uloží a použije se v P2G v příštím kroku.
            TM S = 0.5f * (L + L.transpose());
            float new_shear_intensity = S.norm();
            // Uložíme surovou intenzitu smyku. smoothStep se aplikuje až na zprůměrovanou hodnotu.
            particles.diffusion_factor[p] = new_shear_intensity;

            // 2. Pro aktuální míchání použijeme zprůměrovanou hodnotu z mřížky,
            //    která byla spočítána v P2G.
            float mix_factor = smoothStep(pigment_D_edge0, pigment_D_edge1, grid_shear_intensity_p);

            // 3. Aplikace časového "boostu" pro zrychlení míchání v průběhu času.
            //    Parametry se načítají z pigment_config.json.
            float current_time = (float)time;
            float time_factor = 1.0f;
            if (current_time > start_boost_time && end_boost_time > start_boost_time) {
                float t = (current_time - start_boost_time) / (end_boost_time - start_boost_time);
                t = std::clamp(t, 0.0f, 1.0f); // Lineární náběh od 0 do 1
                time_factor = 1.0f + t * (boost_factor - 1.0f);
            }

            // Aplikace časového faktoru na finální `scaled_mix_factor`
            float dynamic_D_max = pigment_D_max * time_factor;
            float scaled_mix_factor = std::clamp(dynamic_D_max * mix_factor * (float)dt, 0.0f, 1.0f);

            // Symetrická aktualizace: částice se přibližuje k průměrné barvě okolí.
            // p_new = (1 - mix) * p_old + mix * p_avg
            particles.pigments[p] = (1.0f - scaled_mix_factor) * particles.pigments[p] + scaled_mix_factor * grid_pigment_p;

            // Jelikož pracujeme pouze s neprůhlednými pigmenty, jejich součet by měl být vždy 1.
            // Normalizujeme vždy, pokud je přítomen nějaký pigment, abychom opravili numerické chyby.
            float pigment_sum = particles.pigments[p].head<4>().sum();
            if (pigment_sum > 1e-6f) {
                particles.pigments[p].head<4>() /= pigment_sum;
            }
        } // end loop p

    } // end omp paralell

} // end G2P
