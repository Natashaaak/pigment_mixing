#include "simulation.hpp"

void Simulation::precomputeWeights() {
    p_neighbors.resize(Np);

    #pragma omp parallel for num_threads(n_threads)
    for(int p = 0; p < Np; p++) {
        if (!particles.active[p]) continue;
        TV xp = particles.x[p];
        
        // 1. Spočítat báze (jen jednou!)
        int i_base = std::floor((xp(0)-grid.xc)*one_over_dx) - 1;
        int j_base = std::floor((xp(1)-grid.yc)*one_over_dx) - 1;
        p_neighbors[p].base_index[0] = std::max(0, i_base);
        p_neighbors[p].base_index[1] = std::max(0, j_base);

        // p_neighbors[p].base_index[0] = std::max(0, std::min(i_base, (int)Nx - 4));
        // p_neighbors[p].base_index[1] = std::max(0, std::min(j_base, (int)Ny - 4));
#ifdef THREEDIM
        int k_base = std::floor((xp(2)-grid.zc)*one_over_dx) - 1;
        p_neighbors[p].base_index[2] = std::max(0, k_base);

        // p_neighbors[p].base_index[2] = std::max(0, std::min(k_base, (int)Nz - 4));
#endif

        // 2. Spočítat všechny váhy v jednom bloku
        int count = 0;
        for(int i = p_neighbors[p].base_index[0]; i < p_neighbors[p].base_index[0]+4; i++) {
            for(int j = p_neighbors[p].base_index[1]; j < p_neighbors[p].base_index[1]+4; j++) {
#ifdef THREEDIM
                for(int k = p_neighbors[p].base_index[2]; k < p_neighbors[p].base_index[2]+4; k++) {
                    // Zde zavoláš funkci, která vrací váhu i gradient najednou
                    WeightData res = compute_w_and_grad(xp(0), xp(1), xp(2), grid.x[i], grid.y[j], grid.z[k], one_over_dx);
                    p_neighbors[p].weights[count] = res.first;
                    p_neighbors[p].grads[count] = res.second;
                    count++;
                }
#else
                WeightData res = compute_w_and_grad(xp(0), xp(1), grid.x[i], grid.y[j], one_over_dx);
                p_neighbors[p].weights[count] = res.first;
                p_neighbors[p].grads[count] = res.second;
                count++;
#endif
            }
        }
    }
}