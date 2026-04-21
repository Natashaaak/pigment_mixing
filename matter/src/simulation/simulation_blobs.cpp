#include "simulation.hpp"
#include "../sampling/sampling_particles_vdb.hpp"
#include <vector>
#include <algorithm>
#include <utility>

void Simulation::blobs(const std::vector<float>& colorRatios, const std::vector<Eigen::Matrix<float, 7, 1>>& pigments) {
    std::vector<std::unique_ptr<ObjectVdb>> vdb_objects_storage;
    std::vector<ObjectVdb*> vdb_objects_ptrs;

    const float min_ratio = 0.1f;
    const float max_ratio = 0.9f;
    const float min_vol = blobDatabase.front().volume;
    const float max_vol = blobDatabase.back().volume;

    // Center the blobs around the origin
    TV offset = TV::Zero();
    offset(0) = -0.25f * (colorRatios.size() - 1);

    for (size_t i = 0; i < colorRatios.size(); ++i) {
        float ratio = std::max(min_ratio, std::min(max_ratio, colorRatios[i]));
        float targetVolume = min_vol + (ratio - min_ratio) / (max_ratio - min_ratio) * (max_vol - min_vol);
        targetVolume = std::max(min_vol, std::min(max_vol, targetVolume)); // Bezpečné ořezání
        
        int idxA = getMorphPair(targetVolume);
        int idxB = idxA + 1;
        
        auto gridA = ObjectVdb::loadGrid(blobDatabase[idxA].path);
        auto gridB = ObjectVdb::loadGrid(blobDatabase[idxB].path);
        float t = findTForExactVolume(gridA, gridB, blobDatabase[idxA].calibration_factor, blobDatabase[idxB].calibration_factor, targetVolume);

        if (t > 1e-6f) {
            gridA->topologyUnion(*gridB);
            auto accB = gridB->getAccessor();
            for (auto it = gridA->beginValueAll(); it; ++it) {
                openvdb::Coord coord = it.getCoord();
                float valStart = it.getValue();
                float valEnd = accB.getValue(coord);
                it.setValue(valStart * (1.0f - t) + valEnd * t);
            }
        }

        auto blob = std::make_unique<ObjectVdb>(gridA, BC::NoSlip, 0.0, offset);

        vdb_objects_storage.push_back(std::move(blob));
        vdb_objects_ptrs.push_back(vdb_objects_storage.back().get());

        offset(0) += 0.5;
    }

    sampleParticlesFromVdb(*this, vdb_objects_ptrs, pigments, 0.01f);
}

int Simulation::getMorphPair(float targetVolume) {
    if (targetVolume <= blobDatabase.front().volume) return 0;
    if (targetVolume >= blobDatabase.back().volume) return blobDatabase.size() - 2;

    // find the right pair
    for (size_t i = 0; i < blobDatabase.size() - 1; ++i) {
        if (targetVolume >= blobDatabase[i].volume && targetVolume <= blobDatabase[i+1].volume) {
            return i;
        }
    }
    return blobDatabase.size() - 2; // fallback to the largest blob if targetVolume is out of range
}

float Simulation::findTForExactVolume(typename ObjectVdb::GridT::Ptr gridA, typename ObjectVdb::GridT::Ptr gridB, float factorA, float factorB, float targetVolume) {
    float t_low = 0.0f;
    float t_high = 1.0f;
    float t_mid = 0.5f;

    auto accA = gridA->getAccessor();
    auto accB = gridB->getAccessor();

    openvdb::CoordBBox bbox = gridA->evalActiveVoxelBoundingBox();
    bbox.expand(gridB->evalActiveVoxelBoundingBox());

    float voxel_size = 0.03f;
    float voxel_vol = voxel_size * voxel_size * voxel_size;

    // Pre-extract only the boundary voxels (where the interpolation could cross 0) to make the loop lightning fast
    std::vector<std::pair<float, float>> boundary_voxels;
    int always_inside_count = 0;

    for (int z = bbox.min().z(); z <= bbox.max().z(); ++z) {
        for (int y = bbox.min().y(); y <= bbox.max().y(); ++y) {
            for (int x = bbox.min().x(); x <= bbox.max().x(); ++x) {
                openvdb::Coord coord(x, y, z);
                float valA = accA.getValue(coord);
                float valB = accB.getValue(coord);
                
                if (valA <= 0.0f && valB <= 0.0f) {
                    always_inside_count++;
                } else if (valA <= 0.0f || valB <= 0.0f) {
                    boundary_voxels.push_back({valA, valB});
                }
            }
        }
    }

    for(int i = 0; i < 15; ++i) { // 15 iterations (microsecond speed) for very high precision
        t_mid = (t_low + t_high) * 0.5f;
        
        int current_inside = always_inside_count;
        for (const auto& pair : boundary_voxels) {
            float blended_val = pair.first * (1.0f - t_mid) + pair.second * t_mid;
            if (blended_val <= 0.0f) {
                current_inside++;
            }
        }

        float currentFactor = factorA * (1.0f - t_mid) + factorB * t_mid;
        
        float currentVol = current_inside * voxel_vol * currentFactor;

        if (currentVol < targetVolume) t_low = t_mid;
        else t_high = t_mid;
    }
    return t_mid;
}
