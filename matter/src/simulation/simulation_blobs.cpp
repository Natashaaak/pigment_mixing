#include "simulation.hpp"
#include "../sampling/sampling_particles_vdb.hpp"
#include <vector>
#include <algorithm>
#include <utility>

void Simulation::blobs(const std::vector<float>& colorRatios, const std::vector<Eigen::Matrix<float, 7, 1>>& pigments) {
    // TEMP for testing purposes
    // ObjectVdb blob_left("../matter/levelsets/Blob_left_rotated.vdb", BC::NoSlip, 0.0, TV(0.0f, 0.0f, 0.0f));
    // ObjectVdb blob_right("../matter/levelsets/Blob_right_rotated.vdb");
    // blob_left.scale = 0.2; blob_right.scale = 0.2;
    // // ObjectVdb blob_left("../matter/levelsets/blobs/Blob_01.vdb");
    // // ObjectVdb blob_right("../matter/levelsets/blobs/Blob_02.vdb");
    // // ObjectVdb blob_right1("../matter/levelsets/blobs/Blob_03.vdb");
    // // ObjectVdb blob_right2("../matter/levelsets/blobs/Blob_04.vdb");
    // // ObjectVdb blob_right3("../matter/levelsets/blobs/Blob_05.vdb");
    // // ObjectVdb blob_right4("../matter/levelsets/blobs/Blob_06.vdb");
    // std::vector<ObjectVdb*> vdb_objects = {&blob_left, &blob_right};
    // sampleParticlesFromVdb(*this, vdb_objects, pigments, 0.01f);
    // return;
    // END TEMP
    
    std::vector<std::unique_ptr<ObjectVdb>> vdb_objects_storage;
    std::vector<ObjectVdb*> vdb_objects_ptrs;

    const float min_ratio = 0.1f;
    const float max_ratio = 0.9f;
    const float min_vol = blobDatabase.front().volume;
    const float max_vol = blobDatabase.back().volume;

    size_t n = colorRatios.size();
    std::vector<float> radii(n);
    std::vector<TV> base_centers(n, TV::Zero());

    for (size_t i = 0; i < n; ++i) {
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

        auto blob = std::make_unique<ObjectVdb>(gridA, BC::NoSlip, 0.0, TV::Zero());
        
        TV min_bbox, max_bbox;
        blob->bounds(min_bbox, max_bbox);
        base_centers[i] = (min_bbox + max_bbox) * 0.5f;
        
        // Compute an enclosing radius from the base bounding box
        float r_x = (max_bbox(0) - min_bbox(0)) * 0.5f;
#ifdef THREEDIM
        float r_z = (max_bbox(2) - min_bbox(2)) * 0.5f;
#else
        float r_z = r_x;
#endif
        // VDB bounding boxes include a narrow band of active voxels (padding).
        // We reduce the bounding box radius by a factor to pack them tightly.
        radii[i] = std::max(r_x, r_z) * 0.8f;

        vdb_objects_storage.push_back(std::move(blob));
        vdb_objects_ptrs.push_back(vdb_objects_storage.back().get());
    }
    
    std::vector<TV> offsets(n, TV::Zero());
    if (n == 1) {
        offsets[0] = TV::Zero();
    } else if (n == 2) {
        offsets[0](0) = -radii[0];
        offsets[1](0) =  radii[1];
    } else if (n == 3) {
        offsets[0](0) = 0.0f;
        offsets[1](0) = radii[0] + radii[1];
        
        float x1 = offsets[1](0);
        float r0 = radii[0], r1 = radii[1], r2 = radii[2];
        
        float x2 = ((r0 + r2) * (r0 + r2) - (r1 + r2) * (r1 + r2) + x1 * x1) / (2.0f * x1);
        float z2 = std::sqrt(std::max(0.0f, (r0 + r2) * (r0 + r2) - x2 * x2));
        
        offsets[2](0) = x2;
#ifdef THREEDIM
        offsets[2](2) = z2;
#else
        offsets[2](1) = z2;
#endif
    } else if (n == 4) {
        float r0 = radii[0], r1 = radii[1], r2 = radii[2], r3 = radii[3];
        
        offsets[0](0) = -r0;
        offsets[1](0) =  r1;
        offsets[2](0) = -r2;
        offsets[3](0) =  r3;
        
        auto calc_z = [](float dx, float dist) {
            return std::sqrt(std::max(0.0f, dist * dist - dx * dx));
        };
        
        float z02 = calc_z(offsets[0](0) - offsets[2](0), r0 + r2);
        float z13 = calc_z(offsets[1](0) - offsets[3](0), r1 + r3);
        float z03 = calc_z(offsets[0](0) - offsets[3](0), r0 + r3);
        float z12 = calc_z(offsets[1](0) - offsets[2](0), r1 + r2);
        
        float Z = std::max({z02, z13, z03, z12});
        
#ifdef THREEDIM
        offsets[2](2) = Z;
        offsets[3](2) = Z;
#else
        offsets[2](1) = Z;
        offsets[3](1) = Z;
#endif
    } else {
        float current_x = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            if (i > 0) current_x += radii[i - 1] + radii[i];
            offsets[i](0) = current_x;
        }
    }
    
    // Center the bounds and apply offsets
    if (n > 0) {
        TV min_bounds = offsets[0];
        TV max_bounds = offsets[0];
        for (int d = 0; d < offsets[0].size(); ++d) {
            if (d == 1) continue; // Ignore Y axis
            min_bounds(d) = offsets[0](d) - radii[0];
            max_bounds(d) = offsets[0](d) + radii[0];
        }
        for (size_t i = 1; i < n; ++i) {
            for (int d = 0; d < offsets[i].size(); ++d) {
                if (d == 1) continue; // Ignore Y axis
                min_bounds(d) = std::min(min_bounds(d), offsets[i](d) - radii[i]);
                max_bounds(d) = std::max(max_bounds(d), offsets[i](d) + radii[i]);
            }
        }
        TV center = (min_bounds + max_bounds) * 0.5f;
        center(1) = 0.0f; // Keep grounded in Y
        for (size_t i = 0; i < n; ++i) {
            offsets[i] -= center;
            
            TV final_offset = offsets[i];
            final_offset(0) -= base_centers[i](0);
#ifdef THREEDIM
            final_offset(2) -= base_centers[i](2);
#endif
            // Do not subtract base_centers[i](1) so objects rest on the ground
            vdb_objects_ptrs[i]->offset = final_offset;
        }
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
