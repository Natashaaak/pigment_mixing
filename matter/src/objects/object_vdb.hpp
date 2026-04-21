// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#ifndef VDB_OBJ_H
#define VDB_OBJ_H

#include "object_general.hpp"

#define IMATH_HALF_NO_LOOKUP_TABLE
#include <openvdb/openvdb.h>
#include <openvdb/io/File.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/LevelSetMorph.h>

// Make sure to fill the interor when creating the levelset, otherwise normals
// and signed distances are not computed correctly

class ObjectVdb : public ObjectGeneral {
public:
    typedef typename openvdb::Grid<typename openvdb::tree::Tree4<float, 5, 4, 3>::Type> GridT;
    typedef typename GridT::TreeType TreeT;
    typedef typename openvdb::tools::ScalarToVectorConverter<GridT>::Type GradientGridT;
    typedef typename GradientGridT::TreeType GradientTreeT;

    typename GridT::Ptr grid;
    typename GradientGridT::Ptr grad_phi;
    std::vector<float> buffer;
    openvdb::CoordBBox bbox;
    openvdb::Coord dim;
    TV offset;
    T scale;

    ~ObjectVdb(){}

    static typename GridT::Ptr loadGrid(std::string filename) {
        openvdb::io::File file(filename);
        file.open();
        openvdb::GridPtrVecPtr my_grids = file.getGrids();
        file.close();
        typename GridT::Ptr grid;
        for (openvdb::GridPtrVec::iterator iter = my_grids->begin(); iter != my_grids->end(); ++iter) {
            grid = openvdb::gridPtrCast<GridT>(*iter);
        }
        return grid;
    }

    ObjectVdb(std::string filename, BC bc_in = BC::NoSlip, T friction_in = 0.0, TV offset_in = TV::Zero(), T scale_in = 1.0, std::string name_in = "") 
        : ObjectVdb(loadGrid(filename), bc_in, friction_in, offset_in, scale_in, name_in) {}

    ObjectVdb(typename GridT::Ptr grid_in, BC bc_in = BC::NoSlip, T friction_in = 0.0, TV offset_in = TV::Zero(), T scale_in = 1.0, std::string name_in = "") 
        : ObjectGeneral(bc_in, friction_in, name_in), offset(offset_in), scale(scale_in) {
        
        grid = grid_in;

        openvdb::tools::Gradient<GridT> mg(*grid);
        grad_phi = mg.process();

        bbox = grid->evalActiveVoxelBoundingBox();
        dim = bbox.extents();
        buffer.resize(dim.x() * dim.y() * dim.z());

        auto accessor = grid->getAccessor();
        int idx = 0;
        for (int z = bbox.min().z(); z <= bbox.max().z(); ++z) {
            for (int y = bbox.min().y(); y <= bbox.max().y(); ++y) {
                for (int x = bbox.min().x(); x <= bbox.max().x(); ++x) {
                    buffer[idx++] = accessor.getValue(openvdb::Coord(x, y, z));
                }
            }
        }
    }

    bool inside(const TV& X_in) const override {
        int tv_dim = X_in.size();
        Eigen::Matrix<T, 3, 1> X;
        X.setZero();
        for (int d = 0; d < tv_dim; d++)
            X(d) = (X_in(d) - offset(d)) / scale;

        openvdb::tools::GridSampler<TreeT, openvdb::tools::BoxSampler> interpolator(grid->constTree(), grid->transform());
        openvdb::math::Vec3<T> P(X(0), X(1), X(2));
        float phi = interpolator.wsSample(P); // this is the signed distance

        return ((T)phi <= 0);
    }

    TV normal(const TV& X_in) const override {
        int tv_dim = X_in.size();
        Eigen::Matrix<T, 3, 1> X;
        X.setZero();
        for (int d = 0; d < tv_dim; d++)
            X(d) = (X_in(d) - offset(d)) / scale;

        openvdb::tools::GridSampler<GradientTreeT, openvdb::tools::BoxSampler> interpolator(grad_phi->constTree(), grad_phi->transform());
        openvdb::math::Vec3<T> P(X(0), X(1), X(2));
        auto grad_phi = interpolator.wsSample(P);
        TV result;
        for (int d = 0; d < tv_dim; d++)
            result(d) = grad_phi(d);
        T norm = result.norm();
        if (norm != 0)
            return result / norm;
        else
            return TV::Zero();
    }

    void bounds(TV& min_bbox, TV& max_bbox) const {
        int tv_dim = min_bbox.size();
        min_bbox.setZero();
        max_bbox.setZero();
        auto wmin = grid->indexToWorld(bbox.min());
        auto wmax = grid->indexToWorld(bbox.max());

        for (int d = 0; d < tv_dim; d++) {
            min_bbox(d) = (T)wmin(d) * scale + offset(d);
            max_bbox(d) = (T)wmax(d) * scale + offset(d);
        }
    }

    // void test(ObjectVdb& other, float blend_weight = 0.5f) const {
    //     // Note: LevelSetMorphing modifies the source grid (*grid) in-place. 
    //     // If you need to keep the original, make a copy using grid->deepCopy() first.
    //     openvdb::tools::LevelSetMorphing<GridT> morpher(*grid, *other.grid);
        
    //     // Morph the grid over a time interval.
    //     // 0.0 = Source shape, 1.0 = Target shape.
    //     morpher.advect(0.0, blend_weight);
    // }

}; // End class ObjectVdb

#endif // VDB_OBJ_H
