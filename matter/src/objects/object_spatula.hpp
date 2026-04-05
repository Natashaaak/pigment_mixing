#ifndef OBJECTSPATULA_HPP
#define OBJECTSPATULA_HPP

#include "object_general.hpp"
#include <Eigen/Geometry>

class ObjectSpatula : public ObjectGeneral{
public:
    ~ObjectSpatula(){}

    ObjectSpatula(BC bc_in = BC::NoSlip, T friction_in = 0.0, std::string name_in = "") : ObjectGeneral(bc_in, friction_in, name_in) {
        transform.setIdentity();
        invTransform.setIdentity();

        // halfWidth = 0.15; // 15 cm
        // halfLength = 0.1; // 10 cm
        halfWidth = 0.25; // 55 cm
        halfLength = 0.4; // 50 cm
        halfThickness = 0.015; // 1 mm
    }

    bool inside(const TV& X_in) const override {

        // Definujeme 3D vektor se správným typem T (např. float)
        Eigen::Vector4f worldPos;
#ifdef THREEDIM
        worldPos << (float)X_in(0), (float)X_in(1), (float)X_in(2), 1.0f;
#else
        worldPos << (float)X_in(0), 0.0f, (float)X_in(1), 1.0f; // 2D X a Z
#endif

        Eigen::Vector4f localPos = invTransform.matrix() * worldPos;
        Eigen::Vector3f localXi = localPos.head<3>();

    #ifdef THREEDIM
        // 3D test: Plocha v rovině XZ, tloušťka v ose Y
        if (std::abs(localXi(0)) < halfWidth && std::abs(localXi(2)) < halfLength) {
            return std::abs(localXi(1)) < halfThickness; 
        }
    #else
        // 2D test: Úsečka v ose X, tloušťka v ose Y
        if (std::abs(localXi(0)) < halfWidth) {
            return std::abs(localXi(1)) < halfThickness;
        }
    #endif
        return false; 
    }

    TV normal(const TV& X_in) const override {
        // Definujeme 3D vektor se správným typem T (např. float)
        Eigen::Vector4f worldPos;
#ifdef THREEDIM
        worldPos << (float)X_in(0), (float)X_in(1), (float)X_in(2), 1.0f;
#else
        worldPos << (float)X_in(0), 0.0f, (float)X_in(1), 1.0f; // 2D X a Z
#endif

        Eigen::Vector4f localPos = invTransform.matrix() * worldPos;
        Eigen::Vector3f localXi = localPos.head<3>();
        
        // Find which face is closest to return the correct outward normal
        TV localNormal = TV::Zero();
        
        T dist_x = std::abs((T)localXi(0)) - halfWidth;
        T dist_y = std::abs((T)localXi(1)) - halfThickness;
        
#ifdef THREEDIM
        T dist_z = std::abs((T)localXi(2)) - halfLength;
        if (dist_x > dist_y && dist_x > dist_z) {
            localNormal(0) = (localXi(0) > 0) ? (T)1 : (T)-1;
        } else if (dist_z > dist_y && dist_z > dist_x) {
            localNormal(2) = (localXi(2) > 0) ? (T)1 : (T)-1;
        } else {
            localNormal(1) = (localXi(1) > 0) ? (T)1 : (T)-1;
        }
#else
        if (dist_x > dist_y) {
            localNormal(0) = (localXi(0) > 0) ? (T)1 : (T)-1;
        } else {
            localNormal(1) = (localXi(1) > 0) ? (T)1 : (T)-1;
        }
#endif
        
        // Transformace normály zpět do world space (pouze rotace)
        TV worldNormal = (transform.linear().inverse().transpose() * localNormal).normalized();
        
        return worldNormal;
    }

    void updateTransform(const Eigen::Transform<T, 3, Eigen::Affine>& newTransform) {
        transform = newTransform;
        invTransform = transform.inverse();

        // print both matrices for debugging
        std::cout << "Updated Spatula Transform:\n" << transform.matrix() << std::endl;
        std::cout << "Updated Spatula Inverse Transform:\n" << invTransform.matrix() << std::endl;
    }

    Eigen::Transform<T, 3, Eigen::Affine> transform;
    Eigen::Transform<T, 3, Eigen::Affine> invTransform;

    T halfWidth;
    T halfLength;
    T halfThickness;

    // T vx_object;
    // T vy_object;
    // #ifdef THREEDIM
    //     T vz_object;
    // #endif

    // T vx_object_original;
    // T vy_object_original;
    // #ifdef THREEDIM
    //     T vz_object_original;
    // #endif
};

#endif  // OBJECTSPATULA_HPP