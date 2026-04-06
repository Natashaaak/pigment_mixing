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

    // Pomocná SDF funkce (vzdálenost k povrchu v lokálním prostoru)
    T sdSpatula(const Eigen::Vector3f& p) const {
        // Půl-rozměry lichoběžníku
        float b1 = (float)halfWidth;       // Spodní šířka
        float b2 = (float)halfWidth * 0.6f; // Horní šířka
        float he = (float)halfLength;       // Polovina výšky (Z)

        // 1. Přesný 2D SDF lichoběžníku (Centrovaný kolem Z=0)
        Eigen::Vector2f p2d(std::abs(p.x()), p.z());
        Eigen::Vector2f k1(b2, he);
        Eigen::Vector2f k2(b2 - b1, 2.0f * he);
        
        Eigen::Vector2f ca(p2d.x() - std::min(p2d.x(), (p2d.y() < 0.0f) ? b1 : b2), std::abs(p2d.y()) - he);
        
        float dot_val = (k1 - p2d).dot(k2) / k2.dot(k2);
        float clamped_val = std::max(0.0f, std::min(1.0f, dot_val));
        Eigen::Vector2f cb = p2d - k1 + k2 * clamped_val;
        
        float s = (cb.x() < 0.0f && ca.y() < 0.0f) ? -1.0f : 1.0f;
        float d_2d = s * std::sqrt(std::min(ca.dot(ca), cb.dot(cb)));

        // 2. Korektní 3D extruze (Tloušťka v ose Y) pro sphere tracing
        float d_y = std::abs(p.y()) - (float)halfThickness;
        Eigen::Vector2f w(d_2d, d_y);
        
        float max_w_xy = std::max(w.x(), w.y());
        float ext_x = std::max(w.x(), 0.0f);
        float ext_y = std::max(w.y(), 0.0f);
        float ext_length = std::sqrt(ext_x * ext_x + ext_y * ext_y);
        
        return (T)(std::min(max_w_xy, 0.0f) + ext_length);
    }

    bool inside(const TV& X_in) const override {

        // Definujeme 3D vektor se správným typem T (např. float)
        Eigen::Vector4f worldPos;
#ifdef THREEDIM
        worldPos << (float)X_in(0), (float)X_in(1), (float)X_in(2), 1.0f;
#else
        worldPos << (float)X_in(0), 0.0f, (float)X_in(1), 1.0f; // 2D X a Z
#endif

        Eigen::Vector3f localPos = (invTransform.matrix() * worldPos).head<3>();

        return sdSpatula(localPos) < (T)0.0;
    }

    TV normal(const TV& X_in) const override {
        Eigen::Vector4f worldPos;
    #ifdef THREEDIM
        worldPos << (float)X_in(0), (float)X_in(1), (float)X_in(2), 1.0f;
    #else
        worldPos << (float)X_in(0), 0.0f, (float)X_in(1), 1.0f;
    #endif

        Eigen::Vector3f p = (invTransform * worldPos).head<3>();

        // Numerický gradient (centrální diference)
        T e = (T)0.0001;
        Eigen::Matrix<T, 3, 1> n_local;
        n_local << 
            sdSpatula(p + Eigen::Vector3f(e, 0, 0)) - sdSpatula(p - Eigen::Vector3f(e, 0, 0)),
            sdSpatula(p + Eigen::Vector3f(0, e, 0)) - sdSpatula(p - Eigen::Vector3f(0, e, 0)),
            sdSpatula(p + Eigen::Vector3f(0, 0, e)) - sdSpatula(p - Eigen::Vector3f(0, 0, e));
        
        n_local.normalize();

        // Transformace normály zpět do world space (pouze rotace)
        // worldNormal = (ModelMatrix^-1)^T * localNormal
        TV worldNormal3d = (transform.linear().inverse().transpose() * n_local).normalized();

    #ifdef THREEDIM
        return worldNormal3d;
    #else
        // Ve 2D vracíme osy X a Z (odpovídající vaší simulaci)
        return TV(worldNormal3d(0), worldNormal3d(2));
    #endif
    }

    void updateTransform(const Eigen::Transform<T, 3, Eigen::Affine>& newTransform) {
        transform = newTransform;
        invTransform = transform.inverse();
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