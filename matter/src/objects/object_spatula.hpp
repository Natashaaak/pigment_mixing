#ifndef OBJECTSPATULA_HPP
#define OBJECTSPATULA_HPP

#include "object_general.hpp"
#include <Eigen/Geometry>

struct FrameData {
    float pos[3];   // x, y, z (OpenGL Y-up)
    float quat[4];  // quaternion x, y, z, w
};

class ObjectSpatula : public ObjectGeneral{
public:
    ~ObjectSpatula(){}

    ObjectSpatula(BC bc_in = BC::NoSlip, T friction_in = 0.0, std::string name_in = "", std::string anim_path = "../matter/levelsets/spatula_motion_squish.bin") : ObjectGeneral(bc_in, friction_in, name_in) {
        transform.setIdentity();
        invTransform.setIdentity();

        // halfWidth = 0.15; // 15 cm
        // halfLength = 0.1; // 10 cm
        halfWidth = 0.25; // 55 cm
        halfLength = 0.4; // 50 cm
        halfThickness = 0.05; // 1 mm

        animation_path = anim_path;
        loadAnimation(animation_path);

        // set initial position and rotation based on first frame of animation
        if (!animation_data.empty()) {
            const auto& f0 = animation_data[0];
            Eigen::Vector3f p0(f0.pos[0], f0.pos[1], f0.pos[2]);
            Eigen::Quaternionf q0(f0.quat[3], f0.quat[0], f0.quat[1], f0.quat[2]);
            transform.translate(p0.template cast<T>());
            transform.rotate(q0.template cast<T>());
            invTransform = transform.inverse();
        }
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

    TV velocity(const TV& X_in) const override {
        // 1. Translační složka (lineární posun)
        TV v_total = TV::Zero();
        v_total(0) = vx_object;
        v_total(1) = vy_object;
#ifdef THREEDIM
        v_total(2) = vz_object;
#endif

        // 2. Rotační složka
        // Střed špachtle ve světových souřadnicích (sloupec 3 v matici 4x4)
        TV center = transform.translation(); 
        TV r = X_in - center; // Vektor od středu k bodu kolize

#ifdef THREEDIM
        // v_rot = omega x r
        TV v_rot = angularVelocity.cross(r);
        v_total += v_rot;
#else
        // Ve 2D je rotace jen skalár (kolem osy kolmé k rovině)
        // v_rot = (-omega * r.y, omega * r.x)
        TV v_rot;
        v_rot << -angularVelocity2D * r(1), angularVelocity2D * r(0);
        v_total += v_rot;
#endif
        return v_total;
    }

    void move(T dt) {
        // 1. Výpočet aktuálního času simulace
        // Předpokládáme, že si ve třídě držíš proměnnou 'currentTime', kterou inkrementuješ
        currentTime += dt;

        float floatIndex = (float)currentTime * 60.0f; // 60 FPS export
        int i0 = static_cast<int>(std::floor(floatIndex));
        int i1 = i0 + 1;

        // Kontrola mezí animace
        if (i1 >= animation_data.size()) {
            // Zastavit na konci nebo smyčkovat (zde zastavíme)
            vx_object = vy_object = vz_object = 0;
            angularVelocity = TV::Zero();
            return;
        }

        float t = floatIndex - (float)i0;

        // 2. Načtení sousedních transformací
        const auto& f0 = animation_data[i0];
        const auto& f1 = animation_data[i1];

        // 3. Interpolace pozice (Lerp)
        Eigen::Vector3f p0(f0.pos[0], f0.pos[1], f0.pos[2]);
        Eigen::Vector3f p1(f1.pos[0], f1.pos[1], f1.pos[2]);
        Eigen::Vector3f p_interp = p0 + t * (p1 - p0);

        // 4. Interpolace rotace (Slerp)
        Eigen::Quaternionf q0(f0.quat[3], f0.quat[0], f0.quat[1], f0.quat[2]); // w, x, y, z
        Eigen::Quaternionf q1(f1.quat[3], f1.quat[0], f1.quat[1], f1.quat[2]);
        Eigen::Quaternionf q_interp = q0.slerp(t, q1);

        // 5. AKTUALIZACE RYCHLOSTÍ (Klíčové pro MPM stabilitu)
        // Rychlost v = (p_new - p_old) / dt
        Eigen::Vector3f oldPos = transform.translation().template cast<float>();
        Eigen::Vector3f velocity3f = (p_interp - oldPos) / (float)dt;
        
        vx_object = (T)velocity3f.x();
        vy_object = (T)velocity3f.y();
    #ifdef THREEDIM
        vz_object = (T)velocity3f.z();
    #endif

        // Výpočet úhlové rychlosti (omega) z rozdílu kvaternionů
        // dq = q1 * q0_inv -> omega = 2 * log(dq) / dt
        Eigen::Quaternionf dq = q_interp * Eigen::Quaternionf(transform.linear().template cast<float>()).conjugate();
        Eigen::AngleAxisf aa(dq);
        Eigen::Vector3f omega = (aa.axis() * aa.angle()) / (float)dt;

    #ifdef THREEDIM
        angularVelocity = omega.template cast<T>();
    #else
        angularVelocity2D = (T)omega.y(); // V 2D simulaci osa rotace obvykle kolmá na rovinu
    #endif

        // 6. FINÁLNÍ TRANSFORMAČNÍ MATICE
        transform.setIdentity();
        transform.translate(p_interp.template cast<T>());
        transform.rotate(q_interp.template cast<T>());

        invTransform = transform.inverse();
    }

    void loadAnimation(const std::string& path) {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) {
            debug("Failed to open animation file: ", path.c_str());
            return;
        }

        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<FrameData> buffer(size / sizeof(FrameData));
        if (file.read(reinterpret_cast<char*>(buffer.data()), size)) {
            animation_data = buffer;
            debug("Loaded animation with ", animation_data.size(), " frames");
        }
    }

    Eigen::Transform<T, 3, Eigen::Affine> transform;
    Eigen::Transform<T, 3, Eigen::Affine> invTransform;

    std::vector<FrameData> animation_data;
    std::string animation_path;
    const float animation_fps = 60.0f;
    T currentTime = 0; 


    T halfWidth;
    T halfLength;
    T halfThickness;

    T vx_object;
    T vy_object;
    #ifdef THREEDIM
        T vz_object;
    #endif

    // angular velocity in radians per second
    #ifdef THREEDIM
    TV angularVelocity = TV::Zero(); 
#else
    T angularVelocity2D = 0;
#endif
};

#endif  // OBJECTSPATULA_HPP