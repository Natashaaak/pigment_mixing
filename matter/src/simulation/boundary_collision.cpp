// Copyright (C) 2024 Lars Blatny. Released under GPL-3.0 license.

#include "simulation.hpp"

void Simulation::boundaryCollision(int index, TV Xi, TV& vi){
    // Make a copy
    TV vi_orig = vi;

    // New positions
    Xi += dt * vi; // recall Xi was passed by value

    bool influenced_by_spatula = false;

    // --- PŘEDSTIŽNÁ DETEKCE ŠPACHTLE ---
    // Vytvoříme detekční zónu (např. 2 * dx) kolem špachtle.
    // Pokud je částice (uzel) v této zóně, nastavíme influenced_by_spatula = true.
    if (spatula_ptr != nullptr) {
        Eigen::Vector4f worldPos;
#ifdef THREEDIM
        worldPos << (float)Xi(0), (float)Xi(1), (float)Xi(2), 1.0f;
#else
        worldPos << (float)Xi(0), 0.0f, (float)Xi(1), 1.0f;
#endif
        Eigen::Vector3f localPos = (spatula_ptr->invTransform.matrix() * worldPos).head<3>();
        if (spatula_ptr->sdSpatula(localPos) < (T)(2.0 * dx)) {
            influenced_by_spatula = true;
        }
    }

    for(auto& obj : objects) {
        bool colliding = obj->inside(Xi);
        if (colliding) {
            TV v_obj = obj->velocity(Xi);
            TV v_rel = vi_orig - v_obj;

            if (obj->bc == BC::NoSlip) {
                v_rel.setZero();
            } // end BC::NoSlip

            else if (obj->bc == BC::SlipFree) {
                TV n = obj->normal(Xi);
                T dot = v_rel.dot(n);
                if (dot < 0){ // if moving towards object

                    T friction = obj->friction;
                    if (use_mibf)
                        friction = grid.friction[index];

                    TV v_tang = v_rel - dot * n;
                    if (friction > 0){
                        if( -dot * friction < v_tang.norm() )
                            v_rel = v_tang + v_tang.normalized() * dot * friction;
                        else
                            v_rel.setZero();
                    } else{
                        v_rel = v_tang;
                    } // end non-zero friction
                } // end moving towards object

            } // end BC::SlipFree

            else if (obj->bc == BC::SlipStick) {
                TV n = obj->normal(Xi);
                T dot = v_rel.dot(n);

                v_rel = v_rel - dot * n; //  v_rel is now v_tang

                if (dot < 0){ // if moving towards object

                    T friction = obj->friction;
                    if (use_mibf)
                        friction = grid.friction[index];

                    if (friction > 0){
                        if( -dot * friction < v_rel.norm() )
                            v_rel = v_rel + v_rel.normalized() * dot * friction;
                        else
                            v_rel.setZero();
                    } // end non-zero friction
                } // end moving towards object

            } // end BC::SlipStick

            else {
                debug("INVALID BOUNDARY CONDITION!!!");
                exit = 1;
                return;
            }

            vi = v_rel + v_obj;

            // Zaznamenáme, že špachtle ovlivnila částici a má tak přednost
            if (obj.get() == spatula_ptr) {
                influenced_by_spatula = true;

                // Pokud je částice v kritické zóně u dna (hrozí film)
                if (Xi(1) < 0.8 * dx) {
                    TV n = obj->normal(Xi);
                    
                    // Univerzální výpočet tečného vektoru směřujícího NAHORU (pro 2D i 3D)
                    TV up = TV::Zero();
                    up(1) = 1.0; // Globální směr nahoru
                    TV t = up - up.dot(n) * n; // Projekce osy Y do tečné roviny
                    if (t.norm() > 1e-6) t.normalize();
                    else t = TV::Zero(); // Pro jistotu, kdyby byla normála přesně (0,1,0)

                    // Rychlost: rychlost špachtle + "skluz" směrem nahoru
                    // 0.2 * v_obj.norm() určí, jak ochotně barva po špachtli klouže
                    vi = v_obj + t * (0.2 * v_obj.norm());
                }
            }

            // update velocity copy before next iteration
            vi_orig = vi; // Comment this line to enforce ordering of objects (i.e., use only last object in list)

        } // end if colliding

    } // end iterator over general objects


#ifdef THREEDIM

    for (auto& obj : plates) {
        bool colliding = obj->inside(Xi);
        if (colliding) {
            // Špachtle má PŘEDNOST: pokud částice koliduje se špachtlí, ignorujeme tření podložky (aby po ní barva klouzala),
            // ale musíme zabránit propadnutí částic dolů skrz dno.
            if (influenced_by_spatula && obj->plate_type == PlateType::bottom) {
                if (vi(1) < obj->vy_object) {
                    vi(1) = obj->vy_object;
                    vi_orig = vi;
                }
                continue;
            }

            T vx_rel = vi_orig(0) - obj->vx_object;
            T vy_rel = vi_orig(1) - obj->vy_object;
            T vz_rel = vi_orig(2) - obj->vz_object;

            if (obj->bc == BC::NoSlip) {
                vx_rel = 0;
                vy_rel = 0;
                vz_rel = 0;
            } // end BC::NoSlip

            else if (obj->bc == BC::SlipStick) {

                T friction = obj->friction;
                    if (use_mibf)
                        friction = grid.friction[index];

                if (obj->plate_type == PlateType::top || obj->plate_type == PlateType::bottom){
                    // tangential velocity is the (x,z) components

                    T vel_t      = std::sqrt(vx_rel*vx_rel + vz_rel*vz_rel);
                    T fric_vel_n = friction * std::abs(vy_rel);

                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( vel_t <= fric_vel_n ){
                        vx_rel = 0; // tangential component also set to zero
                        vz_rel = 0;
                    }
                    else { // just reduce tangential component
                        T scale = (vel_t - fric_vel_n) / vel_t;
                        vx_rel *= scale;
                        vz_rel *= scale;
                    }

                    // normal component (y) must be set to zero
                    vy_rel = 0;

                } // end PlateType::top and PlateType::bottom plate
                else if (obj->plate_type == PlateType::left || obj->plate_type == PlateType::right){
                    // tangential velocity is the (y,z) components

                    T vel_t      = std::sqrt(vy_rel*vy_rel + vz_rel*vz_rel);
                    T fric_vel_n = friction * std::abs(vx_rel);

                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( vel_t <= fric_vel_n ){
                        vy_rel = 0; // tangential component also set to zero
                        vz_rel = 0;
                    }
                    else { // just reduce tangential component
                        T scale = (vel_t - fric_vel_n) / vel_t;
                        vy_rel *= scale;
                        vz_rel *= scale;
                    }

                    // normal component (x) must be set to zero
                    vx_rel = 0;

                } // end PlateType::left or PlateType::right plate
                else if (obj->plate_type == PlateType::front || obj->plate_type == PlateType::back){
                    // tangential velocity is the (x,y) components

                    T vel_t      = std::sqrt(vx_rel*vx_rel + vy_rel*vy_rel);
                    T fric_vel_n = friction * std::abs(vz_rel);

                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( vel_t <= fric_vel_n ){
                        vx_rel = 0; // tangential component also set to zero
                        vy_rel = 0;
                    }
                    else { // just reduce tangential component
                        T scale = (vel_t - fric_vel_n) / vel_t;
                        vx_rel *= scale;
                        vy_rel *= scale;
                    }

                    // normal component (z) must be set to zero
                    vz_rel = 0;

                } // end PlateType::front or PlateType::back plate
            } // end BC::SlipStick



            else if (obj->bc == BC::SlipFree) {

                T friction = obj->friction;
                    if (use_mibf)
                        friction = grid.friction[index];

                if ((obj->plate_type == PlateType::top && vy_rel > 0) || (obj->plate_type == PlateType::bottom && vy_rel < 0)){
                    // tangential velocity is the (x,z) components
                    T vel_t      = std::sqrt(vx_rel*vx_rel + vz_rel*vz_rel);
                    T fric_vel_n = friction * std::abs(vy_rel);

                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( vel_t <= fric_vel_n ){
                        vx_rel = 0; // tangential component also set to zero
                        vz_rel = 0;
                    }
                    else { // just reduce tangential component
                        T scale = (vel_t - fric_vel_n) / vel_t;
                        vx_rel *= scale;
                        vz_rel *= scale;
                    }

                    // normal component (y) must be set to zero
                    vy_rel = 0;

                } // end PlateType::top and PlateType::bottom plate
                else if ((obj->plate_type == PlateType::left && vx_rel < 0) || (obj->plate_type == PlateType::right && vx_rel > 0)){
                    // tangential velocity is the (y,z) components
                    T vel_t      = std::sqrt(vy_rel*vy_rel + vz_rel*vz_rel);
                    T fric_vel_n = friction * std::abs(vx_rel);

                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( vel_t <= fric_vel_n ){
                        vy_rel = 0; // tangential component also set to zero
                        vz_rel = 0;
                    }
                    else { // just reduce tangential component
                        T scale = (vel_t - fric_vel_n) / vel_t;
                        vy_rel *= scale;
                        vz_rel *= scale;
                    }

                    // normal component (x) must be set to zero
                    vx_rel = 0;

                } // end PlateType::left or PlateType::right plate
                else if ((obj->plate_type == PlateType::front && vz_rel > 0) || (obj->plate_type == PlateType::back && vz_rel < 0 )){
                    // tangential velocity is the (x,y) components
                    T vel_t      = std::sqrt(vx_rel*vx_rel + vy_rel*vy_rel);
                    T fric_vel_n = friction * std::abs(vz_rel);

                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( vel_t <= fric_vel_n ){
                        vx_rel = 0; // tangential component also set to zero
                        vy_rel = 0;
                    }
                    else { // just reduce tangential component
                        T scale = (vel_t - fric_vel_n) / vel_t;
                        vx_rel *= scale;
                        vy_rel *= scale;
                    }

                    // normal component (z) must be set to zero
                    vz_rel = 0;

                } // end PlateType::front or PlateType::back plate
            } // end BC::SlipFree



            else {
                debug("INVALID BOUNDARY CONDITION!!!");
                exit = 1;
                return;
            }

            vi(0) = vx_rel + obj->vx_object;
            vi(1) = vy_rel + obj->vy_object;
            vi(2) = vz_rel + obj->vz_object;

            // update velocity copy before next iteration
            vi_orig = vi; // Comment this line to enforce ordering of objects (i.e., use only last object in list)

        } // end if colliding

    } // end iterator over 3D plate objects


#else // TWODIM


    for (auto& obj : plates) {
        bool colliding = obj->inside(Xi);
        if (colliding) {
            // Špachtle má PŘEDNOST: pokud částice koliduje se špachtlí, ignorujeme tření podložky,
            // ale musíme zabránit propadnutí částic dolů.
            if (influenced_by_spatula && obj->plate_type == PlateType::bottom) {
                if (vi(1) < obj->vy_object) {
                    vi(1) = obj->vy_object;
                    vi_orig = vi;
                }
                continue;
            }

            T vx_rel = vi_orig(0) - obj->vx_object;
            T vy_rel = vi_orig(1) - obj->vy_object;

            if (obj->bc == BC::NoSlip) {
                vx_rel = 0;
                vy_rel = 0;
            } // end BC::NoSlip

            else if (obj->bc == BC::SlipStick) {

                T friction = obj->friction;
                    if (use_mibf)
                        friction = grid.friction[index];

                if (obj->plate_type == PlateType::top || obj->plate_type == PlateType::bottom){
                    // tangential velocity is the (x) component and must be changed
                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( std::abs(vx_rel) < friction * std::abs(vy_rel) ){
                        vx_rel = 0; // tangential component also set to zero
                    }
                    else { // just reduce tangential component
                        if (vx_rel > 0)
                            vx_rel -= friction * std::abs(vy_rel);
                        else
                            vx_rel += friction * std::abs(vy_rel);
                    }

                    // normal component (y) must be set to zero
                    vy_rel = 0;

                } // end PlateType::top and PlateType::bottom plate
                else if (obj->plate_type == PlateType::left || obj->plate_type == PlateType::right){
                    // tangential velocity is the (y) component and must be changed

                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( std::abs(vy_rel) < friction * std::abs(vx_rel) ){
                        vy_rel = 0; // tangential component also set to zero
                    }
                    else { // just reduce tangential component
                        if (vy_rel > 0)
                            vy_rel -= friction * std::abs(vx_rel);
                        else
                            vy_rel += friction * std::abs(vx_rel);
                    }

                    // normal component (x) must be set to zero
                    vx_rel = 0;

                } // end PlateType::left or PlateType::right plate

            } // end BC::SlipStick



            else if (obj->bc == BC::SlipFree) {

                T friction = obj->friction;
                    if (use_mibf)
                        friction = grid.friction[index];

                if ((obj->plate_type == PlateType::top && vy_rel > 0) || (obj->plate_type == PlateType::bottom && vy_rel < 0)){
                    // tangential velocity is the (x) component and must be changed
                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( std::abs(vx_rel) < friction * std::abs(vy_rel) ){
                        vx_rel = 0; // tangential component also set to zero
                    }
                    else { // just reduce tangential component
                        if (vx_rel > 0)
                            vx_rel -= friction * std::abs(vy_rel);
                        else
                            vx_rel += friction * std::abs(vy_rel);
                    }

                    // normal component (y) must be set to zero
                    vy_rel = 0;

                } // end PlateType::top and PlateType::bottom plate
                else if ((obj->plate_type == PlateType::left && vx_rel < 0) || (obj->plate_type == PlateType::right && vx_rel > 0)){
                    // tangential velocity is the (y) component and must be changed
                    if (friction == 0){
                        // Do nothing
                    }
                    else if ( std::abs(vy_rel) < friction * std::abs(vx_rel) ){
                        vy_rel = 0; // tangential component also set to zero
                    }
                    else { // just reduce tangential component
                        if (vy_rel > 0)
                            vy_rel -= friction * std::abs(vx_rel);
                        else
                            vy_rel += friction * std::abs(vx_rel);
                    }

                    // normal component (x) must be set to zero
                    vx_rel = 0;

                } // end PlateType::left or PlateType::right plate
            } // end BC::SlipFree



            else {
                debug("INVALID BOUNDARY CONDITION!!!");
                exit = 1;
                return;
            }

            vi(0) = vx_rel + obj->vx_object;
            vi(1) = vy_rel + obj->vy_object;

            // update velocity copy before next iteration 
            vi_orig = vi; // Comment this line to enforce ordering of objects (i.e., use only last object in list)

        } // end if colliding

    } // end iterator over 2D plate objects

#endif

} // end boundaryCollision
