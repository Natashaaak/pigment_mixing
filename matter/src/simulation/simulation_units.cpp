#include "simulation.hpp"
#include "../sampling/sampling_particles.hpp"
#include "../sampling/sampling_particles_vdb.hpp"
#include <random>
#include "../objects/object_vdb.hpp"
#include "../objects/object_spatula.hpp"
#include "../../deps/json.hpp"

void Simulation::initializeBasic(std::string name){
    std::cout << "-----------------------------------------------------------------------------------" << std::endl;
    std::cout << "    88b           d88                                                              " << std::endl;
    std::cout << "    888b         d888                  aa          aa                              " << std::endl;
    std::cout << "    88 8b       d8 88                  88          88                              " << std::endl;
    std::cout << "    88  8b     d8  88   adPPYYba   aaaa88aaaa  aaaa88aaaa    adPPYba   8b dPPYba   " << std::endl;
    std::cout << "    88   8b   d8   88  aa      Y8  aaaa88aaaa  8888888888  a8P     88  88P     Y8  " << std::endl;
    std::cout << "    88    8b d8    88   adPPPPP88      88          88      adPPPPP88   88          " << std::endl;
    std::cout << "    88     888     88  88      88      aa          aa      a8b         88          " << std::endl;
    std::cout << "    88      8      88    adPPYba                            adPPYba    88          " << std::endl;
    std::cout << "-----------------------------------------------------------------------------------" << std::endl;

    save_grid = false;
    save_sim = false;
    sim_name = name;

    is_initialized = true;
}

void Simulation::setupScene(const float fps_value, const std::vector<float>& colorRatios, const std::vector<Eigen::Matrix<float, 7, 1>>& pigments, const std::string& spatula_anim_path){
    openvdb::initialize();

    // Read JSON configuration
    std::ifstream config_file("../matter/pigment_config.json");
    if (config_file.is_open()) {
        try {
            nlohmann::json data = nlohmann::json::parse(config_file);
            pigment_D_max = data.value("D_max", 10e-2f);
            pigment_D_edge0 = data.value("D_edge0", 0.5f);
            pigment_D_edge1 = data.value("D_edge1", 0.8f);
            start_boost_time = data.value("start_boost_time", 20.0f);
            end_boost_time = data.value("end_boost_time", 40.0f);
            boost_factor = data.value("boost_factor", 2.0f);
            debug("Loaded pigment config successfully.");
        } catch (const nlohmann::json::parse_error& e) {
            debug("Parse error in pigment_config.json: ", e.what());
        }
    } else {
        debug("Could not open pigment_config.json, using default parameters.");
    }

    // Read Simulation Configuration
    std::ifstream sim_config_file("../matter/sim_config.json");
    if (sim_config_file.is_open()) {
        try {
            nlohmann::json data = nlohmann::json::parse(sim_config_file);
            cfl          = data.value("cfl", 0.2f);
            flip_ratio   = data.value("flip_ratio", -0.8f);
            E            = data.value("E", 500000.0f);
            nu           = data.value("nu", 0.3f);
            rho          = data.value("rho", 1000.0f);
            T angle      = data.value("friction_angle", 5.0f);
            M            = std::tan(angle * M_PI / 180.0);
            q_cohesion   = data.value("q_cohesion", 500.0f);
            perzyna_exp  = data.value("perzyna_exp", 1.0f);
            perzyna_visc = data.value("perzyna_visc", 0.1f);
            debug("Loaded simulation config successfully.");
        } catch (const nlohmann::json::parse_error& e) {
            debug("Parse error in sim_config.json: ", e.what());
        }
    } else {
        debug("Could not open sim_config.json, using default parameters.");
        cfl = 0.2;
        flip_ratio = -0.8;
        E = 5e5;
        nu = 0.3;
        rho = 1000;
        M = std::tan(5 * M_PI / 180.0);
        q_cohesion = 500;
        perzyna_exp = 1.0;
        perzyna_visc = 0.1;
    }

    reduce_verbose = true;
    end_frame = 1800;   // last frame to simulate
    fps = fps_value;    // frames per second
    n_threads = 12;      // number of threads in parallel

    // pbc = true;

    // INITILIZE ELASTICITY
    elastic_model = ElasticModel::Hencky;

    ////// GRAVITY ANGLE [default: gravity is 0]
    T theta_deg = 0; // angle in degrees of gravity vector
    T theta = theta_deg * M_PI / 180;
    gravity = TV::Zero(); //
    gravity[0] = +9.81 * std::sin(theta);
    gravity[1] = -9.81 * std::cos(theta);

    ////// INITIAL PARTICLE POSITIONS
    Lx = 1;
    Ly = 1;
    T k_rad = 0.01;
    #ifdef THREEDIM
        Lz = 0.3;
    #endif

    // Restrict runaway particles
    use_particle_boundaries = true;
    particle_boundary_min = -2.0 * TV::Ones(); 
    particle_boundary_max =  2.0 * TV::Ones();
 
    grid_reference_point = TV::Zero();

    // print color ratios
    debug("Color Ratios:");
    for (size_t i = 0; i < colorRatios.size(); ++i) {
        debug("  Color ", i, ": ", colorRatios[i]);
    } 

    blobs(colorRatios, pigments);

    ////// FLOOR OBJECTS
    plates.push_back(std::make_unique<ObjectPlate>(0, PlateType::bottom, BC::SlipStick, 1.0)); 

    ////// SPATULA
    auto spatula = std::make_unique<ObjectSpatula>(BC::SlipFree, 0.0, "hehe", spatula_anim_path);
    spatula_ptr = spatula.get();
    objects.push_back(std::move(spatula));

    debug("Init particles done\n");

    ////// PLASTICITY
    plastic_model = PlasticModel::DPVisc; // Perzyna model with Drucker_Prager yield surface

    use_pradhana = true; // Supress unwanted volume expansion in Drucker-Prager models
    q_prefac = 1.0 / std::sqrt(2.0); // [default: sqrt(1/2)] Prefactor in def. of q, here q = sqrt(1/2 * s:s)
}

void Simulation::prepareSimulation(){
    if (elastic_model != ElasticModel::Hencky && plastic_model != PlasticModel::NoPlasticity){
        debug("This plastic model is only compatible with Hencky's elasticity model");
        debug("Please use: elastic_model = Hencky");
        return;
    }

    if (dim == 3){
        debug("This is a 3D simulation.");
    }
    else if (dim == 2){
        debug("This is a 2D simulation.");
    }
    else{
        debug("Unsupported spline degree");
        return;
    }

     #if SPLINEDEG == 3
      apicDinverse = 3.0/(dx*dx);
      debug("Using cubic splines.");
    #elif SPLINEDEG == 2
      apicDinverse = 4.0/(dx*dx);
      debug("Using quadratic splines.");
    #elif SPLINEDEG == 1
        apicDinverse = 0; // NB not implemented
        debug("Using linear hat functions.");
    #else
        #error Unsupported spline degree
    #endif

    if (exit)
        return;

    lambda = nu * E / ( (1.0 + nu) * (1.0 - 2.0*nu) ); // first Lame parameter
    mu = E / (2.0*(1.0+nu)); // shear modulus
    K = calculateBulkModulus(); // bulk modulus
    wave_speed = std::sqrt(E/rho); // elastic wave speed

    dt_max = cfl_elastic * dx / wave_speed;

    frame_dt = 1.0 / fps;

    gravity_final = gravity;

    one_over_dx = 1.0 / dx;
    one_over_dx_square = one_over_dx * one_over_dx;

    d_prefac = 1 / q_prefac;
    e_mu_prefac = 2*q_prefac            * mu;
    f_mu_prefac = 2*q_prefac * q_prefac * mu;

    use_mibf = false;
    use_musl = true;

    fac_Q = I_ref / (grain_diameter*std::sqrt(rho_s));

    if (use_mibf){
        if (plastic_model == PlasticModel::DPMui || plastic_model == PlasticModel::MCCMui){
            std::fill(particles.muI.begin(), particles.muI.end(), mu_1);
        } 
        else {// e.g., DPVisc
            std::fill(particles.muI.begin(), particles.muI.end(), M);
        }
    }

    // debug("Number of particles: ", Np);
    // debug("Grid spacing dx:     ", dx);
    // debug("Elastic wave speed:  ", wave_speed);
    // debug("Maximum dt:          ", dt_max);
    // debug("Particle volume:     ", particle_volume);
    // debug("Particle mass:       ", particle_mass);

    // initialiye grid and boundaries solely for the purpose of first frame for raytracing
    if (pbc){
        if (current_time_step == 0)
            remeshFixed(4);
    }
    else{
        if (current_time_step == 0) {
            remeshFixedInit(2,2,2);
        } else {
            remeshFixedCont();
        }
    }

    time = 0;
    frame = 0;
    final_time = end_frame * frame_dt;
}

void Simulation::step(){
    if (!reduce_verbose){
        std::cout << "Frame: "               << frame              << std::endl;
        std::cout << "               Name: " << sim_name           << std::endl;
        std::cout << "               Step: " << current_time_step  << std::endl;
        std::cout << "               Time: " << time   << " -> "   << (frame+1)*frame_dt << std::endl;
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    advanceStep();
    time += dt;
    current_time_step++;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    runtime_total += std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    
    // T steps = current_time_step > 0 ? (T)current_time_step : 1.0;
    // std::cout << "Frame: "               << frame              << std::endl;
    // std::cout << "               Time: " << time   << " -> "   << (frame+1)*frame_dt << std::endl;
    // std::cout << "Simulation took " << runtime_total / steps << " milliseconds on average per step";
    
    // debug("Runtime P2G     = ", (runtime_p2g     * 1000.0) / steps, " milliseconds");
    // debug("Runtime G2P     = ", (runtime_g2p     * 1000.0) / steps, " milliseconds");
    // debug("Runtime Euler   = ", (runtime_euler   * 1000.0) / steps, " milliseconds");
    // debug("Runtime DefGrad = ", (runtime_defgrad * 1000.0) / steps, " milliseconds");
}

bool Simulation::frameFinished(){
    if (frame_dt*(frame+1) - time < min_dt*1.1){
        frame++;
        return true;
    }
    return false;
}

bool Simulation::isFinished() const {
    return frame >= end_frame || time >= final_time;
}

std::pair<std::vector<T>, std::vector<T>> Simulation::getGridBoundaries() const {

#ifdef THREEDIM    
    return std::make_pair(std::vector<T>{low_x, low_y, low_z}, std::vector<T>{high_x, high_y, high_z});
#else
    // two dimensions, but we still return 3D vectors for compatibility with raytracing code. The z values are set to dummy values.
    return std::make_pair(std::vector<T>{low_x, low_y, low_x}, std::vector<T>{high_x, high_y, high_x});
#endif
}
