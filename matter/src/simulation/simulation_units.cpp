#include "simulation.hpp"
#include "../sampling/sampling_particles.hpp"
#include <random>
#include "../objects/object_vdb.hpp"


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

void Simulation::setupScene(const float fps_value, const std::vector<float>& colorRatios){
    reduce_verbose = false;
    end_frame = 20;     // last frame to simulate
    fps = fps_value;    // frames per second
    n_threads = 8;      // number of threads in parallel
    cfl = 0.5;          // CFL constant, typically around 0.5
    flip_ratio = -0.95; // (A)PIC-(A)FLIP ratio in [-1,1].

    // pbc = true;

    // INITILIZE ELASTICITY
    elastic_model = ElasticModel::Hencky;
    E = 1e6;     // Young's modulus (Pa)
    nu = 0.3;   // Poisson's ratio (-)
    rho = 1000; // Density (kg/m3)

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
    sampleParticles(*this, k_rad);

    for(int p = 0; p < Np; p++){
        particles.x[p](0) -= 0.5*Lx;
        particles.x[p](1) += 0.5*dx;
    }
    grid_reference_point = TV::Zero();

    ////// ASSIGN COLORS BASED ON RATIOS
    // get random number in [0,1]
    bool setColorByRatio = false;
    if(setColorByRatio){
        std::default_random_engine generator;
        std::uniform_real_distribution<T> distribution(0.0,1.0);
        for(int p = 0; p < Np; p++){
            float rand_num = distribution(generator);
            uint8_t color_index = colorRatios.size();
            for (size_t i = 0; i < colorRatios.size(); i++){
                if (rand_num < colorRatios[i]){
                    color_index = i;
                    break;
                }
            }
            particles.color[p] = color_index;
        }
    }
    else {
        for(int p = 0; p < Np; p++){
            particles.color[p] = particles.x[p](0) < 0 ? 0 : 1; // left half is color 0, right half is color 1
        }
    }

    ////// OBJECTS AND TERRAINS
    plates.push_back(std::make_unique<ObjectPlate>(0, PlateType::bottom, BC::NoSlip)); 

    /////// Here are some examples how to use the objects derived from ObjectGeneral:
    // T friction = 0.2; 
    // objects.push_back(std::make_unique<ObjectBump>(BC::SlipFree, friction));
    // objects.push_back(std::make_unique<ObjectGate>(BC::SlipFree, friction));

    /////// Here is an example how to use ObjectVdb (uncomment includes and openvdb::initialize() above):
    openvdb::initialize();

    TV box_offset = TV::Zero();
    box_offset(0) = 0.6;
    T box_scale = 1; // Example: scale the VDB object by 5x
    objects.push_back(std::make_unique<ObjectVdb>("../matter/levelsets/box.vdb", BC::NoSlip, 0.3, box_offset, box_scale));
    spatula_vdb_ptr = dynamic_cast<ObjectVdb*>(objects.back().get()); // store a pointer to the spatula VDB object for later use in the simulation loop

    ////// PLASTICITY
    plastic_model = PlasticModel::DPVisc; // Perzyna model with Drucker_Prager yield surface

    use_pradhana = true; // Supress unwanted volume expansion in Drucker-Prager models
    q_prefac = 1.0 / std::sqrt(2.0); // [default: sqrt(1/2)] Prefactor in def. of q, here q = sqrt(1/2 * s:s)

    M = std::tan(30*M_PI/180.0); // Internal friction
    q_cohesion = 0; // Yield surface's intercection of q-axis (in Pa), 0 is the cohesionless case
    perzyna_exp = 1; // Exponent in Perzyna models
    perzyna_visc = 0; // Viscous time parameter is Perzyna models
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
}

void Simulation::step(){
    if (!reduce_verbose){
        std::cout << "Frame: "               << frame              << std::endl;
        std::cout << "               Name: " << sim_name           << std::endl;
        std::cout << "               Step: " << current_time_step  << std::endl;
        std::cout << "               Time: " << time   << " -> "   << (frame+1)*frame_dt << std::endl;
    }

    advanceStep();
    time += dt;
    current_time_step++;
}

bool Simulation::frameFinished(){
    if (frame_dt*(frame+1) - time < min_dt*1.1){
        frame++;
        return true;
    }
    return false;
}

std::pair<std::vector<T>, std::vector<T>> Simulation::getGridBoundaries() const {

#ifdef THREEDIM    
    return std::make_pair(std::vector<T>{low_x, low_y, low_z}, std::vector<T>{high_x, high_y, high_z});
#else
    // two dimensions, but we still return 3D vectors for compatibility with raytracing code. The z values are set to dummy values.
    return std::make_pair(std::vector<T>{low_x, low_y, low_x}, std::vector<T>{high_x, high_y, high_x});
#endif
}

