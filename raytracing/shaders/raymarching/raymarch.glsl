#version 450
///Ray structure
struct Ray{
    vec3 start;
    vec3 dir;
};
///State structure for 3D DDA algorithm
struct State{
    ivec3 cell;
    ivec3 move;
    vec3 delta; // is used only when moving inside the voxel
    vec3 tNext;
    float tcurr;
    float insideStepSize;
    uint currLevel;
};

struct ParticlePigment{
    float c[7];
};

layout(local_size_x = 16, local_size_y = 16) in;

layout(rgba32f, binding = 0) writeonly uniform image2D outTex;

layout(rgba32f, binding = 1) writeonly uniform image2D normalDepthTex;

layout(binding = 2) uniform sampler2D Dagg;

layout(binding = 3) uniform sampler2D Dall;

layout(binding = 4) uniform sampler2D Nscreen;

// mixbox coefficients, the same as in mixbox.cpp
const float coefs[20][3] = 
{
  {1.0*+0.07717053,1.0*+0.02826978,1.0*+0.24832992},
  {1.0*+0.95912302,1.0*+0.80256528,1.0*+0.03561839},
  {1.0*+0.74683774,1.0*+0.04868586,1.0*+0.00000000},
  {1.0*+0.99518138,1.0*+0.99978149,1.0*+0.99704802},
  {3.0*+0.01606382,3.0*+0.27787927,3.0*+0.10838459},
  {3.0*-0.22715650,3.0*+0.48702601,3.0*+0.35660312},
  {3.0*+0.09019473,3.0*-0.05108290,3.0*+0.66245019},
  {3.0*+0.26826063,3.0*+0.22364570,3.0*+0.06141500},
  {3.0*-0.11677001,3.0*+0.45951942,3.0*+1.22955000},
  {3.0*+0.35042682,3.0*+0.65938413,3.0*+0.94329691},
  {3.0*+1.07202375,3.0*+0.27090076,3.0*+0.34461513},
  {3.0*+0.92964458,3.0*+0.13855183,3.0*-0.01495765},
  {3.0*+1.00720859,3.0*+0.85124701,3.0*+0.10922038},
  {3.0*+0.98374897,3.0*+0.93733704,3.0*+0.39192814},
  {3.0*+0.94225681,3.0*+0.26644346,3.0*+0.60571754},
  {3.0*+0.99897033,3.0*+0.40864351,3.0*+0.60217887},
  {6.0*+0.31232351,6.0*+0.34171197,6.0*-0.04972666},
  {6.0*+0.42768261,6.0*+1.17238033,6.0*+0.10429229},
  {6.0*+0.68054914,6.0*-0.23401393,6.0*+0.35832587},
  {6.0*+1.00013113,6.0*+0.42592007,6.0*+0.31789917}
};

layout(std430, binding = 0) buffer spheres {
    vec4 spheresData[];
};

layout(std430, binding = 1) buffer Mbuf {
    uint M[];
};

layout(std430, binding = 2) buffer Cellsbuf {
    uvec2 cellsData[];
};

layout(std430, binding = 3) buffer idsbuf {
    uint ids[];
};

layout(std430, binding = 4) buffer M2buf {
    uint M2[];
};

layout(std430, binding = 5) buffer M4buf {
    uint M4[];
};

layout(std430, binding = 6) buffer AnisoMat {
    mat4 an[];
};

layout(std430, binding = 7) buffer Determinants {
    float dets[];
};

layout(std430, binding = 8) buffer PigmentsBuffer {
    ParticlePigment p_pigments[];
};

layout(std430, binding = 9) buffer DiffusionBuffer {
    float p_diffusion[];
};

layout(std430, binding = 10) buffer RenderGrid {
    float data[]; // 8 floats per pixel
};

///Image width
uniform int width;
///Image height
uniform int height;
///View angle
uniform float viewAngle;
///View matrix
uniform mat4 view;
///Project matrix
uniform mat4 proj;
///Inverted view matrix
uniform mat4 invView;
///Inverted proj matrix
uniform mat4 invProj;
///Maximum amount of steps until claiming to not find surface
uniform int maxStepCount;
///Start of the grid
uniform vec3 gridStart;
///Voxel size
uniform float voxelSize;
///support SPH radius
uniform float h;
///Particle size in object space
uniform float DforRIJ;
///Amount of steps required to skip to Dagg
uniform int maxSkipCount;
///Sphere radius
uniform float rad;
///Amount of cells in bdg
uniform ivec3 cellsSize;
///iso-value threshold
uniform float iso;
///Amount of steps inside a voxel marked by bdg
uniform int stepsInside;
///A parameter from normal blending
uniform float A;
///B parameter from normal blending
uniform float B;
///Max level of bdg cells grouped into texels represented as 1 axis
uniform uint maxLevel;
///If should render with anisotropic kernel
uniform bool isAni;

uniform bool showNormals;

uniform bool showDiffusion;
uniform float pigment_D_edge0;
uniform float pigment_D_edge1;
uniform float sigma_color;
uniform float sigma_spatial;

uniform bool fullRender;
uniform mat4 invSpatulaTransform;
uniform bool has_spatula;
uniform vec3 spatulaDim;
bool use_closest_color = true;
float r1 = 0.5;
float r2 = 0.2;

const float pi = 3.141592f;
const int lcs = 16; //equal to local size
float poly6 = 315.0f/(64.0f*pi); //poly6
float spiky = -45.0f/pi; //first derivative of spiky
const int MAX_INT = 2147483647;

const vec3 spatula_color = vec3(0.9f, 0.9f, 0.9f);
const vec3 floorCol = vec3(0.375f, 0.375f, 0.375f);
ivec3 cS4 = ivec3(3);
ivec3 cS2 = ivec3(1);

#include "pbr_lighting.glsl"

uniform Material fluidMat;
uniform Material spatulaMat;
uniform Material floorMat;

float ao_range_fluid = 0.30f; // Dosah stínu
float min_darkness_fluid = 0.05f; // Minimální světlost (0.0 = černá, 1.0 = žádný stín)
float ao_range_spatula = 0.15f; // Dosah stínu
float min_darkness_spatula = 0.5f; // Minimální světlost

vec3 mix_latent_to_rgb( ParticlePigment pigments) {
    float c[7] = pigments.c;
    float c00 = c[0]*c[0];
    float c11 = c[1]*c[1];
    float c22 = c[2]*c[2];
    float c33 = c[3]*c[3];
    float c01 = c[0]*c[1];
    float c02 = c[0]*c[2];

    float weights[20] = {
        c[0]*c00, c[1]*c11, c[2]*c22, c[3]*c33,
        c00*c[1], c01*c[1], c00*c[2], c02*c[2], c00*c[3], c[0]*c33,
        c11*c[2], c[1]*c22, c11*c[3], c[1]*c33, c22*c[3], c[2]*c33,
        c01*c[2], c01*c[3], c02*c[3], c[1]*c[2]*c[3]
    };

    vec3 rgb = vec3(0.0);
    for(int j=0; j<20; j++) {
        rgb += weights[j] * vec3(coefs[j][0], coefs[j][1], coefs[j][2]);
    }
    
    // Add bias (latent[4..6])
    return clamp(rgb + vec3(c[4], c[5], c[6]), 0.0, 1.0);
}

float sdCylinder(vec3 p, float r, float h) {
    vec2 d = abs(vec2(length(p.xz), p.y)) - vec2(r, h);
    return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}

vec2 rotateZ(vec2 v, float a) {
    float s = sin(a);
    float c = cos(a);
    return vec2(c * v.x - s * v.y, s * v.x + c * v.y);
}

vec2 rotateY(vec2 v, float a) {
    float s = sin(a);
    float c = cos(a);
    return vec2(c * v.x - s * v.y, s * v.x + c * v.y);
}

float sdSpatula(vec3 p, bool with_handle) {
    // Posun špachtle o 0.1 nahoru v ose Y.
    p.y -= 0.02;

    float b1 = spatulaDim.x;           // Spodní šířka (poloměr)
    float b2 = spatulaDim.x * 0.25;    // Horní šířka (poloměr)
    float he = spatulaDim.z;           // Polovina výšky (Z)
    float halfThickness = spatulaDim.y / 10.0;

    vec2 p2d = vec2(abs(p.x), p.z);

    // 1. Vzdálenost k šikmé stěně (bok lichoběžníku)
    // Definujeme úsečku od spodního rohu (b1, -he) k hornímu (b2, he)
    vec2 p1 = vec2(b1, -he);
    vec2 p2 = vec2(b2, he);
    vec2 ba = p2 - p1;
    vec2 pa = p2d - p1;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    float d_side = length(pa - ba * h); // Vzdálenost k šikmé úsečce

    // 2. Definice "vnitřku" (vzdálenost k ose X=0 v lichoběžníku)
    // Tady musíme počítat, jak daleko je bod od šikmé stěny, ale se znaménkem
    float inside_x = b1 - (b1 - b2) * (p2d.y + he) / (2.0 * he);
    float d_inside = p2d.x - inside_x;

    // 3. Rozhodnutí (Sjednocení)
    // Pokud jsme "uvnitř" (vlevo od šikmé stěny), vzdálenost je d_inside.
    // Pokud jsme "vně", vzdálenost je d_side (k šikmé stěně).
    float d_2d = (p2d.x < inside_x) ? d_inside : d_side;

    // 4. Přidání půlkruhů na koncích
    if (p2d.y > he) {
        d_2d = length(vec2(p2d.x, p2d.y - he)) - b2;
    } else if (p2d.y < -he) {
        float r_x_bot = b1;
        float r_z_bot = 1.5 * b1; // Opět protáhlost
        vec2 p_rel = vec2(p2d.x, p2d.y + he);
        float k0 = length(p_rel / vec2(r_x_bot, r_z_bot));
        float k1 = length(p_rel / (vec2(r_x_bot, r_z_bot) * vec2(r_x_bot, r_z_bot)));
        d_2d = k0 * (k0 - 1.0) / k1;
    }

    // 5. Extruze
    // float d_y = abs(p.y + 0.01) - halfThickness;
    float d_y = abs(p.y) - halfThickness;
    float d_blade = max(d_2d, d_y);

    if (!with_handle) return d_blade;

    // Válce rukojeti (využíváme inverzní rotace)
    vec3 p_handle = p;
    p_handle.xz = rotateY(p_handle.xz, 90.0 * pi / 180.0); // Inverzní rotace

    vec3 p_cyl1 = p_handle - vec3(5.62, 1.76, 0.0);
    p_cyl1.xy = rotateZ(p_cyl1.xy, -105.19 * pi / 180.0);
    float d_cyl1 = sdCylinder(p_cyl1, 0.536 * 0.5, 5.0 * 0.5);

    vec3 p_cyl2 = p_handle - vec3(2.19, 0.59, 0.0);
    p_cyl2.xy = rotateZ(p_cyl2.xy, 62.0 * pi / 180.0);
    float d_cyl2 = sdCylinder(p_cyl2, 0.131 * 0.5, 2.293 * 0.5);

    // Sjednocení geometrie (Union)
    return min(d_blade, min(d_cyl1, d_cyl2));
}

bool rayMarchSpatula(Ray ray, out float t_hit, out vec3 hit_pos, float max_t) {
    vec3 origin = (invSpatulaTransform * vec4(ray.start, 1.0)).xyz;
    vec3 direction = (invSpatulaTransform * vec4(ray.dir, 0.0)).xyz;

    // Normalizace směru pro bezpečný sphere tracing (řeší ztenčení přes scale matici)
    float dir_len = length(direction);
    vec3 dir_local = direction / dir_len;

    // 1. Slab test pro Bounding Box (urychlení)
    vec3 invDir = 1.0 / (dir_local + sign(dir_local) * 1e-9);
    vec3 paddingT = vec3(0.05); // 5 cm reserve
    
    // Bounding box must encompass the blade and the new handle cylinders!
    vec3 boxMin = vec3(-spatulaDim.x - 9.0, -spatulaDim.y - 2.0, -spatulaDim.z - 9.0);
    vec3 boxMax = vec3( spatulaDim.x + 9.0,  5.0,  spatulaDim.z + 9.0);
    
    vec3 t0 = (boxMin - paddingT - origin) * invDir; // Lower bound of the box
    vec3 t1 = (boxMax + paddingT - origin) * invDir; // Upper bound of the box
    vec3 tMin = min(t0, t1);
    vec3 tMax = max(t0, t1);
    
    float t_near = max(tMin.x, max(tMin.y, tMin.z));
    float t_far = min(tMax.x, min(tMax.y, tMax.z));

    if (t_near > t_far || t_far < 0.0) return false;

    // 2. Sphere Tracing uvnitř Bounding Boxu
    float t = max(0.0, t_near - 0.001); // mírný posun vzad pro jistotu zachycení hrany
    int max_iter = fullRender ? 128 : 40;
    float hit_threshold = fullRender ? 0.0001 : 0.002;
    for(int i = 0; i < max_iter; i++) {
        vec3 p_local = origin + dir_local * t;
        float dist = sdSpatula(p_local, false);

        if(dist < hit_threshold) { // Povrch nalezen
            t_hit = t / dir_len; // Převod vzdálenosti zpět do World Space
            hit_pos = ray.start + ray.dir * t_hit;
            return true;
        }

        t += dist;
        if(t > t_far + 0.01 || (t / dir_len) > max_t) break;
    }

    return false;
}

vec3 getSpatulaNormal(vec3 pos_ws) {
    // Transformace bodu do lokálního prostoru
    vec3 p_local = (invSpatulaTransform * vec4(pos_ws, 1.0)).xyz;
    
    // Gradient - Epsilon (e.x) musí být vždy menší než celková tloušťka objektu!
    vec2 e = vec2(0.00005, 0.0);
    vec3 n_local = normalize(vec3(
        sdSpatula(p_local + e.xyy, false) - sdSpatula(p_local - e.xyy, false),
        sdSpatula(p_local + e.yxy, false) - sdSpatula(p_local - e.yxy, false),
        sdSpatula(p_local + e.yyx, false) - sdSpatula(p_local - e.yyx, false)
    ));
    
    // Transformace normály zpět do World Space
    // Korektní transformace normály pomocí inverzní transponované matice (L2W^-T)
    vec3 normal_ws = normalize(transpose(mat3(invSpatulaTransform)) * n_local);
    
    // Transformace do View Space pro lighting
    return normalize(mat3(view) * normal_ws);
}

///Computes position and direction in the world of the ray
void getRay(ivec2 pix, float depth, out Ray ray){
    vec2 res = vec2(width, height);
    vec2 ndc = 2.0f * ((vec2(pix) + 0.5f) / res) - 1.0f; //getting ndc
    vec4 p_v = invProj * vec4(ndc, -1.0f, 1.0f); //restoring point on a near plane in view coords
    p_v /= p_v.w;
    vec3 dir_v = normalize(p_v.xyz); //getting direction vector
    vec3 start_v = dir_v * depth/(-dir_v.z); //getting start poing from Dall, direction * (Dall depth) / direction.z in that pix
    ray.start = (invView * vec4(start_v, 1.0f)).xyz;
    ray.dir = normalize((invView * vec4(dir_v, 0.0f)).xyz);
}
///Changes level between texels in bdg
void changeLevel(uint level, in Ray ray, inout State state){
    if(state.currLevel == level) return;
    vec3 pos = ray.start + ray.dir * state.tcurr;
    pos += ray.dir * 1e-4;
    float curr = voxelSize * float(level);
    state.cell = ivec3(floor((pos - gridStart) / curr));
    state.currLevel = level;
}
///Initializes state of 3D DDA
void stateInit(out State state, in Ray ray){
    state.cell = ivec3(floor((ray.start - gridStart) / voxelSize));
    bvec3 zero = equal(ray.dir, vec3(0.0f));
    state.delta = mix(voxelSize / abs(ray.dir), vec3(MAX_INT), zero);
    state.move = ivec3(sign(ray.dir));
    state.insideStepSize = voxelSize / float(stepsInside);
    state.tcurr = 0.0f;
    state.currLevel = 1u;
    state.tNext = vec3(0.0f);
}
///Checks if a texel(or voxel) for current bdg might contain surface
uint getPossibility(in State state){
    uint possibility = 0;
    ivec3 currentSize;
    uint levelM;

    if(state.currLevel == 4u) { currentSize = cS4; }
    else if(state.currLevel == 2u) { currentSize = cS2; }
    else { currentSize = cellsSize; }

    bvec3 outOfBoundsGreat = greaterThanEqual(state.cell, currentSize);
    bvec3 outOfBoundsLess = lessThan(state.cell, ivec3(0));

    if(any(outOfBoundsLess) || any(outOfBoundsGreat)){
        // 1. KONTROLA PADDINGU (pro anizotropní přesahy)
        // Povolujeme nahlédnout kousek za hranice v Level 1
        if (state.currLevel == 1u) {
            if (all(greaterThanEqual(state.cell, ivec3(-3))) && 
                all(lessThanEqual(state.cell, cellsSize + ivec3(2)))) 
                return 1u; // Povolíme výpočet hustoty v "halo" zóně
        }

        // 2. KONTROLA SMĚRU (Zabránění nekonečným smyčkám)
        // Pokud jsme mimo a paprsek míří pryč od mřížky, ukončíme ho
        if ((outOfBoundsLess.x && state.move.x <= 0) || (outOfBoundsGreat.x && state.move.x >= 0) ||
            (outOfBoundsLess.y && state.move.y <= 0) || (outOfBoundsGreat.y && state.move.y >= 0) ||
            (outOfBoundsLess.z && state.move.z <= 0) || (outOfBoundsGreat.z && state.move.z >= 0)) {
            return 1000u; // Definitivní konec
        }

        // 3. POKRAČOVÁNÍ (Skip k mřížce)
        // Jsme mimo, ale míříme k mřížce -> skipneme prázdný prostor
        return 0u; 
    }

    // Načtení reálných dat z BDG pokud jsme uvnitř
    if(state.currLevel == 4u) possibility = M4[state.cell.x + cS4.x * (state.cell.z * cS4.y + state.cell.y)];
    else if(state.currLevel == 2u) possibility = M2[state.cell.x + cS2.x * (state.cell.z * cS2.y + state.cell.y)];
    else possibility = M[state.cell.x + cellsSize.x * (state.cell.z * cellsSize.y + state.cell.y)];

    return possibility;
}
///Moves the ray inside with tiny steps
void moveInside(inout State state, in Ray ray){
    bvec3 rl = equal(state.move, ivec3(1));
    vec3 nextCoord = vec3(mix(gridStart + vec3(state.cell) * h, gridStart + (vec3(state.cell)+ vec3(1.0f)) * h, rl));
    bvec3 b = equal(state.move, ivec3(0));
    state.tNext = mix((nextCoord - ray.start) / ray.dir, vec3(MAX_INT), b);
    uint id = (state.tNext.x < state.tNext.y && state.tNext.x < state.tNext.z) ? 0 : state.tNext.y < state.tNext.z ? 1 : 2;
    float next = state.tNext[id];
    state.tcurr += min(state.insideStepSize, next - state.tcurr);
    if(state.tcurr >= next){
        state.cell[id] += state.move[id];
        state.tNext[id] += state.delta[id];
    }
}
///Skips to the next voxel/texel
void skipToNext(inout State state, in Ray ray){
    vec3 pos = ray.start + ray.dir * state.tcurr;
    bvec3 rl = equal(state.move, ivec3(1));
    vec3 nextCoord = vec3(mix(gridStart + vec3(state.cell) * (h * float(state.currLevel)),
    gridStart + (vec3(state.cell)+ vec3(1.0f)) * (h * float(state.currLevel)), rl));
    bvec3 b = equal(state.move, ivec3(0));
    state.tNext = mix((nextCoord - pos) / ray.dir, vec3(MAX_INT), b); //if move is 0 then we do not consider this axis
    uint minMove = (state.tNext.x < state.tNext.y && state.tNext.x < state.tNext.z) ? 0 : state.tNext.y < state.tNext.z ? 1 : 2;
    state.cell[minMove] += state.move[minMove];
    state.tcurr += state.tNext[minMove];
//    state.tNext[minMove] += state.delta[minMove];
}
///Traverses the ray until it finds the cell marked in bdg
void updateTCurr(inout State state, in Ray ray){
    if(state.currLevel == 1u && getPossibility(state) == 1u){
        moveInside(state, ray);
        return;
    }
    uint level = maxLevel;
    changeLevel(level, ray, state);
    while(true){
        uint possibility = getPossibility(state);
        if(possibility == 1000u) return; //means out of bounds
        if(possibility == 0){
            skipToNext(state, ray);
        }
        else{
            if(state.currLevel == 1u){ // if not 1 we will go down because there is change level
                moveInside(state, ray);
                break;
            }
            level >>= 1u;
            changeLevel(level, ray, state);
        }
    }
}

vec3 computeFilteredColor(vec3 pos) {
    // 1. Setup spatial parameters pro rozšířené okolí (3x3x3 = 27 voxelů)
    vec3 pig_pos = (pos - gridStart) / (voxelSize / 4.0) - 0.5;
    ivec3 center_cell = ivec3(round(pig_pos)); // Místo floor bereme nejbližší středový voxel
    ivec3 pigCellsSize = cellsSize * 4;

    ParticlePigment colors[27];
    float weights[27];
    ParticlePigment mean;
    for(int c=0; c<7; ++c) mean.c[c] = 0.0;

    int i = 0;
    float sum_w = 0.0;

    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                ivec3 offset = ivec3(dx, dy, dz);
                ivec3 current_cell = clamp(center_cell + offset, ivec3(0), pigCellsSize - ivec3(1));
                uint idx = (uint(current_cell.z) * uint(pigCellsSize.y) * uint(pigCellsSize.x) + uint(current_cell.y) * uint(pigCellsSize.x) + uint(current_cell.x)) * 8u;
                
                // Načteme barvu souseda
                for(int c=0; c<7; ++c) colors[i].c[c] = data[idx + c];

                // Rozšířená vyhlazená váha (aplikace smoothstep filtru)
                vec3 dist = abs(pig_pos - vec3(center_cell + offset));
                // Smoothstep zajistí, že na hranici (1.5) bude váha plynule 
                // a měkce klesat k nule bez ostrého zlomu v interpolaci.
                vec3 w3 = smoothstep(vec3(1.5), vec3(0.0), dist);
                weights[i] = w3.x * w3.y * w3.z;
                sum_w += weights[i];
                
                i++;
            }
        }
    }

    // Normalizace vah a výpočet váženého průměru (smooth trilinear ref)
    for (int j = 0; j < 27; ++j) {
        weights[j] /= max(sum_w, 0.00001);
        for(int c=0; c<7; ++c) {
            mean.c[c] += colors[j].c[c] * weights[j];
        }
    }

    // Normalizace průměrných pigmentů
    float mean_pigment_sum = mean.c[0] + mean.c[1] + mean.c[2] + mean.c[3];
    // Jelikož pracujeme pouze s neprůhlednými pigmenty, jejich součet by měl být vždy 1.
    if (mean_pigment_sum > 0.00001) {
        for(int c=0; c<4; ++c) mean.c[c] /= mean_pigment_sum;
    }

    // 2. Výpočet lokálního rozptylu (variance)
    float variance = 0.0;
    for (int j = 0; j < 27; ++j) {
        float diffSq = 0.0;
        for(int c=0; c<4; ++c) { // Používáme jen 4 pigmenty pro vzdálenost
            float d = colors[j].c[c] - mean.c[c];
            diffSq += d * d;
        }
        variance += diffSq * weights[j];
    }

    // 3. Adaptivní sigma na základě variance (rozptyl v 27 voxelech)
    float sigma_min = 0.001;
    float sigma_max = max(sigma_color, 0.01);
    float scale = 10.0; // Citlivost na varianci (možno později vytáhnout do uniform)
    float adaptiveSigma = mix(sigma_min, sigma_max, clamp(variance * scale, 0.0, 1.0));
    float sigColorSq = max(adaptiveSigma * adaptiveSigma, 0.000001);

    // 4. Bilaterální filtrace v rozšířeném okolí
    ParticlePigment final_pigment;
    for(int c=0; c<7; ++c) final_pigment.c[c] = 0.0;
    float totalWeight = 0.0;

    for (int j = 0; j < 27; ++j) {
        // Výpočet barevné podobnosti (Mixbox vzdálenost)
        float colorDistSq = 0.0;
        for(int c=0; c<4; ++c) {
            float d = colors[j].c[c] - mean.c[c];
            colorDistSq += d * d;
        }
        
        // Bilaterální váha: W_final = W_spatial * e^(-dist^2 / sigma^2)
        float rangeWeight = exp(-colorDistSq / sigColorSq);
        float finalWeight = weights[j] * (rangeWeight + 0.05);
        
        for (int c = 0; c < 7; ++c) {
            final_pigment.c[c] += colors[j].c[c] * finalWeight;
        }
        totalWeight += finalWeight;
    }

    // 5. Normalizace
    if (totalWeight > 0.00001) {
        for (int c = 0; c < 7; ++c) {
            final_pigment.c[c] /= totalWeight;
        }

        // Normalizace finálních pigmentů
        float final_pigment_sum = final_pigment.c[0] + final_pigment.c[1] + final_pigment.c[2] + final_pigment.c[3];
        // Jelikož pracujeme pouze s neprůhlednými pigmenty, jejich součet by měl být vždy 1.
        if (final_pigment_sum > 0.00001) {
            for(int c=0; c<4; ++c) final_pigment.c[c] /= final_pigment_sum;
        }

        return mix_latent_to_rgb(final_pigment);
    } else {
        // Bezpečný fallback na hladkou interpolaci (mean), pokud obě váhy zkolabují
        return mix_latent_to_rgb(mean); 
    }
}

///Computes density at the current point using only 27 cells around it using poly6 kern
float computeDensity(vec3 pos, vec3 ray_start, float depth, ivec2 pix, out vec3 outColor){
    float density = 0.0f;
    outColor = vec3(0.0f);
    
    float min_dist = 1000000.0f;
    uint closest_id = 0;
    bool found_closest = false;
    float accumulated_diffusion = 0.0f;

    ivec3 cell = ivec3(floor((pos - gridStart) / voxelSize));
    for(int x = -1; x < 2; x++){
        for(int y = -1; y < 2; y++){
            for(int z = -1; z < 2; z++){
                ivec3 currCell = cell + ivec3(x, y, z);
                bvec3 outOfBoundsGreat = greaterThanEqual(currCell, cellsSize);
                bvec3 outOfBoundsLess = lessThan(currCell, ivec3(0));
                if(outOfBoundsGreat.x || outOfBoundsGreat.y || outOfBoundsGreat.z ||
                outOfBoundsLess.x || outOfBoundsLess.y || outOfBoundsLess.z){
                    continue;
                }
                uint cellId = currCell.x + cellsSize.x * (currCell.z * cellsSize.y + currCell.y);
                uvec2 cellData = cellsData[cellId];
                uint amount = cellData.x;
                uint offset = cellData.y;
                for(uint i = 0; i < amount; i++){
                    uint sphereID = ids[offset + i];
                    
                    vec4 sphere = spheresData[sphereID];
                    vec3 d = pos - sphere.xyz;
                    float w = 1.0f;
                    float hr2 = 0.0f;
                    float current_r2 = 0.0f;

                    if(isAni){
                        vec3 d_ani = mat3(an[sphereID]) * d;
                        current_r2 = dot(d_ani, d_ani);
                        if(current_r2 > 1.0f) continue;
                        hr2 = 1.0f - current_r2;
                        w = dets[sphereID];
                    } else {
                        current_r2 = dot(d, d);
                        if(current_r2 > h*h) continue;
                        hr2 = h*h - current_r2;
                    }

                    if (use_closest_color) {
                        float dist_sq = isAni ? dot(d, d) : current_r2; // For closest color we need actual 3D space distance
                        if (dist_sq < min_dist) { // min_dist now stores squared distance
                            min_dist = dist_sq;
                            closest_id = sphereID;
                            found_closest = true;
                        }
                    }

                    w *= poly6 * hr2 * hr2 * hr2;
                    density += w;
                    
                    accumulated_diffusion += p_diffusion[sphereID] * w;
                }
            }
        }
    }
    if(density > iso) { // OPTIMIZATION: Only compute the heavy filtered color if we actually hit the surface, not in the 'halo'
        if (showDiffusion) {
            float shear_intensity;
            if (use_closest_color && found_closest)
            {
                shear_intensity = p_diffusion[closest_id];
            }
            else {
                shear_intensity = accumulated_diffusion / density;
            }
            float mix_factor = smoothstep(pigment_D_edge0, pigment_D_edge1, shear_intensity);
            outColor = mix(vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 0.0), mix_factor);
        } else if (showNormals) {
            float rij = (height * 2.0 * DforRIJ) / (2.0 * abs(depth) * tan(viewAngle / 2.0));
            float w1 = A * length(pos - ray_start);
            float w2 = exp(B * rij);
            float w = min(w1 * w2, 1.0f);
            outColor = mix(vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 0.0), w);
        } else {
            if (fullRender) {
                outColor = computeFilteredColor(pos);
            } else {
                outColor = found_closest ? mix_latent_to_rgb(p_pigments[closest_id]) : vec3(0.8);
            }
        }
    }
    return density;
}

/// Odlehčená verze výpočtu hustoty čistě pro stínové volumetrické paprsky (bez barvy)
float computeDensityOnly(vec3 pos, float radius_scale){
    float density = 0.0f;
    ivec3 cell = ivec3(floor((pos - gridStart) / voxelSize));
    
    // Zvětšíme oblast hledání, pokud je stínový kužel široký (zachytíme i zvětšené částice)
    int ext = radius_scale > 1.5 ? 2 : 1;
    
    for(int x = -ext; x <= ext; x++){
        for(int y = -ext; y <= ext; y++){
            for(int z = -ext; z <= ext; z++){
                ivec3 currCell = cell + ivec3(x, y, z);
                bvec3 outOfBoundsGreat = greaterThanEqual(currCell, cellsSize);
                bvec3 outOfBoundsLess = lessThan(currCell, ivec3(0));
                if(any(outOfBoundsGreat) || any(outOfBoundsLess)) continue;
                
                uint cellId = currCell.x + cellsSize.x * (currCell.z * cellsSize.y + currCell.y);
                uvec2 cellData = cellsData[cellId];
                uint amount = cellData.x;
                uint offset = cellData.y;
                for(uint i = 0; i < amount; i++){
                    uint sphereID = ids[offset + i];
                    vec4 sphere = spheresData[sphereID];
                    vec3 d = pos - sphere.xyz;
                    float w = 1.0f;
                float hr2 = 0.0f;
                float rs2 = radius_scale * radius_scale;

                    if(isAni){
                        vec3 d_ani = mat3(an[sphereID]) * d;
                    float r2 = dot(d_ani, d_ani) / rs2;
                    if(r2 > 1.0f) continue;
                    hr2 = 1.0f - r2;
                        w *= dets[sphereID];
                } else {
                    float r2 = dot(d, d) / rs2;
                    if(r2 > h*h) continue;
                    hr2 = h*h - r2;
                    }
                
                    w *= poly6 * hr2 * hr2 * hr2;
                    // Zachování celkové hmotnosti vydělením novým objemem (scale^3)
                density += w / (rs2 * radius_scale);
                }
            }
        }
    }
    return density;
}

float calcFluidShadow(vec3 ro, vec3 rd, float maxt, bool is_from_floor) {
    float res = 1.0;
    // Dynamický offset: menší pro podlahu (0.02), větší pro self-shadowing (0.1)
    float t = is_from_floor ? 0.02 : 0.1;
    float k = 4.0; // Měkost stínu

    // Optimalizace: Tekutina je jen u podlahy, není třeba trasovat daleko.
    // Omezíme maximální vzdálenost a počet kroků.
    float effective_maxt = 2.5; // Maximální délka stínu od tekutiny
    int max_steps = 80;

    for(int i = 0; i < max_steps && t < effective_maxt; i++) {
        vec3 p = ro + rd * t;
        
        // Vzorkujeme hustotu s radius_scale = 1.0
        float dens = computeDensityOnly(p, 1.0);
        
        if (dens > 0.001) {
            float d = clamp((iso - dens) / iso, 0.0, 1.0);
            res = min(res, k * d / t);
            if(dens > iso * 1.2) return 0.0; // Uvnitř hustého objemu je plný stín
        }
        
        if(res < 0.01) return 0.0;

        // Konzistentní krok pro plynulé stíny, odvozený od 'h'
        t += h * 0.4; 
    }
    return clamp(res, 0.0, 1.0);
}

/// Vypočítá měkký SDF stín pro špachtli a její rukojeť
float calcSpatulaShadow(vec3 ro, vec3 rd, float mint, float maxt, float k) {
    if (!has_spatula) return 1.0;
    
    vec3 origin = (invSpatulaTransform * vec4(ro, 1.0)).xyz;
    vec3 direction = (invSpatulaTransform * vec4(rd, 0.0)).xyz;
    float dir_len = length(direction);
    vec3 dir_local = direction / dir_len;

    // OPTIMIZATION: Slab test pro Bounding Box pro stínový paprsek
    vec3 invDir = 1.0 / (dir_local + sign(dir_local) * 1e-9);
    vec3 paddingT = vec3(0.05); // 5 cm reserve
    vec3 boxMin = vec3(-spatulaDim.x - 9.0, -spatulaDim.y - 2.0, -spatulaDim.z - 9.0);
    vec3 boxMax = vec3( spatulaDim.x + 9.0,  5.0,  spatulaDim.z + 9.0);
    
    vec3 t0 = (boxMin - paddingT - origin) * invDir;
    vec3 t1 = (boxMax + paddingT - origin) * invDir;
    vec3 tMin = min(t0, t1);
    vec3 tMax = max(t0, t1);
    
    float t_near = max(tMin.x, max(tMin.y, tMin.z));
    float t_far = min(tMax.x, min(tMax.y, tMax.z));

    // Paprsek míjí Bounding Box špachtle -> 100% světlo
    if (t_near > t_far || t_far < 0.0 || (t_near / dir_len) > maxt) return 1.0;

    float res = 1.0;
    // Začneme stínovat až od Bounding Boxu, ušetří to kroky v prázdnu!
    float t = max(mint, (t_near / dir_len) - 0.01); 
    
    for(int i = 0; i < 30 && t < maxt; i++) { // OPTIMIZATION: Sníženo z 64 na 30 iterací
        vec3 p = ro + rd * t;
        vec3 p_local = (invSpatulaTransform * vec4(p, 1.0)).xyz;
        float d = sdSpatula(p_local, true);
        float d_world = d / dir_len;
        
        if(d_world < 0.001) return 0.0; // Paprsek narazil do špachtle
        
        res = min(res, k * d_world / t);
        t += clamp(d_world, 0.03, 0.4); // Zvětšený minimální krok
        
        if ((t * dir_len) > t_far + 0.05) break; // Konec, jakmile opustíme AABB
    }
    return clamp(res, 0.0, 1.0);
}

float calcShadow(vec3 ro, vec3 rd, bool is_from_floor) {
    float maxt = 10.0; // Maximální vzdálenost stínování
    float spatShadow = calcSpatulaShadow(ro, rd, 0.05, maxt, 16.0); // k=16 pro měkký penumbra okraj
    float fluidShadow = calcFluidShadow(ro, rd, maxt, is_from_floor);
    return spatShadow * fluidShadow;
}

///Computes object normal using spiky kern
vec3 getObjectNormal(vec3 xij){
    vec3 grad = vec3(0);
    ivec3 cell = ivec3(floor((xij - gridStart) / voxelSize));
    for(int x = -1; x < 2; x++){
        for(int y = -1; y < 2; y++){
            for(int z = -1; z < 2; z++){
                ivec3 currCell = cell + ivec3(x, y, z);
                bvec3 outOfBoundsGreat = greaterThanEqual(currCell, cellsSize);
                bvec3 outOfBoundsLess = lessThan(currCell, ivec3(0));
                if(outOfBoundsGreat.x || outOfBoundsGreat.y || outOfBoundsGreat.z ||
                outOfBoundsLess.x || outOfBoundsLess.y || outOfBoundsLess.z){
                    continue;
                }
                uint cellId = currCell.x + cellsSize.x * (currCell.z * cellsSize.y + currCell.y);
                uvec2 cellData = cellsData[cellId];
                uint amount = cellData.x;
                uint offset = cellData.y;
                for(int i = 0; i < amount; i++){
                    uint sphereId = ids[offset + i];
                    
                    vec4 sphere = spheresData[sphereId];
                    vec3 d = xij - sphere.xyz;
                    float r2 = 0.0f;
                    vec3 d_eval = d;
                    
                    float over2 = h * h;
                    float less2 = 0.0f;
                    
                    if(isAni){
                        d_eval = mat3(an[sphereId]) * d;
                        r2 = dot(d_eval, d_eval);
                        over2 = 1.0f;
                        less2 = 1e-12f;
                    } else {
                        r2 = dot(d, d);
                    }
                    
                    if(r2 > over2 || r2 <= less2){
                        continue;
                    }
                    
                    float r = sqrt(r2); // Odmocninu děláme jen pro body, které skutečně prošly testem (zlomek původního počtu)
                    float over = isAni ? 1.0f : h;
                    float hr = over - r;
                    float dw = spiky * (hr * hr);
                    vec3 dir = isAni ? transpose(mat3(an[sphereId])) * d_eval / r : d / r;
                    grad += dir * dw;
                }
            }
        }
    }
    return normalize(-grad);
}
///Computes final normal by blending screen and object space normals
vec3 computeNormal(vec3 pij, vec3 xij, ivec2 pix, float depth_from_tex){
    float realDepth = (view * vec4(xij, 1.0)).z;
    vec3 NScreenxij = texelFetch(Nscreen, pix, 0).xyz;
    
    if (!fullRender) {
        return NScreenxij; // Extrémní zrychlení pro preview: použije se pouze normála z depth bufferu
    }
    
    // cekch discontinuity
    if (abs(abs(realDepth) - depth_from_tex) > h * 0.5) {
        return normalize(mat3(view) * getObjectNormal(xij));
    }
    float rij = (height * 2*DforRIJ) / (2 * abs(realDepth) * tan(viewAngle/2));
    float w1 = A * length(xij - pij);
    float w2 = exp(B*rij);
    float w = min(w1*w2, 1);
    if(w< 0.05f){
        return NScreenxij;
    }
    vec3 NObjectij = normalize(mat3(view) * getObjectNormal(xij));
    return normalize(w * NObjectij + (1 - w)*NScreenxij);
}

float computeContactShadow(vec3 pos) {
    float ao = 1.0;
    
    // 1. Fluid AO
    float dist_fluid = 1e6;
    vec3 p_ao = pos + vec3(0.0, 0.002, 0.0); // Začneme mírně nad podlahou

    // Prohledáváme prostor směrem nahoru a hledáme první bod, kde hustota překročí 'iso'.
    // Tím zajistíme, že stín vrhá pouze hmota, která je viditelná i v hlavním renderu.
    for (int i = 0; i < 60; ++i) { 
        float density = computeDensityOnly(p_ao, 1.0);
        if (density > iso) { // Používáme stejnou prahovou hodnotu jako pro renderování
            dist_fluid = p_ao.y - pos.y;
            break;
        }
        p_ao.y += 0.005; // Krok o 5mm
    }
    float t_fluid = clamp(dist_fluid / ao_range_fluid, 0.0, 1.0); // Normalizujeme vzdálenost na [0,1]
    float fluid_ao_factor = sqrt(t_fluid); // Ease-out křivka pro plynulý přechod
    ao = min(ao, mix(min_darkness_fluid, 1.0, fluid_ao_factor)); // Smícháme mezi plnou tmavostí a žádným stínem

    // 2. Spatula AO
    if (has_spatula) {
        vec3 p_local = (invSpatulaTransform * vec4(pos, 1.0)).xyz;
        float dist_spatula = sdSpatula(p_local, true);
        float t_spatula = clamp(dist_spatula / ao_range_spatula, 0.0, 1.0);
        float spatula_ao_factor = sqrt(t_spatula); // Ease-out křivka
        ao = min(ao, mix(min_darkness_spatula, 1.0, spatula_ao_factor));
    }

    // Nelineární průběh pro "hustší" stín u kontaktu
    // Původní pow(occ, 2.0) je odstraněn, protože mix() a sqrt() už poskytují nelineární průběh.
    return ao;
}

// Jednoduchá pseudonáhodná funkce
float rand(vec2 co) {
    return fract(sin(dot(co.xy, vec2(12.9898, 78.233))) * 43758.5453);
}

///Main ray marching loop that traverses ray into the scene until it finds the surface or no.
void main(){
    ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
     if(pix.x >= width || pix.y >= height){
        return;
    }

    cS4 = (cellsSize + ivec3(4) - 1) / 4;
    cS2 = (cellsSize + ivec3(2) - 1) / 2;

    if(!isAni)
        poly6 /= pow(h, 9.0f);
    spiky /= pow(h, 6.0f);

    Ray ray_vdb;
    getRay(pix, 0.0f, ray_vdb);
    
    // 1. Intersect Spatula
    float t_spatula = 1e6; 
    bool hit_spatula = false;

    // TEMP
    if (has_spatula) {
    // if (false) {
        vec3 p_temp;
        hit_spatula = rayMarchSpatula(ray_vdb, t_spatula, p_temp, 1000.0);
    }

    // 2. Intersect Floor
    float t_floor = 1e6;
    bool hit_floor = false;
    // Kontrola, že paprsek míří směrem dolů k podlaze
    if (ray_vdb.dir.y < -0.0001) {
        // Zde můžeš upravit výšku podlahy. Např. + 0.05 ji zvedne o 5 cm
        float floor_height = - 0.025; 
        float temp_t = (floor_height - ray_vdb.start.y) / ray_vdb.dir.y;
        if (temp_t > 0.0) {
            t_floor = temp_t;
            hit_floor = true;
        }
    }

    // 3. Resolve nearest environment intersection
    float t_env = 1e6;
    int env_hit_type = 0; // 0 = none, 1 = spatula, 2 = floor
    if (hit_spatula && (!hit_floor || t_spatula < t_floor)) {
        t_env = t_spatula;
        env_hit_type = 1;
    } else if (hit_floor) {
        t_env = t_floor;
        env_hit_type = 2;
    }

    // 4. Raymarch MPM
    float depth = texelFetch(Dall, pix, 0).r;
    if (depth > 0.0f) {
        Ray ray_mpm;
        getRay(pix, depth, ray_mpm);
        State state;
        stateInit(state, ray_mpm);
        
        bool hasSkipped = false;
        float dist_to_mpm_start = length(ray_mpm.start - ray_vdb.start);
        
        for(uint i = 0; i < maxStepCount; i++){
            if(i > maxSkipCount && !hasSkipped){
                depth = texelFetch(Dagg, pix, 0).r;
                hasSkipped = true;
                getRay(pix, depth, ray_mpm);
                stateInit(state, ray_mpm); //Skipping to Dagg depth and starting from there
                dist_to_mpm_start = length(ray_mpm.start - ray_vdb.start);
            }
            updateTCurr(state, ray_mpm);

            // Early exit if we marched further than our closest environment hit
            if (env_hit_type != 0 && (state.tcurr + dist_to_mpm_start) > t_env) {
                break;
            }

            vec3 pos = ray_mpm.start + ray_mpm.dir * state.tcurr;
            if(getPossibility(state) == 1000u){
                break;
            }

            vec3 surfaceColor;
            float density = computeDensity(pos, ray_mpm.start, depth, pix, surfaceColor);

            if(density > iso){
                vec3 N = computeNormal(ray_mpm.start, pos, pix, depth);
                vec3 N_world = normalize(mat3(invView) * N);
                vec3 V_world = -ray_mpm.dir;

                float shadows[MAX_LIGHTS];
                if (fullRender) {
                    for(int s = 0; s < numLights; s++) {
                        shadows[s] = calcShadow(pos, normalize(lightDirs[s]), false);
                    }
                }

                vec3 final_color;
                if (fullRender) {
                    Material fluidMat_colored = fluidMat;
                    fluidMat_colored.albedo = surfaceColor; // Použijeme barvu přímo jako albedo bez umělého zesílení
                    final_color = computePBRLighting(fluidMat_colored, floorMat, pos, N_world, V_world, shadows);
                } else {
                    vec3 irradiance = texture(irradianceMap, N_world).rgb;
                    final_color = surfaceColor * irradiance;
                }

                float viewZ = (view * vec4(pos, 1.0)).z;
                imageStore(outTex, pix, vec4(final_color, 10.0f)); // 10.0f slouží jako tag pro tekutinu
                imageStore(normalDepthTex, pix, vec4(N, viewZ));
                return;
            }
        }
    }

    // 5. Render environment fallback
    if (env_hit_type == 1) { // Spatula
        vec3 pos_spatula = ray_vdb.start + ray_vdb.dir * t_env;
        vec3 N = getSpatulaNormal(pos_spatula);
        vec3 N_world = normalize(mat3(invView) * N);
        vec3 V_world = -ray_vdb.dir;

        float shadows[MAX_LIGHTS];
        if (fullRender) {
            for(int s = 0; s < numLights; s++) {
                shadows[s] = calcShadow(pos_spatula, normalize(lightDirs[s]), false);
            }
        }

        vec3 final_spatula_color;
        if (fullRender) {
            final_spatula_color = computePBRLighting(spatulaMat, floorMat, pos_spatula, N_world, V_world, shadows);
        } else {
            vec3 irradiance = texture(irradianceMap, N_world).rgb;
            final_spatula_color = spatula_color * irradiance;
        }
        
        imageStore(outTex, pix, vec4(final_spatula_color, 10.0f)); // 10.0f slouží jako tag pro špachtli
        float viewZ = (view * vec4(pos_spatula, 1.0)).z;
        imageStore(normalDepthTex, pix, vec4(N, viewZ));
    } else if (env_hit_type == 2) { // Floor
        vec3 hit_pos = ray_vdb.start + t_env * ray_vdb.dir;
        vec3 N_floor = vec3(0.0, 1.0, 0.0);
        vec3 N_floor_view = normalize(mat3(view) * N_floor);
        vec3 V_world = -ray_vdb.dir;
        
        // Checkerboard pattern
        vec2 checker = floor(hit_pos.xz * 4.0);
        float c = mod(checker.x + checker.y, 2.0);
        vec3 floor_color = floorCol.rgb * (0.8 + c * 0.2);
        
        float shadows[MAX_LIGHTS];
        float contactOcc = 1.0;
        if (fullRender) {
            contactOcc = computeContactShadow(hit_pos);
            for(int s = 0; s < numLights; s++) {
                float rnd = rand(hit_pos.xz * 100.0 + vec2(s * 13.0, s * 17.0));
                vec3 jitterDir = normalize(lightDirs[s] + (rnd - 0.5) * 0.2);                
                // Stín z oken (kontaktní stín se aplikuje až na konci)
                shadows[s] = calcShadow(hit_pos, jitterDir, true);
            }
        }

        vec3 final_floor_color;
        if (fullRender) {
            final_floor_color = computePBRLighting(floorMat, floorMat, hit_pos, N_floor, V_world, shadows);
            // Aplikace kontaktního stínu (Ambient Occlusion) na finální barvu.
            // Tím se ztmaví jak přímé, tak nepřímé (ambientní) osvětlení.
            final_floor_color *= contactOcc;
        } else {
            vec3 irradiance = texture(irradianceMap, N_floor).rgb;
            final_floor_color = floor_color * irradiance;
        }
        
        float floorViewZ = (view * vec4(hit_pos, 1.0)).z;
        float outAlpha = 1.0f;
        if (fullRender && numLights > 0) {
            float shadowSum = 0.0;
            for(int s=0; s<numLights; ++s) shadowSum += shadows[s];
            outAlpha = shadowSum / numLights;
        }
        imageStore(outTex, pix, vec4(final_floor_color, outAlpha)); // Hodnota stínu do Alpha kanálu
        imageStore(normalDepthTex, pix, vec4(N_floor_view, floorViewZ));
    } else { // Background
        imageStore(normalDepthTex, pix, vec4(1000.0f));
        if (fullRender) {
            imageStore(outTex, pix, vec4(0.0, 0.0, 0.0, -1.0f)); // -1.0f tag pro průhledné pozadí
        } else {
            imageStore(outTex, pix, vec4(0.2, 0.2, 0.2, -2.0f)); // -2.0f tag pro pevné pozadí
        }
    }
}