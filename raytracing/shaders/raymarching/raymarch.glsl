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
    float c[8]; // Padded to 8 floats to perfectly match std::array<float, 8> 32-byte alignment in C++!
};

layout(local_size_x = 16, local_size_y = 16) in;

layout(rgba32f, binding = 0) writeonly uniform image2D outTex;

layout(rgba32f, binding = 1) writeonly uniform image2D normalDepthTex;

layout(binding = 2) uniform sampler2D Dagg;

layout(binding = 3) uniform sampler2D Dall;

layout(binding = 4) uniform sampler2D Nscreen;

layout(binding = 5) uniform usampler2D VarianceTex;

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
///Current variance pass
uniform int stride;
///if sould show only depth variance
uniform bool debugMode;
///Max level of bdg cells grouped into texels represented as 1 axis
uniform uint maxLevel;
///If should render with anisotropic kernel
uniform bool isAni;

uniform mat4 invSpatulaTransform;
uniform bool has_spatula;
uniform vec3 spatulaDim;
bool use_closest_color = false;
float r1 = 0.5;
float r2 = 0.2;

const float pi = 3.141592f;
const int lcs = 16; //equal to local size
float poly6 = 315.0f/(64.0f*pi); //poly6
float spiky = -45.0f/pi; //first derivative of spiky
const int MAX_INT = 2147483647;
const vec3 lightDirView = normalize(vec3(1.0f, -1.0f, -1.0f));

const vec3 spatula_color = vec3(1.0f, 0.0f, 0.0f);
const vec4 floorCol = vec4(0.375f, 0.35f, 0.325f, 1.0f);
ivec3 cS4 = ivec3(3);
ivec3 cS2 = ivec3(1);
uint variance = 0;

vec3 mix_latent_to_rgb( ParticlePigment pigments) {
    float c[8] = pigments.c;
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

float sdSpatula(vec3 p) {
    // Půl-rozměry lichoběžníku
    float b1 = spatulaDim.x;       // Spodní šířka
    float b2 = spatulaDim.x * 0.25; // Horní šířka
    float he = spatulaDim.z;       // Polovina výšky (Z)

    // 1. Přesný 2D SDF lichoběžníku (Centrovaný kolem Z=0)
    vec2 p2d = vec2(abs(p.x), p.z);
    vec2 k1 = vec2(b2, he);
    vec2 k2 = vec2(b2 - b1, 2.0 * he);
    
    vec2 ca = vec2(p2d.x - min(p2d.x, (p2d.y < 0.0) ? b1 : b2), abs(p2d.y) - he);
    vec2 cb = p2d - k1 + k2 * clamp(dot(k1 - p2d, k2) / dot(k2, k2), 0.0, 1.0);
    float s = (cb.x < 0.0 && ca.y < 0.0) ? -1.0 : 1.0;
    float d_2d = s * sqrt(min(dot(ca, ca), dot(cb, cb)));

    // 2. Korektní 3D extruze (Tloušťka v ose Y) pro sphere tracing
    float d_y = abs(p.y) - spatulaDim.y / 10;
    vec2 w = vec2(d_2d, d_y);
    return min(max(w.x, w.y), 0.0) + length(max(w, 0.0));
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
    vec3 t0 = (-spatulaDim - paddingT - origin) * invDir; // Lower bound of the box
    vec3 t1 = ( spatulaDim + paddingT - origin) * invDir; // Upper bound of the box
    vec3 tMin = min(t0, t1);
    vec3 tMax = max(t0, t1);
    
    float t_near = max(tMin.x, max(tMin.y, tMin.z));
    float t_far = min(tMax.x, min(tMax.y, tMax.z));

    if (t_near > t_far || t_far < 0.0) return false;

    // 2. Sphere Tracing uvnitř Bounding Boxu
    float t = max(0.0, t_near - 0.001); // mírný posun vzad pro jistotu zachycení hrany
    for(int i = 0; i < 128; i++) {
        vec3 p_local = origin + dir_local * t;
        float dist = sdSpatula(p_local);

        if(dist < 0.0001) { // Povrch nalezen
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
        sdSpatula(p_local + e.xyy) - sdSpatula(p_local - e.xyy),
        sdSpatula(p_local + e.yxy) - sdSpatula(p_local - e.yxy),
        sdSpatula(p_local + e.yyx) - sdSpatula(p_local - e.yyx)
    ));
    
    // Transformace normály zpět do World Space
    // Korektní transformace normály pomocí inverzní transponované matice (L2W^-T)
    vec3 normal_ws = normalize(transpose(mat3(invSpatulaTransform)) * n_local);
    
    // Transformace do View Space pro lighting
    return normalize(mat3(view) * -normal_ws);
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
    if(state.currLevel == 4u){
        bvec3 outOfBoundsGreat = greaterThanEqual(state.cell, cS4);
        bvec3 outOfBoundsLess = lessThan(state.cell, ivec3(0));
        if(any(outOfBoundsLess) || any(outOfBoundsGreat)){
            return 1000u;
        }
        possibility = M4[state.cell.x + cS4.x * (state.cell.z * cS4.y + state.cell.y)];
    }
    else if(state.currLevel == 2u){
        bvec3 outOfBoundsGreat = greaterThanEqual(state.cell, cS2);
        bvec3 outOfBoundsLess = lessThan(state.cell, ivec3(0));
        if(any(outOfBoundsLess) || any(outOfBoundsGreat)){
            return 1000u;
        }
        possibility = M2[state.cell.x + cS2.x * (state.cell.z * cS2.y + state.cell.y)];
    }
    else if(state.currLevel == 1u){
        bvec3 outOfBoundsGreat = greaterThanEqual(state.cell, cellsSize);
        bvec3 outOfBoundsLess = lessThan(state.cell, ivec3(0));
        if(any(outOfBoundsLess) || any(outOfBoundsGreat)){
            return 1000u;
        }
        possibility = M[state.cell.x + cellsSize.x * (state.cell.z * cellsSize.y + state.cell.y)];
    }
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

///Computes density at the current point using only 27 cells around it using poly6 kern
float computeDensity(vec3 pos, out vec3 outColor){
    float density = 0.0f;
    outColor = vec3(0.0f);
    
    float min_dist = 1000000.0f;
    uint closest_id = 0;
    bool found_closest = false;
    float accumulated_pigment[8] = float[](0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

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
                    float r = length(d);

                    if (use_closest_color) {
                        if (r < min_dist) {
                            min_dist = r;
                            closest_id = sphereID;
                            found_closest = true;
                        }
                    }

                    float compare = h;
                    float hr2 = h*h - r*r;
                    float w = 1.0f;
                    if(isAni){
                        d = mat3(an[sphereID]) * d;
                        r = dot(d, d);
                        compare = 1.0f;
                        hr2 = 1.0f - r;
                        w *= dets[sphereID];
                    }
                    if(r > compare){
                        continue;
                    }
                    w *= poly6 * hr2 * hr2 * hr2;
                    density += w;
                    
                    if (!use_closest_color) {
                        for (int c = 0; c < 8; ++c) {
                            accumulated_pigment[c] += p_pigments[sphereID].c[c] * w;
                        }
                    }
                }
            }
        }
    }
    if(density > 0.0f) {
        if (use_closest_color && found_closest) {
            outColor = mix_latent_to_rgb(p_pigments[closest_id]); // Convert latent to RGB using the mixbox coefficients
        } else if (!use_closest_color) {
            ParticlePigment blended_pigment;
            for (int i = 0; i < 8; ++i) blended_pigment.c[i] = accumulated_pigment[i] / density;
            outColor = mix_latent_to_rgb(blended_pigment);
        }
    }
    return density;
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
                    float over = h;
                    float less = 0.0f;
                    if(isAni){
                        d = mat3(an[sphereId]) * d;
                        over = 1.0f;
                        less = 1e-6;
                    }
                    float r = length(d);
                    if(r > over || r <= less){
                        continue;
                    }
                    float hr = over - r;
                    float dw = spiky * (hr * hr);
                    vec3 dir = d / r;
                    if(isAni) dir = transpose(mat3(an[sphereId])) * d / r;
                    grad += dir * dw;
                }
            }
        }
    }
    return normalize(grad);
}
///Computes final normal by blending screen and object space normals
vec3 computeNormal(vec3 pij, vec3 xij, ivec2 pix, float depth){
    float rij = (height * 2*DforRIJ) / (2 * abs(depth) * tan(viewAngle/2));
    vec3 NScreenxij = texelFetch(Nscreen, pix, 0).xyz;
    float w1 = A * length(xij - pij);
    float w2 = exp(B*rij);
    float w = min(w1*w2, 1);
    if(w< 0.05f){
        return NScreenxij;
    }
    vec3 NObjectij = normalize(mat3(view) * getObjectNormal(xij));
    return w * NObjectij + (1 - w)*NScreenxij;
}
///Main ray marching loop that traverses ray into the scene until it finds the surface or no.
void main(){
    bool hasSkipped = false;
    bool foundSurface = false;
    ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
    ivec2 origPix = pix * stride;
    if(origPix.x >= width || origPix.y>= height){
            return;
    }
    cS4 = (cellsSize + ivec3(4) - 1) / 4;
    cS2 = (cellsSize + ivec3(2) - 1) / 2;
    uint locId = gl_LocalInvocationIndex;
    variance = texelFetch(VarianceTex, ivec2(origPix/lcs), 0).r;
    if(debugMode){
        if(variance == 0){
            imageStore(outTex, origPix, vec4(1.0, 1.0, 1.0, 1.0)); //White
        }
        else if(variance == 1){
            imageStore(outTex, origPix, vec4(1.0, 0.0, 1.0, 1.0)); //Magenta
        }
        else if(variance == 2){
            imageStore(outTex, origPix, vec4(0.0, 1.0, 1.0, 1.0)); //Turqoise
        }
        else if(variance == 4){
            imageStore(outTex, origPix, vec4(1.0, 1.0, 0.0, 1.0)); //Yellow
        }
        return;
    }
    if(variance != stride /*|| shouldExit*/){
        return;
    }
    float depth = texelFetch(Dall, origPix, 0).r;
    if(!isAni)
        poly6 /= pow(h, 9.0f);
    spiky /= pow(h, 6.0f);
    Ray ray_vdb;
    getRay(origPix, 0.0f, ray_vdb);
    
    float t_spatula = 1e6; 
    bool hit_spatula = false;

    if (has_spatula) {
        float t_temp;
        vec3 p_temp;
        // SDF sphere tracing for spatula
        if (rayMarchSpatula(ray_vdb, t_temp, p_temp, 1000.0)) {
            t_spatula = t_temp;
            hit_spatula = true;
        }
    }

    Ray ray_mpm;
    if (depth > 0.0f) {
        getRay(origPix, depth, ray_mpm);
        State state;
        stateInit(state, ray_mpm);
        float dist_to_mpm_start = length(ray_mpm.start - ray_vdb.start);
        for(uint i = 0; i < maxStepCount; i++){
            if(i > maxSkipCount && !hasSkipped){
                float depthDagg = texelFetch(Dagg, origPix, 0).r;
                depth = depthDagg;
                hasSkipped = true;
                getRay(origPix, depthDagg, ray_mpm);
                stateInit(state, ray_mpm); //Skipping to Dagg depth and starting from there
                dist_to_mpm_start = length(ray_mpm.start - ray_vdb.start);
            }
            updateTCurr(state, ray_mpm);

            if (hit_spatula && (state.tcurr + dist_to_mpm_start) > t_spatula) break;  // if we already hit the spatula surface and ray for mpm is farther than that, we can stop

            vec3 pos = ray_mpm.start + ray_mpm.dir * state.tcurr;
            if(getPossibility(state) == 1000u){
                break;
            }

            vec3 surfaceColor;
            float density = computeDensity(pos, surfaceColor);

            if(density > iso){  // mpm is closer than vbd surface
                foundSurface = true;
                vec3 N = computeNormal(ray_mpm.start, pos, origPix, depth);
                float diff = max(dot(N, lightDirView), 0.0);
                imageStore(outTex, origPix, vec4(surfaceColor * diff, 1.0f));
                imageStore(normalDepthTex, origPix, vec4(N, pos.z));
                return;
            }
        }
    }

    float t_floor = 1e6;
    bool hit_floor = false;

    if (ray_vdb.dir.y < 0.0) {
        float temp_t = (gridStart.y - ray_vdb.start.y) / ray_vdb.dir.y;
        if (temp_t > 0.0) {
            t_floor = temp_t;
            hit_floor = true;
        }
    }

    if(hit_spatula){
        if (hit_floor && t_floor < t_spatula) {
            hit_spatula = false; // "Zrušíme" zásah špachtle, protože je pod zemí
        } else {
            vec3 pos_spatula = ray_vdb.start + ray_vdb.dir * t_spatula;
            vec3 N = getSpatulaNormal(pos_spatula);

            float diff = max(dot(N, lightDirView), 0.0);
            vec3 final_spatula_color = spatula_color * (diff * 0.8 + 0.2);

            imageStore(outTex, origPix, vec4(final_spatula_color, 1.0f));
            float viewZ = (view * vec4(pos_spatula, 1.0)).z;
            imageStore(normalDepthTex, origPix, vec4(N, viewZ));
            foundSurface = true;
        }
    }
    if(!foundSurface && hit_floor){
        // Floor plane intersection
        float t_floor = (gridStart.y - ray_vdb.start.y) / ray_vdb.dir.y;
        vec3 hit_pos = ray_vdb.start + t_floor * ray_vdb.dir;
        vec3 N_floor = vec3(0.0, 1.0, 0.0);
        vec3 N_floor_view = normalize(mat3(view) * N_floor);
        
        // Checkerboard pattern
        vec2 checker = floor(hit_pos.xz * 2.0);
        float c = mod(checker.x + checker.y, 2.0);
        vec3 floor_color = floorCol.rgb * (0.8 + c * 0.2);

        // Use absolute dot product and add ambient light so it doesn't render completely black
        float diff = abs(dot(N_floor_view, lightDirView)) * 0.7 + 0.3;
        
        float floorViewZ = (view * vec4(hit_pos, 1.0)).z; // Sjednocení prostoru hloubky
        imageStore(normalDepthTex, origPix, vec4(N_floor_view, floorViewZ));
        imageStore(outTex, origPix, vec4(floor_color * diff, 1.0));
        foundSurface = true;
    }
    if(!foundSurface){
        imageStore(normalDepthTex, origPix, vec4(1000.0f));
        imageStore(outTex, origPix, vec4(0.1, 0.1, 0.1, 1.0)); // Dark gray background
        return;
    }
    return;
}