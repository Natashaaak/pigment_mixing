#version 450
///Ray structure
struct Ray{
    vec3 start;
    vec3 dir;
};
///State structure for 3D DDA algorithm
struct State{
    ivec3 cell;
    ivec3 smallestCell;
    ivec3 move;
    vec3 delta; // is used only when moving inside the voxel
    vec3 tNext;
    float tcurr;
    float insideStepSize;
    uint currLevel;
};


layout(local_size_x = 16, local_size_y = 16) in;

layout(rgba32f, binding = 0) writeonly uniform image2D outTex;

layout(rgba32f, binding = 1) writeonly uniform image2D normalDepthTex;

layout(binding = 2) uniform sampler2D Dagg;

layout(binding = 3) uniform sampler2D Dall;

layout(binding = 4) uniform sampler2D Nscreen;

layout(binding = 5) uniform usampler2D VarianceTex;

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

const float pi = 3.141592f;
const int lcs = 16; //equal to local size
float poly6 = 315.0f/(64.0f*pi); //poly6
float spiky = -45.0f/pi; //first derivative of spiky
const int MAX_INT = 2147483647;
const vec3 lightDirView = normalize(vec3(1.0f, -1.0f, -1.0f));
const vec3 color = vec3(0.557f, 0.645f, 0.969f);
const vec4 floorCol = vec4(0.375f, 0.35f, 0.325f, 1.0f);
ivec3 cS4 = ivec3(3);
ivec3 cS2 = ivec3(1);
uint variance = 0;
///Computes position and direction in the world of the ray
void getRay(ivec2 pix, float depth, inout Ray ray){
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
void changeLevel(uint level, inout Ray ray, inout State state){
    if(state.currLevel == level) return;
    vec3 pos = ray.start + ray.dir * state.tcurr;
    pos += ray.dir * 1e-4;
    float curr = voxelSize * float(level);
    state.cell = ivec3(floor((pos - gridStart) / curr));
    state.currLevel = level;
}
///Initializes state of 3D DDA
void stateInit(inout State state, inout Ray ray){
    state.cell = ivec3(floor((ray.start - gridStart) / voxelSize));
    bvec3 zero = equal(ray.dir, vec3(0.0f));
    state.delta = mix(voxelSize / abs(ray.dir), vec3(MAX_INT), zero);
    state.move = ivec3(sign(ray.dir));
    state.insideStepSize = voxelSize / float(stepsInside);
    state.tcurr = 0.0f;
    state.currLevel = 1;
}
///Checks if a texel(or voxel) for current bdg might contain surface
uint getPossibility(inout State state){
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
void moveInside(inout State state, inout Ray ray){
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
void skipToNext(inout State state, inout Ray ray){
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
void updateTCurr(inout State state, inout Ray ray){
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
float computeDensity(vec3 pos){
    float density = 0.0f;
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
                }
            }
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
    if(depth <= 0.0f){ //means this is zero and there is no sphere
        imageStore(normalDepthTex, origPix, vec4(1000.0f));
        return;
    }
    if(!isAni)
        poly6 /= pow(h, 9.0f);
    spiky /= pow(h, 6.0f);
    Ray ray;
    getRay(origPix, depth, ray);
    State state;
    stateInit(state, ray);
    for(uint i = 0; i < maxStepCount; i++){
        if(i > maxSkipCount && !hasSkipped){
            float depthDagg = texelFetch(Dagg, origPix, 0).r;
            //            if(depthDagg == depth) // means we started and aggregation and did not found surface
            //            break;
            depth = depthDagg;
            hasSkipped = true;
            getRay(origPix, depthDagg, ray);
            stateInit(state, ray); //Skipping to Dagg depth and starting from there
        }
        updateTCurr(state, ray);
        vec3 pos = ray.start + ray.dir * state.tcurr;
        if(getPossibility(state) == 1000u){
            break;
        }
        float density = computeDensity(pos);
        if(density > iso){
            foundSurface = true;
            vec3 N = computeNormal(ray.start, pos, origPix, depth);
            float diff = max(dot(N, lightDirView), 0.0);
            imageStore(outTex, origPix, vec4(color*diff, 1.0f));
            imageStore(normalDepthTex, origPix, vec4(N, pos.z));
            break;
        }
    }
    if(!foundSurface){
        imageStore(normalDepthTex, origPix, vec4(1000.0f));
        return;
    }
    return;
}