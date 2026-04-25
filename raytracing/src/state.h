//----------------------------------------------------------------------------------------
/**
 * \file       state.h
 * \brief      Contains simulation parameters.
 *
 *  Stores changeable parameters to control the simulation.
 *
 */
//----------------------------------------------------------------------------------------

#ifndef STATE_H
#define STATE_H

#pragma once
struct State {
    float iso = 0.01f;
    bool debugMode = false;
    glm::uvec3 localGroupSize = glm::uvec3(16, 8, 8);
    glm::ivec2 groupSizeRayMarching = glm::ivec2(16, 16);
    std::vector<unsigned> count{0,0};
    GLuint triCount = 0;
    float kern = 315.0f / (64.0f * glm::pi<float>());
    float k = 4.0f;
    float s = 1400.0f;
    int aniso_threshold = 4;
    float isoThresholdForDepthMap = 20.0f;
    bool autoISO = false;
    bool rayMarch = true;
    bool startMarch = false;
    bool recalcMarchParams = true;
    bool play = true;
    bool changeFontSize = false;
    int maxStepCount = 180;
    int maxSkipCount = 176;
    int stepsInside = 9;
    bool seeSpheres = true;
    bool autoScaleS = false;
    float ks = 1.0f;
    float kr = 0.25f;
    float R = 8.0f;
    // normal bleeding
    float A = 0.569f;
    float B = 0.467f;
    // adaptive ray marching treshold
    float e1 = 0.000085f;
    float e2 = 0.000210f;
    int currRes = 2;
    float vdall = 0.1f;
    float kn = 0.5f;
    bool isAni = false;
    bool testAllFilled = false;
    bool testFullRes = true;
    bool showDiffusion = true;
    std::vector<uint> aa{1u, 2u, 4u};
    std::vector<float> fonts{10.0f, 12.0f, 14.0f, 16.0f, 18.0f, 20.0f, 22.0f, 24.0f};
    int fontChoice = 3;
    std::vector<std::string> items{"Marching Cubes", "Ray Marching"};
    glm::vec2 params = {0.75, 0.74}; //a and b params for BDG filling from the paper
};

extern State state;


#endif //STATE_H
