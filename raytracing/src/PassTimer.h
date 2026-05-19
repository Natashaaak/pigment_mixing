//----------------------------------------------------------------------------------------
/**
 * \file       PassTimer.h
 * \brief      Stores data structures for simulation time statistics
 *
 *  Provides the interface to store and use CPU and GPU counters of the passed time for the covered step.
 *
 */
//----------------------------------------------------------------------------------------

#ifndef TIMER_H
#define TIMER_H

#pragma once
#include "state.h"

struct RingBufHist {
    const char *name;
    static const unsigned N = 120;
    std::vector<float> v;
    unsigned id = 0;
    float sum = 0.0f;
    float max = 0.0f;
    float min = 100000.0f;
    unsigned inUse = 1;
    explicit RingBufHist(const char *name) : name(name), v(N, 0) {}
    void add(float ms) {
        if (id == 0) max = 0.0f; // to see max in the last N frames
        if (ms > max) max = ms;
        if (ms < min) min = ms;
        if (id >= inUse) inUse++; //to have right statistics first seconds
        sum -= v[id];
        sum += ms;
        v[id++] = ms;
        id %= N;
    }
    float avg() const{
        return sum/static_cast<float>(inUse);
    }
    float getMax() const {
        return max;
    }
    float getMin() const {
        return min;
    }
    float last() const {
        if (inUse == 0) return 0.0f;
        return v[(id + N - 1) % N];
    }
};

struct GpuPassTimer {
    static const unsigned N_PASSES = 2;
    static const unsigned N_FRAMES = 120;
    int currFrame = 0;
    std::vector<RingBufHist> rbh;
    std::vector<std::vector<GLuint>> queries;
    void init() {
        queries.resize(N_PASSES);
        for (unsigned i = 0; i < N_PASSES; ++i) {
            queries[i].resize(N_FRAMES, 0);
            glGenQueries(N_FRAMES, queries[i].data());
        }
        rbh.emplace_back("Rendering (GPU)");
        rbh.emplace_back("GUI (GPU)");
    }
    void start(unsigned id) {
        if (id >= N_PASSES) return;
        glBeginQuery(GL_TIME_ELAPSED, queries[id][currFrame]);
    }
    void end(unsigned id) {
        if (id >= N_PASSES) return;
        glEndQuery(GL_TIME_ELAPSED);
    }
    void update() {
        int prev = (currFrame + N_FRAMES - 1) % N_FRAMES;
        for (unsigned i = 0; i < N_PASSES; ++i) {
             // This is a blocking call. It will stall the CPU until the GPU result is available.
            // This provides accurate, non-delayed timing at the cost of performance.
            GLuint64 ns = 0;
            glGetQueryObjectui64v(queries[i][prev], GL_QUERY_RESULT, &ns);
            if (state.play)
                rbh[i].add(ns / 1e6f);
        }
        currFrame = (currFrame + 1) % N_FRAMES;
    }
    void plotLine() {
        for (auto& buf : rbh) {
            ImGui::SeparatorText(buf.name);
            ImGui::Text("avg: %.3f ms", buf.avg());
            ImGui::Text("Worst time: %.3f ms", buf.max);
            ImGui::PlotLines(buf.name, buf.v.data(), buf.N, buf.id, nullptr, 0.0f, buf.max);
        }
    }
    void clear() {
        for (unsigned i = 0; i < N_PASSES; ++i)
            if (!queries[i].empty())
                glDeleteQueries(N_FRAMES, queries[i].data());
        queries.clear();
    }
};

struct CpuPassTimer {
    static const unsigned N = 5;
    std::vector<RingBufHist> rbh;
    std::vector<std::pair<std::chrono::high_resolution_clock::time_point, std::chrono::high_resolution_clock::time_point>> se;
    void start(unsigned id) {
        if (id >= se.size()) return;
        se[id].first = std::chrono::high_resolution_clock::now();
    }
    void end(unsigned id) {
        if (id >= se.size()) return;
        se[id].second = std::chrono::high_resolution_clock::now();
        if (state.play)
            rbh[id].add(std::chrono::duration<float, std::milli>(se[id].second - se[id].first).count());
    }
    void init() {
        se.resize(N);
        rbh.emplace_back("Simulation step");
        rbh.emplace_back("GPU Data Prep");
        rbh.emplace_back("Classification");
        rbh.emplace_back("Particle Marshalling");
        rbh.emplace_back("Adding particles"); // This seems unused but I will leave it
        // rbh.emplace_back("Whole frame");
    }
    void plotLine() {
        float cpuPassTime = 0.0f;
        for (auto& buf: rbh) {
            const char* name = buf.name;
            float avg = buf.avg();
            cpuPassTime += avg;
            ImGui::SeparatorText(name);
            ImGui::Text("avg: %.3f ms", avg);
            ImGui::Text("Worst time: %.3f ms", buf.max);
            ImGui::PlotLines(name, buf.v.data(), buf.N, buf.id, nullptr, 0.0f, buf.max);
        }
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 200, 0, 255));
        ImGui::Text("Average time for the whole CPU pass is: %.3f ms", cpuPassTime);
        ImGui::PopStyleColor();
    }
};

extern GpuPassTimer timer;
extern GpuPassTimer depthVarTimer;
extern CpuPassTimer ctimer;

#endif //TIMER_H
