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
};

struct GpuPassTimer {
    static const unsigned N = 120;
    int currFrame = 0;
    RingBufHist buf{"Whole GPU pass"};
    std::vector<GLuint> queries;
    void init() {
        queries.resize(N, 0);
        glGenQueries(N, queries.data());
    }
    void start() {
        glBeginQuery(GL_TIME_ELAPSED, queries[currFrame]);
    }
    void end() {
        glEndQuery(GL_TIME_ELAPSED);
    }
    void update() {
        int prev = (currFrame + N - 1) % N;
        GLint done = 0;
        glGetQueryObjectiv(queries[prev], GL_QUERY_RESULT_AVAILABLE, &done);
        if (done == 0) {
            spdlog::debug("GPU did not finish computes for {}", buf.name);
        }
        else {
            GLuint64 ns = 0;
            glGetQueryObjectui64v(queries[prev], GL_QUERY_RESULT, &ns);
            // if (state.play)
            buf.add(ns / 1e6f);
        }
        currFrame = (currFrame + 1) % N;
    }
    void plotLine() {
        const char* name = buf.name;
        float avg = buf.avg();
        ImGui::SeparatorText(name);
        ImGui::Text("avg: %.3f ms", avg);
        ImGui::Text("Worst time in the last %d frames: %.3f ms", buf.N ,buf.max);
        ImGui::PlotLines(name, buf.v.data(), buf.N, buf.id, nullptr, 0.0f, buf.max);
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 200, 0, 255));
        ImGui::Text("Average time for the whole GPU pass is: %.3f ms", avg);
        ImGui::PopStyleColor();
    }
    void clear() {glDeleteQueries(N, queries.data());}
};

struct CpuPassTimer {
    static const unsigned N = 4;
    std::vector<RingBufHist> rbh;
    std::vector<std::pair<std::chrono::high_resolution_clock::time_point, std::chrono::high_resolution_clock::time_point>> se;
    void start(unsigned id) {
        se[id].first = std::chrono::high_resolution_clock::now();
    }
    void end(unsigned id) {
        se[id].second = std::chrono::high_resolution_clock::now();
        if (state.play)
            rbh[id].add(std::chrono::duration<float, std::milli>(se[id].second - se[id].first).count());
    }
    void init() {
        se.resize(N);
        rbh.emplace_back("Simulation step");
        rbh.emplace_back("Binary Density Grid");
        rbh.emplace_back("Classification");
        rbh.emplace_back("Adding particles");
        // rbh.emplace_back("Whole frame");
    }
    void plotLine() {
        float cpuPassTime = 0.0f;
        for (auto& buf: rbh) {
            const char* name = buf.name;
            float avg = buf.avg();
            cpuPassTime += avg;
            ImGui::SeparatorText(name);
            ImGui::Text("%s", name);
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
