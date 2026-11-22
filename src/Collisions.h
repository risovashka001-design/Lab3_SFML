#pragma once
#include "Body.h"
#include <cstddef>

namespace lab3 {

enum class PairPass { All, WithFast, WithoutFast };

struct SweepHit {
    bool found = false;
    double time = 0;
    Vec2 normal{};
    std::size_t first = 0, second = 0;
};

void advance(std::vector<Body>& bodies, double dt);
int solveOverlaps(std::vector<Body>& bodies, int iterations, PairPass pass, StepStats* stats = nullptr);
void recordImpulse(const Body& a, const Body& b, StepStats& stats);
bool applyImpulse(Body& a, Body& b, Vec2 normal, double penetration = 0);
SweepHit sweptAABB(const Body& a, const Body& b, double maxTime);
} 
