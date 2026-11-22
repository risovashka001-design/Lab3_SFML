#include "Collisions.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace lab3 {
namespace {
constexpr double EPS = 1e-9;
double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

bool matchPair(const Body& a, const Body& b, PairPass pass) {
    if (!a.active || !b.active) return false;
    if (!isMoving(a) && !isMoving(b)) return false;
    const bool fast = a.kind == Kind::Fast || b.kind == Kind::Fast;
    if (pass == PairPass::WithFast) return fast;
    if (pass == PairPass::WithoutFast) return !fast;
    return true;
}

bool overlapping(const Body& a, const Body& b, Vec2& normal, double& penetration) {
    const double dx = b.position.x - a.position.x;
    const double dy = b.position.y - a.position.y;
    const double px = a.halfSize.x + b.halfSize.x - std::abs(dx);
    const double py = a.halfSize.y + b.halfSize.y - std::abs(dy);
    if (px <= 0 || py <= 0) return false;
    if (px < py) { normal = {dx < 0 ? -1.0 : 1.0, 0}; penetration = px; }
    else { normal = {0, dy < 0 ? -1.0 : 1.0}; penetration = py; }
    return true;
}
}

void advance(std::vector<Body>& bodies, double dt) {
    for (Body& b : bodies) if (isMoving(b)) b.position += b.velocity * dt;
}

bool applyImpulse(Body& a, Body& b, Vec2 normal, double penetration) {
    const double totalInvMass = a.invMass + b.invMass;
    if (totalInvMass <= 0) return false;
  
    if (penetration > 0) {
        const double correction = std::max(0.0, penetration - 0.0001) * 0.88 / totalInvMass;
        a.position -= normal * (correction * a.invMass);
        b.position += normal * (correction * b.invMass);
    }
    const Vec2 relative = b.velocity - a.velocity;
    const double toward = dot(relative, normal);
    if (toward >= -1e-8) return false;
   
    double e = std::abs(toward) < 35.0 ? 0.0 : std::max(a.restitution, b.restitution);

    if ((a.kind == Kind::Boundary || b.kind == Kind::Boundary) &&
        std::abs(normal.y) > 0.5) {
        const Body& moving = a.kind == Kind::Boundary ? b : a;
        const double floorE = moving.floorRestitution >= 0.0
                                ? moving.floorRestitution : 0.20;
        e = std::min(e, floorE);
    }
    const double impulse = -(1.0 + e) * toward / totalInvMass;
    a.velocity -= normal * (impulse * a.invMass);
    b.velocity += normal * (impulse * b.invMass);

    const Vec2 tangent{-normal.y, normal.x};
    const double slip = dot(b.velocity - a.velocity, tangent);

    const double frictionCoefficient = std::abs(normal.y) > 0.5 ? 0.35 : 0.005;
    const double friction = std::clamp(-slip / totalInvMass,
                                       -frictionCoefficient * impulse,
                                        frictionCoefficient * impulse);
    a.velocity -= tangent * (friction * a.invMass);
    b.velocity += tangent * (friction * b.invMass);
    return true;
}

void recordImpulse(const Body& a, const Body& b, StepStats& stats) {
    if (a.markedWall || b.markedWall) ++stats.wallImpulses;
    if (isMoving(a) && isMoving(b)) {
        ++stats.dynamicDynamic;
        if ((a.kind == Kind::Fast && b.kind == Kind::Normal) ||
            (a.kind == Kind::Normal && b.kind == Kind::Fast)) ++stats.fastNormal;
    } else {
        ++stats.dynamicStatic;
    }
}

int solveOverlaps(std::vector<Body>& bodies, int iterations, PairPass pass, StepStats* stats) {
    int impulseCount = 0;
    for (int k = 0; k < iterations; ++k) {
        bool contact = false;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            for (std::size_t j = i + 1; j < bodies.size(); ++j) {
                if (!matchPair(bodies[i], bodies[j], pass)) continue;
                Vec2 normal{}; double penetration = 0;
                if (!overlapping(bodies[i], bodies[j], normal, penetration)) continue;
                contact = true;
                if (applyImpulse(bodies[i], bodies[j], normal, penetration)) {
                    ++impulseCount;
                    if (stats) recordImpulse(bodies[i], bodies[j], *stats);
                }
            }
        }
        if (!contact) break;
    }
    return impulseCount;
}


SweepHit sweptAABB(const Body& a, const Body& b, double maxTime) {
    SweepHit hit;
    if (!a.active || !b.active) return hit;
    if (!isMoving(a) && !isMoving(b)) return hit;
    const Vec2 v = a.velocity - b.velocity;
    const double amin[2] = {a.position.x - a.halfSize.x, a.position.y - a.halfSize.y};
    const double amax[2] = {a.position.x + a.halfSize.x, a.position.y + a.halfSize.y};
    const double bmin[2] = {b.position.x - b.halfSize.x, b.position.y - b.halfSize.y};
    const double bmax[2] = {b.position.x + b.halfSize.x, b.position.y + b.halfSize.y};
    const double speed[2] = {v.x, v.y};
    double enter[2], leave[2];
    for (int axis = 0; axis < 2; ++axis) {
        if (std::abs(speed[axis]) < 1e-10) {
            if (amax[axis] < bmin[axis] - EPS || amin[axis] > bmax[axis] + EPS) return hit;
            enter[axis] = -std::numeric_limits<double>::infinity();
            leave[axis] = std::numeric_limits<double>::infinity();
        } else if (speed[axis] > 0) {
            enter[axis] = (bmin[axis] - amax[axis]) / speed[axis];
            leave[axis] = (bmax[axis] - amin[axis]) / speed[axis];
        } else {
            enter[axis] = (bmax[axis] - amin[axis]) / speed[axis];
            leave[axis] = (bmin[axis] - amax[axis]) / speed[axis];
        }
    }
    const double first = std::max(enter[0], enter[1]);
    const double last = std::min(leave[0], leave[1]);
    if (first > last + 1e-10 || first < -1e-8 || first > maxTime + EPS || last < 0)
        return hit;
    const int axis = enter[0] > enter[1] ? 0 : 1;
    hit.normal = axis == 0 ? Vec2{v.x > 0 ? 1.0 : -1.0, 0}
                           : Vec2{0, v.y > 0 ? 1.0 : -1.0};
    if (dot(b.velocity - a.velocity, hit.normal) >= -1e-8) return {};
    hit.found = true;
    hit.time = std::max(0.0, first);
    return hit;
}
} 
