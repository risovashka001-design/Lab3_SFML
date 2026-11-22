#include "World.h"
#include "Collisions.h"
#include "Scenes.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace lab3 {
namespace {
void gravity(std::vector<Body>& bodies, double dt, bool includeFast = false) {
    for (Body& b : bodies)
        if (b.kind == Kind::Normal || (includeFast && b.kind == Kind::Fast))
            b.velocity.y += GRAVITY * dt;
}
bool pairDemo(Scene scene) {
    return scene == Scene::FastNormal || scene == Scene::NormalNormal || scene == Scene::FastFast;
}

void airResistance(std::vector<Body>& bodies, Scene scene, double dt) {
    if (scene != Scene::ThinWall) return;
    constexpr double resistancePerSecond = 3.8;
    const double decay = std::exp(-resistancePerSecond * dt);
    for (Body& body : bodies) if (body.kind == Kind::Fast) {
        body.velocity.x *= decay;
        body.velocity.y *= decay;
    }
}
bool sceneHasGravity(Scene scene) {
    return scene == Scene::Platforms || scene == Scene::Supports ||
           scene == Scene::Stack || scene == Scene::StackImpact ||
           pairDemo(scene);
}
}

World::World(Scene s, Mode m) : scene(s), mode(m), bodies(makeScene(s)) {
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].kind == Kind::Normal) normalGroup.push_back(i);
        else if (bodies[i].kind == Kind::Fast) fastGroup.push_back(i);
    }
}

void World::step() {
    stats = {};
    microTrace.clear();
  
    if (scene == Scene::StackImpact && !stackProjectileLaunched &&
        timeSeconds >= 2.0 - 1e-9 && !fastGroup.empty()) {
        bodies[fastGroup.front()].velocity.x = 350.0;
        stackProjectileLaunched = true;
    }
    const double dt = 1.0 / NORMAL_HZ;
 
    airResistance(bodies, scene, dt);
    if (mode == Mode::Discrete) discrete(dt);
    else if (mode == Mode::MultiRate) multiRate(dt);
    else continuous(dt);
 
    if (scene == Scene::ThinWall && !wallEventResolved && bodies.size() > 1) {
        const Body& projectile = bodies[0];
        const Body& wall = bodies[1];
        const bool passed = projectile.position.x - projectile.halfSize.x >
                            wall.position.x + wall.halfSize.x;
        if (stats.wallImpulses > 0 || passed) wallEventResolved = true;
    }
   
    if (pairDemo(scene) && stats.dynamicDynamic > 0) pairGravityActive = true;
    if (scene == Scene::StackImpact && stats.fastNormal > 0) impactOccurred = true;
    timeSeconds += dt;
    evaluateStackRest();
}

void World::evaluateStackRest() {
    if (scene != Scene::Stack && scene != Scene::StackImpact) return;
    
    if (scene == Scene::StackImpact && !impactOccurred) return;
 
    constexpr double maxDrift = 0.35; 
    constexpr double maxSpeed = 5.0;  
    constexpr double requiredSeconds = 1.0;
    if (restWindowStart < 0) {
        restWindowStart = timeSeconds;
        restReferencePositions.clear();
        restReferencePositions.reserve(bodies.size());
        for (const Body& b : bodies) restReferencePositions.push_back(b.position);
        stackRestDurationSeconds = 0;
        stackRestDriftPx = 0;
        stackRestPeakSpeed = 0;
    }
    double drift = 0, speed = 0;
    for (std::size_t i=0; i < bodies.size(); ++i) {
        if (bodies[i].kind != Kind::Normal) continue;
        const Vec2 d=bodies[i].position-restReferencePositions[i];
        drift=std::max(drift, std::hypot(d.x,d.y));
        speed=std::max(speed, std::hypot(bodies[i].velocity.x,bodies[i].velocity.y));
    }
   
    if (drift > maxDrift || speed > maxSpeed) {
        restWindowStart = -1;
        stackRestDurationSeconds = 0;
        stackRestDriftPx = drift;
        stackRestPeakSpeed = speed;
        stackRestConfirmed = false;
        return;
    }
    stackRestDriftPx = std::max(stackRestDriftPx,drift);
    stackRestPeakSpeed = std::max(stackRestPeakSpeed,speed);
    stackRestDurationSeconds = timeSeconds - restWindowStart;
    if (stackRestDurationSeconds + 1e-10 >= requiredSeconds)
        stackRestConfirmed = true;
}


void World::discrete(double dt) {
    if (sceneHasGravity(scene) || pairGravityActive || scene == Scene::ThinWall)
        gravity(bodies, dt * (scene == Scene::ThinWall && wallEventResolved ? 2.5 : 1.0),
                pairDemo(scene) || scene == Scene::ThinWall || scene == Scene::Platforms ||
                (scene == Scene::StackImpact && impactOccurred));
    advance(bodies, dt);
    stats.impulses += solveOverlaps(bodies, 12, PairPass::All, &stats);
    stats.normalSteps = 1;
    stats.fastSteps = 0;
}

void World::multiRate(double dt) {

    if (fastGroup.empty()) {
        discrete(dt);
        stats.fastSteps = 0;
        return;
    }

    std::vector<Body> predicted = bodies;
    const bool useGravity = sceneHasGravity(scene) || pairGravityActive || scene == Scene::ThinWall;
    const bool fastAffectedByGravity = pairDemo(scene) || scene == Scene::ThinWall ||
                                     scene == Scene::Platforms ||
                                     (scene == Scene::StackImpact && impactOccurred);
    const double gravityScale = (scene == Scene::ThinWall && wallEventResolved) ? 2.5 : 1.0;
 
    if (useGravity) gravity(predicted, dt * gravityScale, false);
    const int microsteps = FAST_HZ / NORMAL_HZ;
    const double h = 1.0 / FAST_HZ;
    stats.normalSteps = 1;
    for (int i = 0; i < microsteps; ++i) {
        if (useGravity && fastAffectedByGravity) {
            for (std::size_t id : fastGroup)
                if (predicted[id].active)
                    predicted[id].velocity.y += GRAVITY * gravityScale * h;
        }
 
        for (std::size_t id : normalGroup) predicted[id].position += predicted[id].velocity * h;
        for (std::size_t id : fastGroup) predicted[id].position += predicted[id].velocity * h;
        const int previousWallImpulses = stats.wallImpulses;
        stats.impulses += solveOverlaps(predicted, 12, PairPass::WithFast, &stats);
        if (stats.wallImpulses > previousWallImpulses && stats.wallContactSubstep == 0)
            stats.wallContactSubstep = i + 1;
        ++stats.fastSteps;
        if (!fastGroup.empty() && !normalGroup.empty()) {
            const Body& fast = predicted[fastGroup.front()];
            const Body& normal = predicted[normalGroup.front()];
            microTrace.push_back({timeSeconds + (i + 1) * h,
                fast.position, normal.position, fast.velocity, normal.velocity});
        }
    }
    stats.impulses += solveOverlaps(predicted, 12, PairPass::WithoutFast, &stats);
    bodies = std::move(predicted); 
}

void World::continuous(double dt) {
    if (sceneHasGravity(scene) || pairGravityActive || scene == Scene::ThinWall)
        gravity(bodies, dt * (scene == Scene::ThinWall && wallEventResolved ? 2.5 : 1.0),
                pairDemo(scene) || scene == Scene::ThinWall || scene == Scene::Platforms ||
                (scene == Scene::StackImpact && impactOccurred));
    stats.impulses += solveOverlaps(bodies, 6, PairPass::All, &stats);
    double remaining = dt;
 
    for (int iteration = 0; iteration < 64 && remaining > 1e-9; ++iteration) {
        SweepHit earliest;
        earliest.time = remaining + 1.0;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            for (std::size_t j = i + 1; j < bodies.size(); ++j) {
                SweepHit candidate = sweptAABB(bodies[i], bodies[j], remaining);
                if (candidate.found && candidate.time < earliest.time - 1e-10) {
                    candidate.first = i; candidate.second = j;
                    earliest = candidate;
                }
            }
        }
        if (!earliest.found) { advance(bodies, remaining); remaining = 0; break; }
        advance(bodies, earliest.time);
        remaining -= earliest.time;
        Body& a = bodies[earliest.first];
        Body& b = bodies[earliest.second];
        if (applyImpulse(a, b, earliest.normal)) {
            ++stats.impulses;
            recordImpulse(a, b, stats);
          
            if ((a.markedWall || b.markedWall) && stats.wallContactTimeMs < 0)
                stats.wallContactTimeMs = (dt - remaining) * 1000.0;
        }
        ++stats.ccdEvents;
        const double inv = a.invMass + b.invMass;
        if (inv > 0) {
            a.position -= earliest.normal * (1e-6 * a.invMass / inv);
            b.position += earliest.normal * (1e-6 * b.invMass / inv);
        }
    }
    if (remaining > 0) advance(bodies, remaining);
    stats.impulses += solveOverlaps(bodies, 12, PairPass::All, &stats);
    stats.normalSteps = 1;
    stats.fastSteps = 0;
}

World runNormalDemo(Scene scene) { return World(scene, Mode::Discrete); }
World runMultiRateDemo(Scene scene) { return World(scene, Mode::MultiRate); }
World runCCDDemo(Scene scene) { return World(scene, Mode::CCD); }
World runStaticNormalFastDemo(Mode mode) { return World(Scene::Platforms, mode); }
World runNormalDemo() { return runNormalDemo(Scene::FastNormal); }
World runMultiRateDemo() { return runMultiRateDemo(Scene::Platforms); }
World runCCDDemo() { return runCCDDemo(Scene::ThinWall); }
World runStaticNormalFastDemo() { return runStaticNormalFastDemo(Mode::MultiRate); }
} 
