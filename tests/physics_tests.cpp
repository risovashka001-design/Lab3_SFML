#include "Body.h"
#include "World.h"
#include "Scenes.h"
#include "Collisions.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace lab3;
namespace {
int checks = 0;
void require(bool success, const std::string& message) {
    ++checks;
    if (!success) throw std::runtime_error("FAILED: " + message);
    std::cout << "[OK] " << message << '\n';
}
void run(World& world, int steps) {
    for (int i = 0; i < steps; ++i) world.step();
}
bool sameBody(const Body& a, const Body& b) {
    return a.position.x == b.position.x && a.position.y == b.position.y &&
           a.velocity.x == b.velocity.x && a.velocity.y == b.velocity.y;
}
bool finite(const Body& b) {
    return std::isfinite(b.position.x) && std::isfinite(b.position.y) &&
           std::isfinite(b.velocity.x) && std::isfinite(b.velocity.y);
}
void checkCollider(Mode mode, Scene scene, int steps, const std::string& label) {
    World world(scene, mode);
    run(world, steps);
    require(world.bodies[0].velocity.x < -1 && world.bodies[1].velocity.x > 1,
            label + " both velocities changed [" + modeName(mode) + "]");
}
}

int main() {
    try {
 
        for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World demo(Scene::ThinWall, mode);
            DisplayFrameStats screen;
            screen.clear();
            require(screen.normalSteps == 0 && screen.fastSubsteps == 0,
                "rendered frame with zero physics updates displays 0 substeps ["+modeName(mode)+"]");
            demo.step(); screen.add(demo.stats);
            const int perTick = mode == Mode::MultiRate ? FAST_HZ / NORMAL_HZ : 0;
            require(screen.normalSteps == 1 && screen.fastSubsteps == perTick,
                "one simulated tick contributes accurate per-render-frame steps ["+modeName(mode)+"]");
            demo.step(); screen.add(demo.stats);
            require(screen.normalSteps == 2 && screen.fastSubsteps == 2 * perTick,
                "two simulated ticks accumulate all substeps, not last tick only ["+modeName(mode)+"]");
            screen.clear();
            require(screen.normalSteps == 0 && screen.fastSubsteps == 0,
                "render frame counters reset before next image ["+modeName(mode)+"]");
        }
        {
            World noFast(Scene::Stack, Mode::MultiRate);
            DisplayFrameStats screen;
            noFast.step(); screen.add(noFast.stats);
            require(screen.normalSteps == 1 && screen.fastSubsteps == 0,
                "multi-rate scene with no fast group does not invent 240 Hz substeps");
        }

    
        for (Mode method : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            const std::string modeLabel = modeName(method);
            {
                World wall(Scene::ThinWall, method);
                bool floorBounce = false;
                double peakLift = 0.0;
                double previousVerticalVelocity = 0.0;
                for (int tick = 0; tick < 180; ++tick) {
                    previousVerticalVelocity = wall.bodies[0].velocity.y;
                    wall.step();
                    const Body& projectile = wall.bodies[0];
                    const double restingY = 330.0 - projectile.halfSize.y;
                    if (previousVerticalVelocity > 20.0 && projectile.velocity.y < -20.0 &&
                        projectile.position.y >= restingY - 2.0) floorBounce = true;
                    if (floorBounce)
                        peakLift = std::max(peakLift, restingY - projectile.position.y);
                }
                require(floorBounce, "wall projectile has a genuine floor-rebound impulse ["+modeLabel+"]");
                require(peakLift > 0.5, "wall projectile visibly rises after bouncing ["+modeLabel+"]");
                const Body& end = wall.bodies[0];
                require(std::abs(end.position.y + end.halfSize.y - 330.0) < 0.3 &&
                        std::abs(end.velocity.y) < 2.0,
                        "wall projectile settles on the actual floor ["+modeLabel+"]");
            }
            {
                World supports(Scene::Supports, method);
                bool bounced = false;
                for (int tick = 0; tick < 480; ++tick) {
                    supports.step();
                    for (const Body& body : supports.bodies)
                        if (body.kind == Kind::Normal && body.velocity.y < -25.0)
                            bounced = true;
                }
                require(bounced, "blue blocks rebound gently from supports/one another ["+modeLabel+"]");
                bool allGroundedAndFlush = true;
                for (int col = 0; col < 5; ++col) {
                    const std::size_t i = 3 + 4 * col;
                    const Body& low  = supports.bodies[i+1];
                    const Body& mid  = supports.bodies[i+2];
                    const Body& high = supports.bodies[i+3];
                    if (std::abs(low.position.y - 275.0) > 0.25 ||
                        std::abs(mid.position.y - 253.0) > 0.25 ||
                        std::abs(high.position.y - 231.0) > 0.25 ||
                        std::abs(mid.position.y - low.position.y + 22.0) > 0.25 ||
                        std::abs(high.position.y - mid.position.y + 22.0) > 0.25 ||
                        std::abs(low.velocity.y) > 2.0 ||
                        std::abs(mid.velocity.y) > 2.0 ||
                        std::abs(high.velocity.y) > 2.0)
                        allGroundedAndFlush = false;
                }
                require(allGroundedAndFlush,
                    "five blue columns settle flush without physical gaps ["+modeLabel+"]");
            }
            {
                World impact(Scene::StackImpact, method);
                run(impact, 240); 
                const Body& pink = impact.bodies[impact.fastGroup.front()];
                require(impact.stackProjectileLaunched && impact.impactOccurred,
                    "stack-impact projectile fires once and hits a dynamic block ["+modeLabel+"]");
                require(std::abs(pink.position.y + pink.halfSize.y - 330.0) < 0.25 &&
                        std::abs(pink.velocity.y) < 1.0 && std::abs(pink.velocity.x) < 1.0,
                    "stack-impact projectile falls to actual ground and stops ["+modeLabel+"]");
                const Vec2 restPosition = pink.position;
                run(impact, 120);
                const Body& after = impact.bodies[impact.fastGroup.front()];
                require(std::hypot(after.position.x - restPosition.x,
                                   after.position.y - restPosition.y) < 0.2,
                    "stack-impact projectile stays landed without re-launch ["+modeLabel+"]");
                require(impact.stackRestConfirmed,
                    "blue stack stays stable after the actual mixed collision ["+modeLabel+"]");
            }
        }

    
        {
            World gravityTest(Scene::FastNormal, Mode::MultiRate);
            gravityTest.bodies[0].velocity = {0, 0};
            gravityTest.bodies[1].velocity = {0, 0};
            const double initialY = gravityTest.bodies[0].position.y;
            gravityTest.step();
            const int n = FAST_HZ / NORMAL_HZ;
            const double h = 1.0 / FAST_HZ;
            require(gravityTest.microTrace.size() == static_cast<std::size_t>(n),
                    "fast gravity: one trace per 240-Hz substep");
            for (int i = 0; i < n; ++i) {
                const double expectedVy = (i + 1) * GRAVITY * h;
                require(std::abs(gravityTest.microTrace[i].fastVelocity.y - expectedVy) < 1e-8,
                        "fast gravity: vertical speed advances by g/240 at each microstep");
            }
            const double expectedY = initialY + GRAVITY * h * h * n * (n + 1) / 2.0;
            require(std::abs(gravityTest.bodies[0].position.y - expectedY) < 1e-7,
                    "fast gravity: position uses four successive microstep velocities");
            require(std::abs(gravityTest.bodies[1].velocity.y - GRAVITY / NORMAL_HZ) < 1e-8,
                    "ordinary gravity: normal group remains on its 60-Hz update");
        }
 
        for (Scene scene : {Scene::Platforms, Scene::ThinWall, Scene::FastNormal,
                            Scene::Supports, Scene::Stack, Scene::StackImpact,
                            Scene::NormalNormal, Scene::FastFast}) {
            World normal(scene, Mode::Discrete);
            World rate(scene, Mode::MultiRate);
            World ccd(scene, Mode::CCD);
            require(normal.bodies.size() == rate.bodies.size() &&
                    normal.bodies.size() == ccd.bodies.size(),
                    "mode comparison: same body count at start");
            for (std::size_t i = 0; i < normal.bodies.size(); ++i) {
                const Body& a=normal.bodies[i];
                for (const Body* body : {&rate.bodies[i], &ccd.bodies[i]}) {
                    require(body->kind == a.kind &&
                            body->halfSize.x == a.halfSize.x && body->halfSize.y == a.halfSize.y &&
                            body->position.x == a.position.x && body->position.y == a.position.y &&
                            body->velocity.x == a.velocity.x && body->velocity.y == a.velocity.y &&
                            body->invMass == a.invMass && body->restitution == a.restitution &&
                            body->floorRestitution == a.floorRestitution,
                            "mode comparison: each corresponding body has identical initial properties");
                }
            }
        }

        const auto mainBodies = makeScene(Scene::Platforms);
        require(countKind(mainBodies, Kind::Static) == 6, "main has 6 static bodies");
        require(countKind(mainBodies, Kind::Normal) == 20, "main has 20 ordinary bodies");
        require(countKind(mainBodies, Kind::Fast) == 3, "main has 3 fast bodies");
        for (const auto& b : mainBodies) {
            if (b.kind == Kind::Static) require(b.halfSize.x == 22 && b.halfSize.y == 22,
                                                "static 44x44 is double ordinary 22x22");
        }
        const auto stackBodies = makeScene(Scene::Stack);
        require(countKind(stackBodies, Kind::Static) == 4, "stack has four static supports");
        require(countKind(stackBodies, Kind::Normal) == 15, "stack has 15 ACTUAL dynamic bodies");
        require(static_cast<int>(makeScene(Scene::ThinWall)[1].halfSize.x * 2) == 10,
                "wall is 10 px thick");
        require(makeScene(Scene::ThinWall)[0].velocity.x == 2000.0,
                "projectile speed is 2000 px/sec");
        require(makeScene(Scene::ThinWall)[1].markedWall,
                "the 10-px reference wall is marked for honest hit tracking");
        const auto mainSpawn = makeScene(Scene::Platforms);
        int raised = 0;
        for (const auto& body : mainSpawn) if (body.kind == Kind::Normal) {
            
            require(body.position.y < 135, "ordinary cube STARTS ABOVE supports, not already resting");
            ++raised;
        }
        require(raised == 20, "all twenty ordinary cubes start in the air");
        for (const auto& body : mainSpawn) if (body.kind == Kind::Normal)
            require(body.restitution >= 0.20 && body.restitution <= 0.35,
                "main ordinary cubes have gentle, non-explosive restitution");
        for (const auto& body : stackBodies) if (body.kind == Kind::Normal)
            require(body.restitution <= 0.10,
                "resting stack uses low restitution to suppress jitter");
        for (std::size_t i = 0; i < mainSpawn.size(); ++i)
            if (mainSpawn[i].kind == Kind::Normal)
                for (std::size_t j = i+1; j < mainSpawn.size(); ++j)
                    if (mainSpawn[j].kind == Kind::Normal) {
                        const auto& a=mainSpawn[i]; const auto& b=mainSpawn[j];
                        const bool separated = std::abs(a.position.x-b.position.x) > 22 ||
                                               std::abs(a.position.y-b.position.y) > 22;
                        require(separated, "normal blocks have visible gaps at time zero");
                    }
       
        for (int row = 0; row < 5; ++row) {
            for (int col = 0; col < 2; ++col) {
                const auto& left = stackBodies[7 + row * 3 + col]; 
                const auto& right = stackBodies[7 + row * 3 + col + 1];
                require(std::abs(right.position.x - left.position.x - 22.0) < 1e-9,
                    "stack columns touch exactly: no horizontal gap");
            }
            if (row > 0) {
                const auto& lower = stackBodies[7 + (row-1)*3];
                const auto& upper = stackBodies[7 + row*3];
                require(lower.position.y - upper.position.y > 30.0,
                    "stack rows begin at distinct heights and fall separately");
            }
        }
    
        for (Mode method : {Mode::Discrete,Mode::MultiRate,Mode::CCD}) {
            World stack(Scene::Stack,method);
            for(int i=0;i<360;++i) stack.step();
            for(int row=0;row<5;++row)for(int col=0;col<3;++col){
                const auto& cube=stack.bodies[7+row*3+col];
                require(std::abs(cube.position.x-(178+col*22))<0.15,
                    "settled block has no sideways spacing");
                if(row>0){
                    const auto& below=stack.bodies[7+(row-1)*3+col];
                    require(std::abs((below.position.y-cube.position.y)-22)<0.16,
                        "settled stack rows have no vertical gaps");
                }
            }
        }
  
        for (Mode method : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World wall(Scene::ThinWall, method);
            int contacts = 0;
            int contactMicrostep = 0;
            double ccdMs = -1;
            for (int i = 0; i < 6; ++i) {
                wall.step();
                contacts += wall.stats.wallImpulses;
                if (wall.stats.wallContactSubstep)
                    contactMicrostep = wall.stats.wallContactSubstep;
                if (wall.stats.wallContactTimeMs >= 0)
                    ccdMs = wall.stats.wallContactTimeMs;
            }
            if (method == Mode::Discrete)
                require(contacts == 0, "60 Hz control misses the thin reference wall");
            else
                require(contacts > 0, "Multi-rate / CCD detect contact with reference wall");
            if (method == Mode::MultiRate)
                require(contactMicrostep >= 1 && contactMicrostep <= FAST_HZ/NORMAL_HZ,
                    "Multi-rate records which real substep detected the wall");
            if (method == Mode::CCD)
                require(ccdMs >= 0 && ccdMs <= 1000.0/NORMAL_HZ,
                    "CCD records collision time within the current physics step");
        }
    
        World air(Scene::ThinWall, Mode::Discrete);
        air.bodies[0].velocity.y=80.0;
        air.step();
        require(air.bodies[0].velocity.x < 2000.0 && air.bodies[0].velocity.x > 1700.0,
                "air resistance decreases horizontal speed smoothly, not by abrupt 0.4 multiplier");
        require(air.bodies[0].velocity.y > 0 && air.bodies[0].velocity.y < 80.0 + GRAVITY / NORMAL_HZ,
                "air resistance acts on vertical velocity as well as horizontal");
        require(FAST_HZ % NORMAL_HZ == 0 && FAST_HZ >= 2 * NORMAL_HZ,
                "fast frequency is an integer multiple of ordinary frequency");

        for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World wall(Scene::ThinWall, mode);
            run(wall, 6);
            if (mode == Mode::Discrete) {
                require(wall.bodies[0].velocity.x > 0 && wall.bodies[0].position.x > 220,
                        "control 60 Hz TUNNELS through the wall");
            } else {
                require(wall.bodies[0].velocity.x < 0 && wall.bodies[0].position.x < 205,
                        "Multi-rate / CCD bounces against wall [" + modeName(mode) + "]");
            }
            if (mode == Mode::MultiRate) {
                require(wall.stats.normalSteps == 1, "multi-rate 1 outer 60 Hz step");
                require(wall.stats.fastSteps == FAST_HZ / NORMAL_HZ,
                        "multi-rate performs FAST_HZ/NORMAL_HZ microsteps");
            }
            if (mode == Mode::CCD) {
                World firstImpact(Scene::ThinWall, Mode::CCD);
                run(firstImpact, 6);
         
                require(firstImpact.bodies[0].velocity.x < 0, "CCD resolves time-of-impact");
            }
            checkCollider(mode, Scene::FastNormal, 13, "fast-normal");
            checkCollider(mode, Scene::NormalNormal, 45, "normal-normal");
            checkCollider(mode, Scene::FastFast, 20, "fast-fast");

            World before(Scene::Platforms, mode);
            const auto fixed = before.bodies;
            run(before, 120);
            for (std::size_t id = 0; id < fixed.size(); ++id) {
                if (fixed[id].kind == Kind::Static || fixed[id].kind == Kind::Boundary)
                    require(sameBody(fixed[id], before.bodies[id]),
                            "static body stays still [" + modeName(mode) + "]");
            }
            bool finiteWorld = true;
            for (const auto& b : before.bodies) finiteWorld &= finite(b);
            require(finiteWorld, "no NaN/inf [" + modeName(mode) + "]");
            World a(Scene::Platforms, mode), b(Scene::Platforms, mode);
            run(a, 160); run(b, 160);
            bool deterministic = a.bodies.size() == b.bodies.size();
            for (std::size_t i = 0; i < a.bodies.size(); ++i)
                deterministic &= sameBody(a.bodies[i], b.bodies[i]);
            require(deterministic, "same inputs produce identical outputs [" + modeName(mode) + "]");

            World stack(Scene::Stack, mode);
            run(stack, 300); 
            const auto settled = stack.bodies;
            double maxDrift = 0.0, maxSpeed = 0.0;
            for (int i = 0; i < 60; ++i) {
                stack.step();
                for (std::size_t j = 0; j < stack.bodies.size(); ++j) {
                    if (stack.bodies[j].kind != Kind::Normal) continue;
                    const Body& body = stack.bodies[j];
                    maxDrift = std::max(maxDrift, std::hypot(
                        body.position.x - settled[j].position.x,
                        body.position.y - settled[j].position.y));
                    maxSpeed = std::max(maxSpeed, std::hypot(body.velocity.x, body.velocity.y));
                }
            }
            std::cout << "STACK " << modeName(mode) << ": max last-second drift " <<
                maxDrift << " px; peak speed " << maxSpeed << " px/s\n";
            require(maxDrift < 1.0, "no visible stack jitter or explosion [" + modeName(mode) + "]");
            require(maxSpeed < 15, "low resting-stack speed [" + modeName(mode) + "]");
        }

       
        for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World wall(Scene::ThinWall, mode);
            const auto before = wall.bodies;
            require(countKind(wall.bodies, Kind::Boundary) == 3,
                "wall demo has a physical floor and side boundaries");
            run(wall, 6);
            if (mode == Mode::Discrete) {
                require(wall.bodies[0].position.x > 222 && wall.bodies[0].velocity.x > 0,
                    "wall: 60Hz still tunnels even with symmetric air resistance");
            } else {
                require(wall.bodies[0].position.x < 205 && wall.bodies[0].velocity.x < 0,
                    "wall: multi-rate and CCD still reflect before falling");
            }
            run(wall, 114); 
            require(wall.bodies[0].position.y > before[0].position.y + 100,
                "wall: fast projectile falls under gravity ["+modeName(mode)+"]");
            require(std::abs(wall.bodies[0].position.y - 324.0) < 1.0,
                "wall: projectile settles on floor ["+modeName(mode)+"]");
            require(std::abs(wall.bodies[0].velocity.y) < 1.0,
                "wall: projectile no longer bounces endlessly ["+modeName(mode)+"]");
            require(sameBody(before[1],wall.bodies[1]),
                "wall: immovable barrier stays fixed ["+modeName(mode)+"]");
            require(wall.stats.fastSteps == (mode == Mode::MultiRate ? FAST_HZ/NORMAL_HZ : 0),
                "wall: honest 240Hz substep counter ["+modeName(mode)+"]");
        }

       
        for (Scene pair : {Scene::FastNormal, Scene::NormalNormal, Scene::FastFast}) {
            for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
                World falling(pair, mode);
                const double firstY = falling.bodies[0].position.y;
                const double secondY = falling.bodies[1].position.y;
                require(!falling.pairGravityActive,
                        "pair starts without falling until collision ["+modeName(mode)+"]");
                run(falling, 95);
                require(falling.pairGravityActive,
                        "pair physically collided and unlocked gravity ["+modeName(mode)+"]");
                require(falling.bodies[0].position.y > firstY + 40 &&
                        falling.bodies[1].position.y > secondY + 40,
                        "BOTH dynamic bodies fall after collision ["+modeName(mode)+"]");
                require(finite(falling.bodies[0]) && finite(falling.bodies[1]),
                        "falling pair stays finite ["+modeName(mode)+"]");
            }
        }
       
        require(runNormalDemo(Scene::ThinWall).mode == Mode::Discrete, "runNormalDemo entry point");
        require(runMultiRateDemo(Scene::ThinWall).mode == Mode::MultiRate, "runMultiRateDemo entry point");
        require(runCCDDemo(Scene::ThinWall).mode == Mode::CCD, "runCCDDemo entry point");
        require(runStaticNormalFastDemo(Mode::MultiRate).scene == Scene::Platforms,
                "runStaticNormalFastDemo entry point");
        require(runNormalDemo().mode == Mode::Discrete, "runNormalDemo() from video");
        require(runMultiRateDemo().mode == Mode::MultiRate, "runMultiRateDemo() from video");
        require(runStaticNormalFastDemo().scene == Scene::Platforms,
                "runStaticNormalFastDemo() from video");

        World mixed(Scene::FastNormal, Mode::MultiRate);
        std::ofstream trace("synchronization_trace.csv");
        trace << "same_time_s,fast_x,ordinary_x,fast_vx,ordinary_vx\n";
        bool collisionFound = false;
        for (int frame = 0; frame < 18; ++frame) {
            mixed.step();
            if (!collisionFound && mixed.bodies[1].velocity.x > 0) {
                collisionFound = true;
                require(mixed.microTrace.size() == static_cast<std::size_t>(FAST_HZ / NORMAL_HZ),
                        "synchronized time samples for every microstep");
                bool movingOnMicrostep = false;
                for (std::size_t i = 0; i < mixed.microTrace.size(); ++i) {
                    const auto& sample = mixed.microTrace[i];
                    if (i) {
                        require(std::abs(sample.time - mixed.microTrace[i - 1].time - 1. / FAST_HZ) < 1e-9,
                                "microstep timestamps differ by 1/240 s");
                    }
                    if (sample.normalVelocity.x > 0 && i + 1 < mixed.microTrace.size())
                        movingOnMicrostep = true;
                    trace << std::setprecision(12) << sample.time << ',' <<
                        sample.fastPosition.x << ',' << sample.normalPosition.x << ',' <<
                        sample.fastVelocity.x << ',' << sample.normalVelocity.x << '\n';
                }
                if (FAST_HZ / NORMAL_HZ > 2)
                    require(movingOnMicrostep,
                            "ordinary velocity changed before end of 60Hz tick");
                require(mixed.bodies[0].velocity.x < 0 && mixed.bodies[1].velocity.x > 0,
                        "fast and ordinary exchange impulse at same time");
            }
        }
        require(collisionFound, "actual mixed-rate collision was traced");
        trace.close();

       
        for (Mode m : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World supports(Scene::Supports, m);
            require(countKind(supports.bodies, Kind::Static)==5, "five video-style static pillars exist");
            require(countKind(supports.bodies, Kind::Normal)==15, "fifteen falling dynamic blocks exist");
            const double initialY = supports.bodies[4].position.y;
            const double fixedY = supports.bodies[3].position.y;
            run(supports, 180);
            require(supports.bodies[4].position.y > initialY + 30,
                    "body falls onto static obstacle");
            require(supports.bodies[3].position.y == fixedY,
                    "static obstacle never moves");
        }

        
        for (Mode method : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World scene(Scene::Platforms, method);
            int mixedContacts=0, dynamicContacts=0, fixedContacts=0;
            for (int i=0; i<180; ++i) {
                scene.step();
                mixedContacts += scene.stats.fastNormal;
                dynamicContacts += scene.stats.dynamicDynamic;
                fixedContacts += scene.stats.dynamicStatic;
            }
            require(mixedContacts>0, "main scene has REAL fast-normal collision ["+modeName(method)+"]");
            require(dynamicContacts>0 && fixedContacts>0,
                "main scene checks moving-moving and moving-static ["+modeName(method)+"]");
        }

     
        std::ofstream impact("impact_stability.csv");
        impact << "mode,fast_normal_impulses,normal_and_fast_dynamic_impulses,"
                  "dynamic_static_impulses,maximum_stack_displacement_px,"
                  "peak_stack_speed_px_s,last_second_drift_px,last_second_speed_px_s\n";
        for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World world(Scene::StackImpact, mode);
            const auto initial = world.bodies;
            require(countKind(world.bodies, Kind::Static) == 4, "impact: four actual static supports");
            require(countKind(world.bodies, Kind::Normal) == 15, "impact: 15 actual dynamic blocks");
            require(countKind(world.bodies, Kind::Fast) == 1, "impact: one fast moving projectile");
            run(world, 120);
            require(world.bodies.back().velocity.x == 0,
                    "impact: stack settles before projectile launches [" + modeName(mode) + "]");
            const auto before = world.bodies;
            int mixed = 0, dynamic = 0, statics = 0;
            double maximumDisplacement = 0, peakSpeed = 0, maxLateDrift = 0, latePeakSpeed = 0;
            std::vector<Body> beforeLastSecond;
            for (int frame = 0; frame < 240; ++frame) {
                if (frame == 180) beforeLastSecond = world.bodies;
                world.step();
                mixed += world.stats.fastNormal;
                dynamic += world.stats.dynamicDynamic;
                statics += world.stats.dynamicStatic;
                for (std::size_t j = 0; j < world.bodies.size(); ++j) {
                    const Body& b = world.bodies[j];
                    if (!finite(b)) throw std::runtime_error("Non-finite state after stack impact");
                    if (initial[j].kind == Kind::Static || initial[j].kind == Kind::Boundary) {
                        if (!sameBody(b, initial[j])) throw std::runtime_error("static object moved after impact");
                    }
                    if (b.kind != Kind::Normal) continue;
                    const Vec2 delta = b.position - before[j].position;
                    maximumDisplacement = std::max(maximumDisplacement, std::hypot(delta.x,delta.y));
                    peakSpeed = std::max(peakSpeed, std::hypot(b.velocity.x,b.velocity.y));
                    if (frame >= 180) {
                        const Vec2 tail = b.position - beforeLastSecond[j].position;
                        maxLateDrift = std::max(maxLateDrift, std::hypot(tail.x,tail.y));
                        latePeakSpeed = std::max(latePeakSpeed, std::hypot(b.velocity.x,b.velocity.y));
                    }
                }
            }
            require(mixed >= 1, "actual fast-normal impact against stack [" + modeName(mode) + "]");
            require(dynamic > 0, "actual dynamic-dynamic contact in impacted stack [" + modeName(mode) + "]");
            require(statics > 0, "actual dynamic-static contact at supports [" + modeName(mode) + "]");
            require(maximumDisplacement > 0.05 && maximumDisplacement < 18.0,
                "stack moves measurably but does not explode [" + modeName(mode) + "]");
            require(peakSpeed < 250.0, "impact does not create unrealistic block velocities [" + modeName(mode) + "]");
            require(maxLateDrift < 1.0 && latePeakSpeed < 15.0,
                "impacted stack settles without visible jitter [" + modeName(mode) + "]");
            std::cout << "IMPACT " << modeName(mode) << ": shift " << maximumDisplacement <<
                " px, peak " << peakSpeed << " px/s, tail drift " << maxLateDrift << " px, mixed " << mixed << "\n";
            impact << modeName(mode) << ',' << mixed << ',' << dynamic << ',' << statics << ',' <<
                maximumDisplacement << ',' << peakSpeed << ',' << maxLateDrift << ',' << latePeakSpeed << '\n';
            World replay(Scene::StackImpact, mode);
            run(replay, 360);
            bool same = replay.bodies.size() == world.bodies.size();
            for (std::size_t j=0; j<replay.bodies.size(); ++j) same &= sameBody(replay.bodies[j],world.bodies[j]);
            require(same,"impact simulation deterministic [" + modeName(mode) + "]");
        }
        impact.close();

  
        for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World rested(Scene::StackImpact,mode);
            require(!rested.stackRestConfirmed && !rested.impactOccurred,
                    "impact rest not fabricated at launch ["+modeName(mode)+"]");
            for(int tick=0;tick<360;++tick) rested.step();
            require(rested.impactOccurred,
                    "rest is checked after an actual mixed dynamic impact ["+modeName(mode)+"]");
            require(rested.stackRestConfirmed,
                    "one continuous second of rest verified after stack impact ["+modeName(mode)+"]");
            require(rested.stackRestDurationSeconds >= 1.0 - 1e-8 &&
                    rested.stackRestDriftPx <= 0.35 && rested.stackRestPeakSpeed <= 5.0,
                    "position and velocity measured throughout quiet second ["+modeName(mode)+"]");
            World quiet(Scene::Stack,mode);
            for(int tick=0;tick<360;++tick) quiet.step();
            require(quiet.stackRestConfirmed && quiet.stackRestDriftPx <= 0.35,
                    "unperturbed 15-block stack confirms rest too ["+modeName(mode)+"]");
        }

   
        for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World w(Scene::ThinWall, mode);
            double interactionTime = -1.0, floorTime = -1.0;
            for (int i = 0; i < 150; ++i) {
                w.step();
                if (interactionTime < 0 && w.wallEventResolved)
                    interactionTime = w.timeSeconds;
                if (interactionTime >= 0 && w.bodies[0].position.y >= 322.0) {
                    floorTime = w.timeSeconds;
                    break;
                }
            }
            require(interactionTime > 0 && interactionTime < 0.20,
                "wall experiment preserves the initial fast projectile [" + modeName(mode) + "]");
            require(floorTime > 0 && floorTime - interactionTime < 0.95,
                "projectile falls promptly after wall contact or tunnelling [" + modeName(mode) + "]");
        }


        for (Scene scene : {Scene::Stack, Scene::StackImpact}) {
            for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
                World w(scene, mode);
                run(w, 360);
                double minX=1e8, maxX=-1e8, minY=1e8, maxY=-1e8;
                int n=0;
                for (const auto& b : w.bodies) if (b.kind == Kind::Normal) {
                    minX=std::min(minX, b.position.x-b.halfSize.x);
                    maxX=std::max(maxX, b.position.x+b.halfSize.x);
                    minY=std::min(minY, b.position.y-b.halfSize.y);
                    maxY=std::max(maxY, b.position.y+b.halfSize.y);
                    ++n;
                }
                require(w.stackRestConfirmed && n == 15,
                    "merged display requires 15 physically resting bodies [" + modeName(mode) + "]");
                require(maxX-minX < 71.0 && maxY-minY < 112.0,
                    "resting stack has no large physical gaps [" + modeName(mode) + "]");
            }
        }

       
        const double physicalFloorY = 330.0;
        for (Scene scene : {Scene::Platforms, Scene::Supports, Scene::Stack,
                            Scene::StackImpact}) {
            const auto initial = makeScene(scene);
            bool anchored = true;
            for (const auto& a : initial) if (a.kind == Kind::Static) {
                bool support = std::abs((a.position.y + a.halfSize.y) - physicalFloorY) < 1e-6;
                for (const auto& below : initial) if (&a != &below && below.kind == Kind::Static) {
                    const double xOverlap = a.halfSize.x + below.halfSize.x -
                                            std::abs(a.position.x - below.position.x);
                    if (xOverlap > 0.1 && std::abs(a.position.y + a.halfSize.y -
                                                   (below.position.y - below.halfSize.y)) < 1e-6)
                        support = true;
                }
                anchored &= support;
            }
            require(anchored, "static supports have a grounded physical connection [" +
                    std::to_string(static_cast<int>(scene)) + "]");
        }

   
        for (Scene scene : {Scene::Platforms, Scene::Supports, Scene::Stack,
                            Scene::StackImpact, Scene::ThinWall,
                            Scene::FastNormal, Scene::NormalNormal, Scene::FastFast}) {
            for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
                World w(scene, mode);
                run(w, 540);  
                bool allSupported = true;
                int dynamicCount = 0;
                for (std::size_t i = 0; i < w.bodies.size(); ++i) {
                    const Body& a = w.bodies[i];
                    if (!a.active || !isMoving(a)) continue;
                    ++dynamicCount;
                    bool supported = false;
                    for (std::size_t j = 0; j < w.bodies.size(); ++j) {
                        if (i == j || !w.bodies[j].active) continue;
                        const Body& below = w.bodies[j];
                        const double bottom = a.position.y + a.halfSize.y;
                        const double top = below.position.y - below.halfSize.y;
                        const double overlapX = a.halfSize.x + below.halfSize.x -
                                                std::abs(a.position.x - below.position.x);
                        if (below.position.y > a.position.y && overlapX > 0.05 &&
                            std::abs(bottom-top) < 2.0) supported = true;
                    }
                    allSupported &= supported;
                }
                require(dynamicCount > 0 && allSupported,
                    "all moving bodies land on real surfaces (no hovering) [" +
                    std::to_string(static_cast<int>(scene)) + ", " + modeName(mode) + "]");
            }
        }

    
        for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
            World w(Scene::Stack, mode);
            run(w, 360);
            bool compact = true;
            const int start = 7; 
            for (int row = 0; row < 5; ++row) {
                for (int col = 0; col < 3; ++col) {
                    const Body& b = w.bodies[start + row*3 + col];
                    if (col < 2) {
                        const Body& right = w.bodies[start + row*3 + col + 1];
                        compact &= std::abs((right.position.x-b.position.x)-22.0) < 0.20;
                    }
                    if (row < 4) {
                        const Body& above = w.bodies[start + (row+1)*3 + col];
                        compact &= std::abs((b.position.y-above.position.y)-22.0) < 0.25;
                    }
                }
            }
            require(compact, "15 separate stack colliders settle edge-to-edge [" + modeName(mode) + "]");
        }

   
        std::ofstream bench("benchmark.csv");
        bench << "scene,mode,frames,mean_cpu_physics_ms_per_frame,mean_240_substeps_per_frame,"
                 "dynamic_dynamic_impulses,dynamic_static_impulses,mixed_impulses,ccd_events\n";
        for (Scene scene : {Scene::ThinWall, Scene::FastNormal, Scene::Platforms,
                            Scene::Stack, Scene::StackImpact}) {
            for (Mode mode : {Mode::Discrete, Mode::MultiRate, Mode::CCD}) {
                World world(scene, mode);
                constexpr int iterations = 360;
                long long steps240 = 0, dd = 0, ds = 0, mixed = 0, events = 0;
                const auto start = std::chrono::steady_clock::now();
                for (int i = 0; i < iterations; ++i) {
                    world.step();
                    steps240 += mode == Mode::MultiRate ? world.stats.fastSteps : 0;
                    dd += world.stats.dynamicDynamic;
                    ds += world.stats.dynamicStatic;
                    mixed += world.stats.fastNormal;
                    events += world.stats.ccdEvents;
                }
                const auto end = std::chrono::steady_clock::now();
                const double ms = std::chrono::duration<double, std::milli>(end - start).count() / iterations;
                std::cout << "BENCHMARK " << sceneName(scene) << " / " << modeName(mode) <<
                    ": " << std::fixed << std::setprecision(5) << ms << " CPU ms/sim-frame\n";
                bench << sceneName(scene) << ',' << modeName(mode) << ',' << iterations << ',' <<
                    std::setprecision(7) << ms << ',' <<
                    static_cast<double>(steps240) / iterations << ',' << dd << ',' << ds << ',' << mixed << ',' << events << '\n';
            }
        }
        std::cout << "ALL " << checks << " TESTS PASSED.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
