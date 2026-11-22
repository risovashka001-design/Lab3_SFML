#pragma once
#include "Body.h"

namespace lab3 {

struct DisplayFrameStats {
    int normalSteps = 0;
    int fastSubsteps = 0;
    void clear() { normalSteps = 0; fastSubsteps = 0; }
    void add(const StepStats& step) {
        normalSteps += step.normalSteps;
        fastSubsteps += step.fastSteps;
    }
};

class World {
public:
    World(Scene scene, Mode mode);
    void step(); 
    Scene scene;
    Mode mode;
    std::vector<Body> bodies;
   
    std::vector<std::size_t> normalGroup, fastGroup;
    StepStats stats{};
    std::vector<TraceRecord> microTrace; 
    double timeSeconds = 0;
 
    bool pairGravityActive = false;
    bool wallEventResolved = false; 
    bool impactOccurred = false; 
    bool stackProjectileLaunched = false; 
    bool stackRestConfirmed = false;
    double stackRestDurationSeconds = 0;
    double stackRestDriftPx = 0;
    double stackRestPeakSpeed = 0;
private:
    void evaluateStackRest();
    double restWindowStart = -1;
    std::vector<Vec2> restReferencePositions;

    void discrete(double dt);
    void multiRate(double dt);
    void continuous(double dt);
};

World runNormalDemo(Scene scene);
World runMultiRateDemo(Scene scene);
World runCCDDemo(Scene scene);
World runStaticNormalFastDemo(Mode mode);

World runNormalDemo();
World runMultiRateDemo();
World runCCDDemo();
World runStaticNormalFastDemo();
} 
