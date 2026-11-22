#pragma once
#include <vector>

namespace lab3 {

constexpr int NORMAL_HZ = 60;
constexpr int FAST_HZ = 240;
static_assert(FAST_HZ % NORMAL_HZ == 0, "FAST_HZ must be a multiple of NORMAL_HZ");
constexpr double GRAVITY = 560.0;

struct Vec2 {
    double x = 0, y = 0;
    Vec2 operator+(Vec2 v) const { return {x + v.x, y + v.y}; }
    Vec2 operator-(Vec2 v) const { return {x - v.x, y - v.y}; }
    Vec2 operator*(double k) const { return {x * k, y * k}; }
    Vec2& operator+=(Vec2 v) { x += v.x; y += v.y; return *this; }
    Vec2& operator-=(Vec2 v) { x -= v.x; y -= v.y; return *this; }
};

enum class Kind { Boundary, Static, Normal, Fast };
enum class Mode { Discrete, MultiRate, CCD };
enum class Scene { Platforms, ThinWall, FastNormal, Supports, Stack, StackImpact, NormalNormal, FastFast };

struct Body {
    Kind kind = Kind::Static;
    Vec2 position{}, velocity{}, halfSize{};
    double invMass = 0; 
    double restitution = 0;
    double floorRestitution = -1.0; 
    bool markedWall = false; 
    bool active = true;       
    bool timedGate = false;   
};

struct StepStats {
    int normalSteps = 0;
    int fastSteps = 0;
    int impulses = 0;
    int ccdEvents = 0;
    int dynamicStatic = 0;     
    int dynamicDynamic = 0;    
    int fastNormal = 0;        
    int wallImpulses = 0;      
    int wallContactSubstep = 0; 
    double wallContactTimeMs = -1; 
};


struct TraceRecord {
    double time = 0;
    Vec2 fastPosition{}, normalPosition{};
    Vec2 fastVelocity{}, normalVelocity{};
};
inline bool isMoving(const Body& b) { return b.invMass > 0; }
} 
