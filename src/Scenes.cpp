#include "Scenes.h"

namespace lab3 {
namespace {
Body box(Kind kind, double x, double y, double hx, double hy,
         double vx = 0.0, double vy = 0.0) {
    Body b;
    b.kind = kind;
    b.position = {x, y};
    b.velocity = {vx, vy};
    b.halfSize = {hx, hy};
    b.invMass = kind == Kind::Normal ? 0.25 : kind == Kind::Fast ? 1.0 : 0.0;
    b.restitution = kind == Kind::Fast ? 0.85 : kind == Kind::Normal ? 0.08 : 0.0;
    return b;
}
void addWorldBorders(std::vector<Body>& bodies) {
    
    bodies.push_back(box(Kind::Boundary, 200, 338, 200, 8));
    bodies.push_back(box(Kind::Boundary, -8, 180, 8, 180));
    bodies.push_back(box(Kind::Boundary, 408, 180, 8, 180));
}
}

std::vector<Body> makeScene(Scene scene) {
    std::vector<Body> b;
    if (scene == Scene::ThinWall) {
  
        b.push_back(box(Kind::Fast, 62, 174, 3, 6, 2000));
        b.push_back(box(Kind::Static, 210, 205, 5, 125)); 
        b[0].restitution = 0.37; 
        b[0].floorRestitution = 0.37; 
        b[1].restitution = 0.32; 
        b[1].markedWall = true;
        addWorldBorders(b);
        return b;
    }
    if (scene == Scene::FastNormal) {
        
        Body fast = box(Kind::Fast, 55, 135, 5, 8, 1550);
        Body normal = box(Kind::Normal, 245, 135, 11, 11, 0);
        fast.restitution = 0.32;   
        normal.restitution = 0.12;
        b.push_back(fast);
        b.push_back(normal);
        addWorldBorders(b);
        return b;
    }
    if (scene == Scene::NormalNormal) {
        Body left = box(Kind::Normal, 75, 140, 11, 11, 300);
        Body right = box(Kind::Normal, 325, 140, 11, 11, -300);
        left.restitution = right.restitution = 0.12;
        b.push_back(left);
        b.push_back(right);
        addWorldBorders(b);
        return b;
    }
    if (scene == Scene::FastFast) {
        Body left = box(Kind::Fast, 90, 140, 7, 7, 800);
        Body right = box(Kind::Fast, 310, 140, 7, 7, -800);
        left.restitution = right.restitution = 0.22;
        b.push_back(left);
        b.push_back(right);
        addWorldBorders(b);
        return b;
    }
    addWorldBorders(b);
    if (scene == Scene::Supports) {
   
        for (int i = 0; i < 5; ++i) {
            const double x = 75 + i * 61;
            b.push_back(box(Kind::Static, x, 308, 22, 22)); 
            Body low = box(Kind::Normal, x, 245 + i * 1, 11, 11, 0);
            Body mid = box(Kind::Normal, x, 95 + (i % 2) * 2, 11, 11, 0);
            Body high = box(Kind::Normal, x, 15 + (i % 3) * 3, 11, 11, 0);
         
            low.invMass = 0.008; mid.invMass = 0.040; high.invMass = 0.250;
            low.restitution = 0.06;
            mid.restitution = high.restitution = 0.39;
            b.push_back(low);
            b.push_back(mid);
            b.push_back(high);
        }
        return b;
    }
    if (scene == Scene::Stack || scene == Scene::StackImpact) {
 
        for (int i = 0; i < 4; ++i)
            b.push_back(box(Kind::Static, 134 + i * 44, 308, 22, 22)); 
        const double startHeights[5] = {228, 178, 127, 78, 28};
        for (int row = 0; row < 5; ++row)
            for (int col = 0; col < 3; ++col)
                b.push_back(box(Kind::Normal, 178 + col * 22, startHeights[row], 11, 11));
        if (scene == Scene::StackImpact) {
       
            Body projectile = box(Kind::Fast, 35, 212, 7, 7, 0, 0);
            projectile.restitution = 0.70; 
            projectile.floorRestitution = 0.12; 
            b.push_back(projectile);
        }
        return b;
    }

  
    const double sx[6] = {110, 110, 230, 230, 350, 350};
    const double sy[6] = {308, 264, 308, 264, 308, 264};
    for (int i = 0; i < 6; ++i)
        b.push_back(box(Kind::Static, sx[i], sy[i], 22, 22));

    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 10; ++col) {
            const double x = 76 + col * 28;
            const double y = 48 + row * 55 + (col % 3) * 4;
            Body ordinary = box(Kind::Normal, x, y, 11, 11,
                                (col % 3 - 1) * 5.0);
            ordinary.restitution = 0.25; 
            b.push_back(ordinary);
        }
    }
   
    for (int i = 0; i < 3; ++i) {
        Body fast = box(Kind::Fast, 25, 100 + i * 39, 4, 4, 280 + i * 35);
        fast.restitution = 0.40;
        b.push_back(fast);
    }
    return b;
}

const char* sceneName(Scene scene) {
    switch (scene) {
    case Scene::Platforms: return "Main experiment";
    case Scene::ThinWall: return "Fast projectile / thin wall";
    case Scene::FastNormal: return "Fast / ordinary collision";
    case Scene::Stack: return "Stable block stack";
    case Scene::StackImpact: return "Fast projectile hits 15-block stack";
    case Scene::Supports: return "Five immovable supports";
    case Scene::NormalNormal: return "Ordinary / ordinary collision";
    case Scene::FastFast: return "Fast / fast collision";
    }
    return "Unknown";
}
std::string modeName(Mode mode) {
    switch (mode) {
    case Mode::Discrete: return "60 Hz (control)";
    case Mode::MultiRate: return "Multi-rate 60/240 Hz";
    case Mode::CCD: return "CCD 60 Hz";
    }
    return "Unknown";
}
int countKind(const std::vector<Body>& bodies, Kind kind) {
    int count = 0;
    for (const auto& body : bodies)
        if (body.kind == kind) ++count;
    return count;
}
} 
