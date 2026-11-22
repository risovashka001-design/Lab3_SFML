
#include "World.h"
#include "Scenes.h"
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

using namespace lab3;
namespace {
constexpr unsigned WIN_W = 1100, WIN_H = 780;
const sf::Color BG(25, 27, 37);
const sf::Color WHITE(239, 241, 248);
const sf::Color SOFT(167, 177, 192);
const sf::Color NORMAL(83, 196, 242);       
const sf::Color FAST(224, 105, 201);      
const sf::Color STATIC(136, 133, 227);      
const sf::Color GREEN(119, 225, 169);
const sf::Color BAD(246, 123, 147);

const wchar_t* methodName(Mode mode) {
    switch (mode) {
        case Mode::Discrete: return L"Контроль: обычные 60 Гц";
        case Mode::MultiRate: return L"MULTI-RATE: обычные 60 Гц, быстрые 240 Гц";
        case Mode::CCD: return L"CCD: поиск момента столкновения (60 Гц)";
    }
    return L"";
}
const wchar_t* sceneTitle(Scene scene) {
    switch (scene) {
        case Scene::ThinWall: return L"4  |  Снаряд 2000 пикс./с — стенка 10 пикс.";
        case Scene::FastNormal: return L"5  |  Быстрый и обычный — столкновение и падение";
        case Scene::Supports: return L"6  |  Столбики и падающие тела";
        case Scene::Stack: return L"7  |  Стек из 15 кубов";
        case Scene::StackImpact: return L"7 повторно  |  Удар по стеку";
        case Scene::NormalNormal: return L"8  |  Два обычных тела";
        case Scene::FastFast: return L"9  |  Два быстрых тела";
        case Scene::Platforms: return L"0  |  Общая сцена (6/20/3)";
    }
    return L"";
}
bool loadFont(sf::Font& font) {
    const char* files[] = {
        "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/tahoma.ttf", "C:/Windows/Fonts/calibri.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    };
    for (const char* file : files) if (font.loadFromFile(file)) return true;
    return false;
}
void txt(sf::RenderTarget& target, const sf::Font& f, const std::wstring& value,
         unsigned size, float x, float y, sf::Color color=WHITE) {
    sf::Text t(sf::String(value), f, size);
    t.setPosition(x, y); t.setFillColor(color);
    target.draw(t);
}
void rect(sf::RenderTarget& target, float x, float y, float w, float h, sf::Color color) {
    sf::RectangleShape r({w,h}); r.setPosition(x,y); r.setFillColor(color); target.draw(r);
}
std::wstring fixed(double n, int precision=3) {
    std::wostringstream out; out << std::fixed << std::setprecision(precision) << n; return out.str();
}
struct App {
    Scene scene = Scene::Platforms;
    Mode mode = Mode::MultiRate;
    World world{scene, mode};
    double averageMs=0, frameMs=0, physicsFrameMs=0, accumulator=0, hold=0, speed=1.0;
    double maxBlockShift=0, peakBlockSpeed=0;
    std::vector<Vec2> preImpactPositions;
    int impulses=0, ccdEvents=0, mixedImpulses=0, dynamicImpulses=0, staticImpulses=0;
    DisplayFrameStats displayedFrame;
    bool pause=false, compare=false;
    bool wallBounced=false, wallPassed=false;
    int wallMicrostep = 0;
    double wallTimeMs = -1;
    void restart() {
     
        if (scene == Scene::Platforms) world = runStaticNormalFastDemo(mode);
        else if (mode == Mode::Discrete) world = runNormalDemo(scene);
        else if (mode == Mode::MultiRate) world = runMultiRateDemo(scene);
        else world = runCCDDemo(scene);
        averageMs=0; accumulator=0; hold=0;
        maxBlockShift=0; peakBlockSpeed=0; preImpactPositions.clear();
        wallBounced=false; wallPassed=false; wallMicrostep=0; wallTimeMs=-1;
        impulses=0; ccdEvents=0; mixedImpulses=0; dynamicImpulses=0; staticImpulses=0;
    }
    void tick() {
        auto start = std::chrono::steady_clock::now();
        world.step();
        auto end = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end-start).count();
        physicsFrameMs += ms;
        displayedFrame.add(world.stats);
        averageMs = averageMs == 0 ? ms : 0.93*averageMs + 0.07*ms;
        impulses += world.stats.impulses;
        ccdEvents += world.stats.ccdEvents;
        mixedImpulses += world.stats.fastNormal;
        dynamicImpulses += world.stats.dynamicDynamic;
        staticImpulses += world.stats.dynamicStatic;
        if (scene == Scene::ThinWall) {
            const Body& projectile = world.bodies[0];
            const Body& wall = world.bodies[1];
        
            if (world.stats.wallImpulses > 0) wallBounced = true;
            if (world.stats.wallContactSubstep > 0 && wallMicrostep == 0)
                wallMicrostep = world.stats.wallContactSubstep;
            if (world.stats.wallContactTimeMs >= 0 && wallTimeMs < 0)
                wallTimeMs = world.stats.wallContactTimeMs;
            if (!wallBounced && projectile.position.x >
                    wall.position.x + wall.halfSize.x + projectile.halfSize.x)
                wallPassed = true;
        }
        if (scene == Scene::StackImpact) {
            if (world.timeSeconds >= 2.0 - 1e-8 && preImpactPositions.empty()) {
                for (const auto& body : world.bodies) preImpactPositions.push_back(body.position);
            }
            if (!preImpactPositions.empty()) {
                for (std::size_t i=0; i<world.bodies.size(); ++i) {
                    const Body& body=world.bodies[i];
                    if (body.kind!=Kind::Normal) continue;
                    const Vec2 d=body.position - preImpactPositions[i];
                    maxBlockShift=std::max(maxBlockShift,std::hypot(d.x,d.y));
                    peakBlockSpeed=std::max(peakBlockSpeed,std::hypot(body.velocity.x,body.velocity.y));
                }
            }
        }
    }
};

double stopMoment(Scene s) {
    switch (s) {
        case Scene::ThinWall: return 2.35; 
        case Scene::FastNormal: return 3.10; 
        case Scene::NormalNormal: return 3.10;
        case Scene::FastFast: return 3.10;
        case Scene::StackImpact: return 8.00;
        default: return 0;
    }
}
void drawWorld(sf::RenderWindow& window, const App& app) {
    const World& world = app.world;
    
    const bool wallScene = world.scene == Scene::ThinWall;
    const float scale = wallScene ? 1.80f : 1.35f;
    const float originX = wallScene ? 180.f : 280.f;
    const float originY = wallScene ? 35.f : 125.f;
    const auto px=[scale](double p){ return static_cast<float>(p*scale); };
 
    rect(window, originX, originY+px(330), px(400), 3.f, WHITE);
  
    for (const Body& b : world.bodies) {
        const auto x = originX + px(b.position.x-b.halfSize.x);
        const auto y = originY + px(b.position.y-b.halfSize.y);
        const auto w = px(b.halfSize.x*2), h=px(b.halfSize.y*2);
        if (!b.active) continue;
        if (b.kind == Kind::Boundary) {
            if (b.timedGate) rect(window, x, y+h*.5f-2.f, w, 4.f, WHITE);
            continue; 
        }
        sf::Color c = b.kind==Kind::Static ? STATIC : b.kind==Kind::Normal ? NORMAL : FAST;
        const bool stackCube = b.kind == Kind::Normal &&
            (world.scene == Scene::Stack || world.scene == Scene::StackImpact ||
             world.scene == Scene::Supports);

        constexpr float seamAllowance = 1.25f;
        sf::RectangleShape shape(stackCube ? sf::Vector2f(w + 2*seamAllowance,
                                         h + 2*seamAllowance) : sf::Vector2f(w,h));
        shape.setPosition(stackCube ? x-seamAllowance : x,
                          stackCube ? y-seamAllowance : y);
        shape.setFillColor(c);
       
        if ((b.kind == Kind::Normal && !stackCube) || b.kind == Kind::Fast) {
           
            shape.setOutlineThickness(-1.2f);
            shape.setOutlineColor(sf::Color(30,34,47));
        }
        window.draw(shape);
     
        if (isMoving(b) && (world.scene==Scene::FastNormal ||
            world.scene==Scene::NormalNormal || world.scene==Scene::FastFast)) {
            if (std::abs(b.velocity.x)>1.) {
                float head = x+w*.5f + (b.velocity.x>0 ? w*.6f+12.f : -w*.6f-12.f);
                float cy=y+h*.5f;
                rect(window, std::min(x+w*.5f,head),cy-1.f,
                     std::abs(head-(x+w*.5f)),2.0f,c);
                sf::CircleShape tip(4.f,3);
                tip.setFillColor(c);
                tip.setRotation(b.velocity.x>0?90.f:270.f);
                tip.setPosition(head,cy);
                window.draw(tip);
            }
        }
    }
}
std::wstring observation(const App& a, sf::Color& color) {
    const auto& w=a.world;
    color=SOFT;
    if (a.scene==Scene::ThinWall) {
        const Body& projectile=w.bodies[0];
        const bool landed=projectile.position.y>=322.0 &&
                          std::abs(projectile.velocity.y)<30.0;
        if (a.wallBounced) {
            color=GREEN;
            return landed ? L"СТОЛКНОВЕНИЕ: отскок от стены, затем падение на пол"
                          : L"СТОЛКНОВЕНИЕ: отскок от стены, затем падение";
        }
        if (a.wallPassed) {
            color=BAD;
            return landed ? L"ТУННЕЛИРОВАНИЕ: снаряд прошёл стену и упал"
                          : L"ТУННЕЛИРОВАНИЕ: снаряд прошёл стену";
        }
        return L"Контроль 60 Гц проверяет конец шага; Multi-rate — 4 подшага; CCD — время контакта";
    }
    if (a.scene==Scene::FastNormal) {
        if (w.pairGravityActive) {
            color=GREEN;
            bool bothLanded = true;
            for (const Body& body : w.bodies) if (isMoving(body)) {
                if (std::abs(body.position.y + body.halfSize.y - 330.0) > 0.5 ||
                    std::abs(body.velocity.y) > 2.0) bothLanded = false;
            }
            return bothLanded
                ? L"УДАР ПОДТВЕРЖДЁН: ОБА ТЕЛА ПРИЗЕМЛИЛИСЬ"
                : L"УДАР ПОДТВЕРЖДЁН: оба тела получили импульс и падают";
        }
        return L"Розовый быстрый объект приближается к голубому обычному...";
    }
    if (a.scene==Scene::FastFast || a.scene==Scene::NormalNormal) {
        if (w.pairGravityActive) {
            color=GREEN;
            bool bothLanded = true;
            for (const Body& body : w.bodies) if (isMoving(body)) {
                if (std::abs(body.position.y + body.halfSize.y - 330.0) > 0.5 ||
                    std::abs(body.velocity.y) > 2.0) bothLanded = false;
            }
            return bothLanded ? L"ОБА ДИНАМИЧЕСКИХ ТЕЛА СТОЛКНУЛИСЬ И ПРИЗЕМЛИЛИСЬ"
                              : L"ДВА ДИНАМИЧЕСКИХ ТЕЛА СТОЛКНУЛИСЬ И ПАДАЮТ ВНИЗ";
        }
        return L"Два подвижных тела движутся навстречу друг другу...";
    }
    if (a.scene==Scene::StackImpact) {
        if (w.timeSeconds < 2.0) return L"ПОДГОТОВКА: стек оседает на неподвижные опоры; удар через 2 секунды.";
        if (a.mixedImpulses == 0) return L"Снаряд летит в стек. Ожидаем удар динамика-динамика...";
        if (w.stackRestConfirmed) {
            color=GREEN;
            bool pinkLanded = false;
            for (const Body& body : w.bodies) if (body.kind == Kind::Fast)
                pinkLanded = std::abs(body.position.y + body.halfSize.y - 330.0) < 0.5 &&
                             std::abs(body.velocity.y) < 2.0;
            const std::wstring eventText = pinkLanded
                     ? L"СТЕК УСТОЙЧИВ; РОЗОВЫЙ КУБ ПРИЗЕМЛИЛСЯ. "
                     : L"СТЕК УСТОЙЧИВ; РОЗОВЫЙ КУБ ЕЩЁ ПАДАЕТ. ";
            return eventText + L"Дрейф " + fixed(w.stackRestDriftPx,2) + L" пикс.";
        }
        color=SOFT;
        return L"УДАР ЕСТЬ: пик " + fixed(a.peakBlockSpeed,1) +
               L" пикс./с; ожидаем 1 с устойчивого покоя...";
    }
    if (a.scene==Scene::Stack) {
        double peak=0;
        for (const auto& b:w.bodies) if (b.kind==Kind::Normal)
            peak=std::max(peak,std::hypot(b.velocity.x,b.velocity.y));
        if (w.stackRestConfirmed) { color=GREEN; return L"СТЕК В ПОКОЕ: 1 с без заметного смещения; дрейф " + fixed(w.stackRestDriftPx,2) + L" пикс."; }
        return L"Подвижные блоки оседают на неподвижных опорах... скорость: " + fixed(peak,1);
    }
    if (a.scene==Scene::Supports) return L"Фиолетовые столбики стоят на месте; голубые тела взаимодействуют с ними.";
    return L"Шесть неподвижных опор, двадцать обычных и три быстрых тела.";
}
void drawHud(sf::RenderWindow& w, const sf::Font& f, const App& a) {
   
    txt(w,f,L"ЛР №3  |  " + std::wstring(sceneTitle(a.scene)),20,27,17);
    txt(w,f,L"Метод: " + std::wstring(methodName(a.mode)),19,27,52,FAST);

   
    rect(w,765,58,12,12,NORMAL); txt(w,f,L"обычный",13,782,53,SOFT);
    rect(w,868,58,12,12,FAST); txt(w,f,L"быстрый",13,885,53,SOFT);
    rect(w,974,58,12,12,STATIC); txt(w,f,L"опора",13,990,53,SOFT);

    sf::Color statusColor;
    const std::wstring result=observation(a,statusColor);
    txt(w,f,result,17,27,675,statusColor);

   
    if (a.scene==Scene::ThinWall) {
        std::wstring detail;
        if (a.mode==Mode::MultiRate) {
            detail = L"Multi-rate: 4 проверки/шаг 60 Гц";
            if (a.wallMicrostep>0) detail += L"  |  Удар найден на подшаге " + std::to_wstring(a.wallMicrostep) + L" из 4";
        } else if (a.mode==Mode::CCD) {
            detail = L"CCD: поиск точного времени контакта внутри шага";
            if (a.wallTimeMs>=0) detail += L"  |  t = " + fixed(a.wallTimeMs,2) + L" мс";
        } else detail = L"60 Гц: проверка пересечения только в конце шага";
        txt(w,f,detail,15,27,648,SOFT);
    }
    
    const std::wstring metrics =
        L"Физика: " + fixed(a.physicsFrameMs,3) + L" мс/кадр" +
        L"   |   60 Гц/кадр: " + std::to_wstring(a.displayedFrame.normalSteps) +
        L"   |   240 Гц/кадр: " + std::to_wstring(a.displayedFrame.fastSubsteps) +
        L"   |   Д-Д: " + std::to_wstring(a.dynamicImpulses) +
        L"   |   Д-С: " + std::to_wstring(a.staticImpulses);
    txt(w,f,metrics,14,27,706,SOFT);
    txt(w,f,L"Режимы: 1 — 60 Гц   2 — Multi-rate   3 — CCD",15,27,731,WHITE);
    txt(w,f,L"Сцены: 0 — все   4 — стенка   5 — пара   6 — опоры   7 — стек / удар   8,9 — пары",14,27,754,SOFT);
}
} 

int main() {
    sf::RenderWindow window(sf::VideoMode(WIN_W,WIN_H),
        sf::String(L"Лабораторная №3 — SFML, CCD и multi-rate"),
        sf::Style::Titlebar|sf::Style::Close);
    window.setFramerateLimit(60);
    window.setKeyRepeatEnabled(false);
    sf::Font font;
    if (!loadFont(font)) return 2;
    App app;
    sf::Clock frame;
    while (window.isOpen()) {
        const auto cpuFrameStart=std::chrono::steady_clock::now();
       
        app.physicsFrameMs=0;
        app.displayedFrame.clear();
        sf::Event e;
        while(window.pollEvent(e)) {
            if (e.type==sf::Event::Closed) window.close();
            if (e.type!=sf::Event::KeyPressed) continue;
            const auto k=e.key.code;
            if (k==sf::Keyboard::Escape) { window.close(); continue; }
          
            auto mode=[&](Mode m) { app.mode=m; app.restart(); };
            auto scene=[&](Scene next) { app.scene=next; app.restart(); };
            if (k==sf::Keyboard::Num1 || k==sf::Keyboard::Numpad1) mode(Mode::Discrete);
            if (k==sf::Keyboard::Num2 || k==sf::Keyboard::Numpad2) mode(Mode::MultiRate);
            if (k==sf::Keyboard::Num3 || k==sf::Keyboard::Numpad3) mode(Mode::CCD);
            if (k==sf::Keyboard::Num0 || k==sf::Keyboard::Numpad0) scene(Scene::Platforms);
            if (k==sf::Keyboard::Num4 || k==sf::Keyboard::Numpad4)
                scene(Scene::ThinWall); 
            if (k==sf::Keyboard::Num5 || k==sf::Keyboard::Numpad5) scene(Scene::FastNormal);
            if (k==sf::Keyboard::Num6 || k==sf::Keyboard::Numpad6)
                scene(Scene::Supports); 
            if (k==sf::Keyboard::Num7 || k==sf::Keyboard::Numpad7)
                scene(app.scene == Scene::Stack ? Scene::StackImpact : Scene::Stack);
            if (k==sf::Keyboard::Num8 || k==sf::Keyboard::Numpad8) scene(Scene::NormalNormal);
            if (k==sf::Keyboard::Num9 || k==sf::Keyboard::Numpad9) scene(Scene::FastFast);
        }
        const double elapsed=std::min(.1,static_cast<double>(frame.restart().asSeconds()));
     
        const bool wallPlayback = app.scene == Scene::ThinWall;
  
        const double physicsElapsed = elapsed * (wallPlayback ?
            (app.world.wallEventResolved ? 1.0 : 0.40) : 1.0);
        const double limit=stopMoment(app.scene);
        if (limit>0 && app.world.timeSeconds+1e-9>=limit) {
     
            app.hold+=elapsed;
            if (app.hold>=1.5) app.restart();
        } else {
            app.accumulator=std::min(.2,app.accumulator+physicsElapsed);
            int safety=0;
            while (app.accumulator >= 1.0/NORMAL_HZ && safety++<15) {
                app.tick();
                app.accumulator-=1.0/NORMAL_HZ;
                if (limit>0 && app.world.timeSeconds+1e-9>=limit) {
                    app.accumulator=0; break;
                }
            }
        }
        window.clear(BG);
        drawWorld(window,app);
        drawHud(window,font,app);
        const auto cpuFrameEnd=std::chrono::steady_clock::now();
        const double cpuMs=std::chrono::duration<double, std::milli>(cpuFrameEnd-cpuFrameStart).count();
        app.frameMs=app.frameMs==0 ? cpuMs : 0.9*app.frameMs + 0.1*cpuMs;
        window.display();
    }
    return 0;
}
