#pragma once
#include "Body.h"
#include <string>
namespace lab3 {
std::vector<Body> makeScene(Scene scene);
const char* sceneName(Scene scene);
std::string modeName(Mode mode);
int countKind(const std::vector<Body>& bodies, Kind kind);
}
