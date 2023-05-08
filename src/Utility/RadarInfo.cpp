#include "RadarInfo.hpp"

RadarTransform& RadarTransform::Instance() {
    static RadarTransform instance;
    return instance;
}