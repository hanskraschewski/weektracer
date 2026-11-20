#ifndef RTWEEKEND_H
#define RTWEEKEND_H

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <random>

using std::make_shared;
using std::shared_ptr;

const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;

// utility functions

inline double degree_to_radians(double degrees) {
    return degrees * pi / 180.0;
}

inline double random_double() {
    // vibecoded, maybe speedup idk
    thread_local uint64_t s = std::random_device{}() | (uint64_t(std::random_device{}()) << 32);
    // xorshift64*
    s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
    uint64_t r = s * 2685821657736338717ULL;
    return (r >> 11) * (1.0 / 9007199254740992.0);  // 53 bits -> [0,1)
}

inline double random_double(double min, double max) {
    return min + (max - min) * random_double();
}

inline int random_int(int min, int max) {
    return int(random_double(min, max+1));
}

#include "color.h"
#include "interval.h"
#include "ray.h"
#include "vec3.h"
#endif