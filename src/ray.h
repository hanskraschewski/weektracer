#ifndef RAY_H
#define RAY_H

#include "vec3.h"

class ray {
    public:
        ray() {}

        ray(const point3& origin, const vec3& direction) : orig(origin), dir(direction) {}

        const point3& origin() const { return orig; }
        const vec3& direction() const { return dir; }

        point3 at(double t) const {
            return orig + t * dir;
        }
        
    private:
        point3 orig;
        vec3 dir;
};

/*
inline vec3 vec3_to_rcc(const ray& r, const vec3& a) {
    vec3 b0, b1, b2;
    ortho_basis(r.direction(), b0, b1, b2);

    return vec3(dot(a, b1), dot(a, b2), dot(a, b0));
}

inline point3 point3_to_rcc(const ray& r, const point3& a) {
    return vec3_to_rcc(r, a - r.origin());
}
    */
#endif