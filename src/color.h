#ifndef COLOR_H
#define COLOR_H

#include "interval.h"
#include  "vec3.h"

using color = vec3;

inline double linear_to_gamma(double linear_component) {
    if (linear_component > 0)
        return std::sqrt(linear_component);
    
    return 0;
}

// for P3 format output
void write_color(std::ostream& out, const color& pixel_color) {
    auto r = pixel_color.x();
    auto g = pixel_color.y();
    auto b = pixel_color.z();

    // gamma correction
    r = linear_to_gamma(r);
    g = linear_to_gamma(g);
    b = linear_to_gamma(b);

    // translate [0, 1] component values to byte range [0, 255]
    static const interval intensity(0.000, 0.999);
    int rbyte = int(256 * intensity.clamp(r));
    int gbyte = int(256 * intensity.clamp(g));
    int bbyte = int(256 * intensity.clamp(b));

    // Write out the pixel color components
    out << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
}

// for P6 format output
inline void color_to_bytes(const color& pixel_color, unsigned char* out) {
    static const interval intensity(0.000, 0.999);
    out[0] = static_cast<unsigned char>(int(256 * intensity.clamp(linear_to_gamma(pixel_color.x()))));
    out[1] = static_cast<unsigned char>(int(256 * intensity.clamp(linear_to_gamma(pixel_color.y()))));
    out[2] = static_cast<unsigned char>(int(256 * intensity.clamp(linear_to_gamma(pixel_color.z()))));
}
#endif