#ifndef BEZIER_HAIR_CURVE_H
#define BEZIER_HAIR_CURVE_H

#include "hittable.h"
#include <array>

struct RayConeIntersection {
    double t;
    point3 c0;  // curve(t) base in RCC (cones base)
    vec3 cd;    // curve tangent(t) in RCC (cones axis)
    double s;   // ray.s (also hit_record.t) - c0.z for intersection between ray and cone(t)
    double dt;  // dt to the (ray intersected with cone) from t
    double dp;  // |(ray ∩ plane(t))−curve(t)|2
    double dc;  // |(ray∩cone(t))−curve(t)|2
    double sp;  // ray.s−c0.z for ray ∩ plane(t)

    inline bool intersect(double r, double dr) {
            double r2 = r * r;
            double drr = r * dr;

            double ddd  = cd.x() * cd.x() + cd.y() * cd.y(); // ddd
            dp          = (c0.x() * c0.x() + c0.y() * c0.y()); 
            double cdd  = c0.x() * cd.x() + c0.y() * cd.y(); // cdd
            double cxd  = c0.x() * cd.y() - c0.y() * cd.x(); // cxd

            double c    = ddd;
            double b    = cd.z() * (drr - cdd);
            double cdz2 = cd.z() * cd.z();
            ddd        += cdz2;
            double a    = 2*drr*cdd + cxd*cxd - ddd*r2 + dp*cdz2;

            // here dr squared terms, which can be dropped because the radius changes only slightly 
            // typical hair models and it does not affect convergence
            // maybe important for drastically changing radii
            if (false) {
                double qs = (dr * dr) / ddd;
                a -= qs * cdd *cdd;
                b -= qs * cd.z() * cdd;
                c -= qs * cdz2;
            }

            double det = b * b - a * c;
            s          = (b - (det > 0 ? sqrt(det) : 0))/c;
            dt         = (s*cd.z() - cdd)/ddd;
            dc         = s*s + dp;
            sp         = cdd/cd.z();
            dp        += sp*sp;

            return det > 0;
    }

};

class bezier_hair_curve : public hittable {
    public:
        bezier_hair_curve(const std::array<point3, 4>& control_points, 
                        double r1, double r2, 
                        shared_ptr<material> mat)
                        : r1(std::fmax(0, r1)), r2(std::fmax(0, r2)), mat(mat) {
                            u0 = control_points[0];
                            u1 = 3*(control_points[1] - control_points[0]);
                            u2 = 3*(control_points[0]-2*control_points[1]+control_points[2]);
                            u3 = -control_points[0]+3*control_points[1]-3*control_points[2]+control_points[3];
                        }
        
        bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
            point3 u0_rcc; vec3 u1_rcc, u2_rcc, u3_rcc;
            to_rcc(r, u0_rcc, u1_rcc, u2_rcc, u3_rcc);

            double s0, t0, s1, t1;
            bool hit0 = trace_from(0.0, u0_rcc, u1_rcc, u2_rcc, u3_rcc, s0, t0);
            bool hit1 = trace_from(1.0, u0_rcc, u1_rcc, u2_rcc, u3_rcc, s1, t1);
            if (!hit0 && !hit1) return false;

            double s, t;
            if (hit0 && (!hit1 || s0 < s1)) { s = s0; t = t0; }
            else                            { s = s1; t = t1; }

            s /= r.direction().length();
            if (!ray_t.surrounds(s)) return false;

            vec3 p = r.at(s);
            point3 world_c = curve_point(t);
            vec3 world_dc = tangent(t);
            vec3 v = p - world_c;
            vec3 n = v - dot(v, world_dc)*world_dc/world_dc.length_squared();
            n = unit_vector(n);
            rec.set_face_normal(r, n);
            rec.t = s;
            rec.mat = mat;
            rec.p = p;

            return true;
        }

        private:
            point3 u0, u1, u2, u3; // bezier curve points in world coordinates
            point3 u0_rcc, u1_rcc, u2_rcc, u3_rcc; // bezier curves points in current rays rcc
            double r1, r2;
            shared_ptr<material> mat;

            bool trace_from(double t_start,
                            const point3& u0_rcc, const vec3& u1_rcc,
                            const vec3& u2_rcc, const vec3& u3_rcc,
                            double& s_out, double& t_out) const {
                const double eps = 5e-5;
                const int max_iter = 50;
                const double dr = r2 - r1;
                interval clamp_dt(-0.5, 0.5);

                    RayConeIntersection rc;
                    double t = t_start;

                    for (int iter = 0; iter < max_iter; ++iter) {
                        rc.c0 = curve_point(t, u0_rcc, u1_rcc, u2_rcc, u3_rcc);
                        rc.cd = tangent(t, u0_rcc, u1_rcc, u2_rcc, u3_rcc);
                        bool real = rc.intersect(radius(t), dr);

                        // converged: decide now, while s and t still belong together
                        if (std::fabs(rc.dt) < eps) {
                            if (!real) return false;          // converged phantom = near miss
                            s_out = rc.s + rc.c0.z();
                            t_out = t;
                            return true;
                        }

                        t += clamp_dt.clamp(rc.dt);
                        if (t < 0.0 || t > 1.0) return false;  // left the segment (buttend logic later)
                    }
                    return false;                              // hit the iteration cap, not converged
            }
            
            point3 curve_point(double t) const {
                return ((u3 * t + u2) * t + u1) * t + u0;
            }

            point3 curve_point(double t, const point3& u0, const vec3& u1, const vec3& u2, const vec3&  u3) const {
                return ((u3 * t + u2) * t + u1) * t + u0;
            }

            point3 tangent(double t) const {
                return (3 * u3 * t + 2 * u2) * t + u1;
            }

            point3 tangent(double t,  const point3& u0, const vec3& u1, const vec3& u2, const vec3&  u3) const {
                return (3 * u3 * t + 2 * u2) * t + u1;
            }

            double radius(double t) const {
                return ((1-t) * r1 + t * r2);
            }

            void to_rcc(const ray& r, point3& u0_rcc, vec3& u1_rcc, vec3& u2_rcc, vec3& u3_rcc) const {
                vec3 b1, b2, b3;
                ortho_basis(r.direction(), b1, b2, b3);

                vec3 a = u0 - r.origin();
                u0_rcc = vec3(dot(a, b2), dot(a, b3), dot(a, b1));
                u1_rcc = vec3(dot(u1, b2), dot(u1, b3), dot(u1, b1));
                u2_rcc = vec3(dot(u2, b2), dot(u2, b3), dot(u2, b1));
                u3_rcc = vec3(dot(u3, b2), dot(u3, b3), dot(u3, b1));
            }

            
};

#endif