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

                            set_bbox(control_points);
                        }
        
        bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
            point3 u0_rcc; vec3 u1_rcc, u2_rcc, u3_rcc;
            to_rcc(r, u0_rcc, u1_rcc, u2_rcc, u3_rcc);

            double s, t;
            bool is_cap = false;
            bool hit = false;
            // start at closer end
            double t_start = (u1_rcc + u2_rcc + u3_rcc).z() > 0 ? 0.0 : 1.0;
            double len = r.direction().length();
            hit = trace_from(t_start, u0_rcc, u1_rcc, u2_rcc, u3_rcc, s, t, is_cap);
            if (hit) { 
                s /= len; 
                if (!ray_t.surrounds(s)) hit = false; 
            }
            // if closer end returned false/is outside of t range try with far end
            if (!hit) {
                hit = trace_from(1.0 - t_start, u0_rcc, u1_rcc, u2_rcc, u3_rcc, s, t, is_cap);
                if (hit) { 
                    s /= len; 
                    if (!ray_t.surrounds(s)) hit = false; 
                }
            }

            if (!hit) return false;

            vec3 p = r.at(s);
            
            vec3 n;
            if (is_cap) {
                n = (t == 0) ? -tangent(0) : tangent(1);   
                n = unit_vector(n);
            }
            else {
                point3 world_c = curve_point(t);
                vec3 world_dc = tangent(t);
                vec3 v = p - world_c;
                n = v - dot(v, world_dc)*world_dc/world_dc.length_squared();
                n = unit_vector(n);
                n = unit_vector(n - (r2 - r1)/(world_dc.length()) * unit_vector(world_dc));
            }
            rec.set_face_normal(r, n);
            rec.t = s;
            rec.mat = mat;
            rec.p = p;

            return true;
        }
        
        aabb bounding_box() const override { return bbox; }

    private:
        point3 u0, u1, u2, u3; // bezier curve points in world coordinates
        point3 u0_rcc, u1_rcc, u2_rcc, u3_rcc; // bezier curves points in current rays rcc
        double r1, r2;
        shared_ptr<material> mat;
        aabb bbox;

        bool trace_from(double t_start,
                        const point3& u0_rcc, const vec3& u1_rcc,
                        const vec3& u2_rcc, const vec3& u3_rcc,
                        double& s_out, double& t_out,
                        bool& is_cap) const {
            const double eps = 5e-5; // if ∆t smaller eps: success
            const int max_iter = 50; 
            const double dr = r2 - r1; // derivative of linear radius
            interval clamp_dt(-0.5, 0.5); // clamp magnitude of ∆t to 0.5
            is_cap = false;
            RayConeIntersection rc;
            double t = t_start;

            // regula falsi/bisection variables
            double t_pos, f_pos, t_neg, f_neg;
            bool n_brack = false, p_brack = false, bracket = false;
            // previous iteration signs for regula falsi
            double t_prev, f_prev;
            bool has_prev = false;

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

                rc.dt = clamp_dt.clamp(rc.dt);
                
                // set regula falsi
                if (rc.dt > 0) {
                    f_pos = rc.dt;
                    t_pos = t;
                    p_brack = true;
                } else {
                    f_neg = rc.dt;
                    t_neg = t;
                    n_brack = true;
                }
                if (p_brack && n_brack) bracket = true;
                
                // no bracket exists, normal step, otherwise regula falsi step
                if (bracket && (iter % 4) == 0) 
                    t = (t_pos + t_neg) / 2;
                else if (has_prev && f_prev * rc.dt < 0) 
                    t = (f_pos * t_neg - f_neg * t_pos) / (f_pos - f_neg);
                else
                    t += rc.dt;
                
                t_prev = t;
                f_prev = rc.dt;
                has_prev = true;


                // buttend hit logic
                if (t < 0.0 || t > 1.0) {
                    t = (t < 0) ? 0.0 : 1.0;
                    rc.c0 = curve_point(t, u0_rcc, u1_rcc, u2_rcc, u3_rcc);
                    rc.cd = tangent(t, u0_rcc, u1_rcc, u2_rcc, u3_rcc);
                    double r_end = radius(t);
                    rc.intersect(r_end, dr);
                    bool enters_from_outside = (t == 0) ? rc.cd.z() > 0 : rc.cd.z() < 0;
                    if(enters_from_outside && rc.dp < r_end*r_end) {
                        s_out = rc.sp + rc.c0.z();
                        t_out = t;
                        is_cap = true;
                        return true;
                    }
                    return false;
                } 
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

        void set_bbox(const std::array<point3, 4>& cp) {
            double R = std::fmax(r1, r2);
            aabb hull(aabb(cp[0], cp[1]), aabb(cp[2], cp[3]));

            bbox = aabb(
                interval(hull.x.min - R, hull.x.max + R),
                interval(hull.y.min - R, hull.y.max + R),
                interval(hull.z.min - R, hull.z.max + R));
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