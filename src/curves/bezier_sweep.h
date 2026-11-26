#ifndef BEZIER_SWEEP_H
#define BEZIER_SWEEP_H

#include "intersectors.h"
#include <array>

class RaySpace {
    public:
        RaySpace(const ray& r) {
            origin = r.origin();
            basis = ortho_basis_from_vec(r.direction());
        }
        
        point3 to_local_point(const point3& p) const {
            return to_basis(basis, p - origin);
        }
        vec3 to_local_vec(const vec3& v) const {
            return to_basis(basis, v);
        }
    
    private:
        point3 origin;
        ortho_basis basis;
};

class CubicBezier {
    public:
        CubicBezier(const std::array<point3, 4>& control_points){
            u0 = control_points[0];
            u1 = 3*(control_points[1] - control_points[0]);
            u2 = 3*(control_points[0]-2*control_points[1]+control_points[2]);
            u3 = -control_points[0]+3*control_points[1]-3*control_points[2]+control_points[3];
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

        void to_rcc(RaySpace ray_space, point3& u0_rcc, vec3& u1_rcc, vec3& u2_rcc, vec3& u3_rcc) const {
            u0_rcc = ray_space.to_local_point(u0);
            u1_rcc = ray_space.to_local_vec(u1);
            u2_rcc = ray_space.to_local_vec(u2);
            u3_rcc = ray_space.to_local_vec(u3); 
        }

    private:
        vec3 u0, u1, u2, u3;
};

class CircleSweep {
    public:
        CircleSweep(const std::array<point3, 4>& control_points, shared_ptr<material> mat, double r1, double r2) 
                :  centerline(control_points), mat(mat), r1(r1), r2(r2) {}
        
        bool hit_segment(const ray& r, interval ray_t, double t_min, double t_max, hit_record& rec) const {
            RaySpace ray_space = RaySpace(r);
            const CubicBezier& curve = centerline;
            point3 u0_rcc; vec3 u1_rcc, u2_rcc, u3_rcc;
            curve.to_rcc(ray_space, u0_rcc, u1_rcc, u2_rcc, u3_rcc);

            double s, t; // ray and curve parameters of intersection
            bool is_cap = false;
            bool hit = false;
            // start at closer end
            double za = curve.curve_point(t_min, u0_rcc, u1_rcc, u2_rcc, u3_rcc).z();
            double zb = curve.curve_point(t_max, u0_rcc, u1_rcc, u2_rcc, u3_rcc).z();
            double t_start = (za < zb) ? t_min : t_max;
            double t_other = (za < zb) ? t_max : t_min;
            double len = r.direction().length();

            hit = trace_from(t_start, t_min, t_max, u0_rcc, u1_rcc, u2_rcc, u3_rcc, s, t, is_cap);
            if (hit) { 
                s /= len; 
                if (!ray_t.surrounds(s)) hit = false; 
            }
            // if closer end returned false/is outside of t range try with far end
            if (!hit) {
                hit = trace_from(t_other, t_min, t_max, u0_rcc, u1_rcc, u2_rcc, u3_rcc, s, t, is_cap);
                if (hit) { 
                    s /= len; 
                    if (!ray_t.surrounds(s)) hit = false; 
                }
            }

            if (!hit) return false;

            vec3 p = r.at(s);
            
            vec3 n;
            if (is_cap) {
                n = (t == 0) ? -curve.tangent(0) : curve.tangent(1);   
                n = unit_vector(n);
            }
            else {
                point3 world_c = curve.curve_point(t);
                vec3 world_dc = curve.tangent(t);
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

        bool trace_from(double t_start, double t_min, double t_max,
                const point3& u0_rcc, const vec3& u1_rcc,
                const vec3& u2_rcc, const vec3& u3_rcc,
                double& s_out, double& t_out,
                bool& is_cap) const {
            const double eps = 5e-5; // if ∆t smaller eps: success
            const int max_iter = 50; 
            const double dr = r2 - r1; // derivative of linear radius
            interval clamp_dt(-(t_max - t_min)/2.0, (t_max - t_min)/2.0); // clamp magnitude of ∆t to 0.5
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
                rc.c0 = centerline.curve_point(t, u0_rcc, u1_rcc, u2_rcc, u3_rcc);
                rc.cd = centerline.tangent(t, u0_rcc, u1_rcc, u2_rcc, u3_rcc);
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
                if (t < t_min || t > t_max) {
                    bool at_true_end = (t < t_min && t_min == 0.0) || (t > t_max && t_max == 1.0);
                    if (!at_true_end) return false;   // interior joint: neighbor handles it

                    t = (t < t_min) ? t_min : t_max;
                    rc.c0 = centerline.curve_point(t, u0_rcc, u1_rcc, u2_rcc, u3_rcc);
                    rc.cd = centerline.tangent(t, u0_rcc, u1_rcc, u2_rcc, u3_rcc);
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

    double radius(double t) const {
        return ((1-t) * r1 + t * r2);
    }

    const CubicBezier& get_centerline() const {
        return centerline;
    }

    aabb get_bounding_box(double t_min, double t_max) const {
        double h = t_max-t_min;
            double R = std::fmax(radius(t_min), radius(t_max));
            point3 q0, q1, q2, q3;
            point3 ca, cb;
            vec3 ta, tb;
            ca = centerline.curve_point(t_min); cb = centerline.curve_point(t_max);
            ta = centerline.tangent(t_min);     tb = centerline.tangent(t_max);
            q0 = ca;
            q1 = ca + (h/3.0)*ta;
            q2 = cb - (h/3.0)*tb;
            q3 = cb;

            aabb hull(aabb(q0, q1), aabb(q2, q3));

            return aabb(
                interval(hull.x.min - R, hull.x.max + R),
                interval(hull.y.min - R, hull.y.max + R),
                interval(hull.z.min - R, hull.z.max + R));
    }

    private:
        shared_ptr<material> mat;
        CubicBezier centerline;
        double r1, r2;


};

class EllipseSweep {
    public:
        EllipseSweep(const std::array<point3, 4>& control_points, shared_ptr<material> mat, double a, double b) 
                :  centerline(control_points), mat(mat), a(a), b(b) {
            precompute_rmf();
        }

        bool hit_segment(const ray& r, interval ray_t, double t_min, double t_max, hit_record& rec) const {
            return false;
        }

        bool trace_from(double t_start, double t_min, double t_max, 
                        const point3& u0_rcc, const vec3& u1_rcc,
                        const vec3& u2_rcc, const vec3& u3_rcc,
                        const RaySpace ray_space,
                        double& s_out, double& t_out,
                        bool& is_cap) const {
            return false;
        }

    private:
        shared_ptr<material> mat;
        CubicBezier centerline;
        double a, b;
        std::array<vec3, 64> frame = {};

        // helper function to precompute rotation minimizing frames along curve in array
        void precompute_rmf() {
            double step_size = 1.0 / (frame.size() - 1.0);
            double eps = 1e-12;
            vec3 c = centerline.curve_point(0);
            vec3 ct = unit_vector(centerline.tangent(0));
            vec3 r = random_ortho_vector(ct);
            frame[0] = r;
            vec3 c_next, ct_next;
            for (int i = 0; i < frame.size() - 1; i++)
            {
                double next = double(i + 1) * step_size;
                c_next = centerline.curve_point(next);
                ct_next = unit_vector(centerline.tangent(next));
                vec3 v1 = c_next - c;
                if (v1.length_squared() < eps) {
                    r = unit_vector(r - dot(r, ct_next) * ct_next);
                } else {
                    v1 = unit_vector(v1);
                    vec3 v2 = ct_next - reflect(ct, v1);
                    if (v2.length_squared() < eps)
                        r = reflect(r, v1);
                    else
                        r = reflect(reflect(r, v1), unit_vector(v2));
                }

                frame[i + 1] = r;

                c = c_next;
                ct = ct_next;
            }
        }

        // interpolates frame axis orthogonal to tangent from procomputed rmf using current t
        void axes(double t, vec3& v1, vec3& v2) const {
            t = interval(0.0, 1.0).clamp(t);

            const double position = t * (frame.size() - 1);
            int i = static_cast<int>(std::floor(position));
            int j = std::min(i + 1, static_cast<int>(frame.size()) - 1);
            double blend = position - i;
            vec3 axis = unit_vector(frame[i] * (1.0 - blend) + frame[j] * blend);
            vec3 tan = unit_vector(centerline.tangent(t));

            v1 = a * axis;
            v2 = b * unit_vector(cross(tan, axis));
        }
};
#endif