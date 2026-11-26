#ifndef INTERSECTORS_H
#define INTERSECTORS_H

struct RayConeIntersection {
    double t;
    point3 c0;  // curve(t) base in RCC (cones base)
    vec3 cd;    // curve tangent(t) in RCC (cones axis)
    double s;   // ray.s (also hit_record.t) - c0.z for intersection between ray and cone(t)
    double dt;  // dt to the (ray intersected with cone) from t
    double dp;  // |(ray ∩ plane(t))−curve(t)|2
    double dc;  // |(ray∩cone(t))−curve(t)|2
    double sp;  // ray.s−c0.z for ray ∩ plane(t)

    // calculates ray cone intersection
    // cone based on plane c(t), c'(t), radius r and change of radius dr
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

struct RayEllipticCylinderIntersection {
    double t;
    point3 c0;
    vec3 cd;
    vec3 v1, v2;
    double s;
    double dt;

    // calculates
    inline bool intersect() {}

};

#endif