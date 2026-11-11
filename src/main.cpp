#include "rtweekend.h"

#include "camera.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "sphere.h"
#include "bezier_hair_curve.h"



int main() {
    hittable_list world;

    auto ground_material = make_shared<lambertian>(color(0.8, 0.8, 0.0));
    world.add(make_shared<sphere>(point3(0, -100.5, -1.0), 100.0, ground_material));

    auto hair_material1 = make_shared<lambertian>(color(0.8, 0.2, 1.0));
    auto hair_material2 = make_shared<metal>(color(0.6, 0.6, 0.6), 0.2);
    std::array<point3, 4> control_points1 = {
        point3(0.8, 0.45, -1.0),
        point3(1.05, 1.25, -0.8),
        point3(1.55, 0.3, 0.6),
        point3(1.8, 0.75, 1.0)
    };
        std::array<point3, 4> control_points2 = {
        point3(1.3, 0.95, -1.0),
        point3(1.55, 1.75, -0.8),
        point3(1.85, 0.8, 0.6),
        point3(2.1, 1.25, 1.0)
    };
    world.add(make_shared<bezier_hair_curve>(control_points1, 0.06, 0.01, hair_material1));
    world.add(make_shared<bezier_hair_curve>(control_points2, 0.06, 0.01, hair_material2));

    camera cam;
    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 1920;
    cam.samples_per_pixel = 20;
    cam.max_depth = 5;

    cam.vfov = 30;
    cam.lookfrom = point3(4, 0.7, 0);
    cam.lookat   = point3(0, 0.7, 0);
    cam.vup      = vec3(0,1,0);
    cam.defocus_angle = 0;
    cam.focus_dist = 4.0;
    cam.render(world);
}