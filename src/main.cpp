#include "rtweekend.h"

#include "camera.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "sphere.h"
#include "bezier_hair_curve.h"
#include <omp.h>



int main() {
    
    hittable_list world;

    auto ground_material = make_shared<lambertian>(color(0.8, 0.8, 0.0));
    world.add(make_shared<sphere>(point3(0, -100.5, -1.0), 100.0, ground_material));

    auto metallic = make_shared<metal>(color(0.6, 0.6, 0.6), 0.0);
    std::array<point3, 4> control_points1 = {
        point3(0.8, 0.95, -1.2),
        point3(1.05, 1.75, -1.0),
        point3(1.55, 0.8, 0.6),
        point3(1.8, 1.25, 0.9)
    };

    auto glass = make_shared<dielectric>(1.0 / 1.3);
    std::array<point3, 4> control_points2 = {
        point3(0.8, 0.7, -1.0),
        point3(1.05, 1.5, -0.8),
        point3(1.55, 0.55, 0.6),
        point3(1.8, 1.0, 1.0)
    };

    auto lambert = make_shared<lambertian>(color(0.8, 0.2, 1.0));
    std::array<point3, 4> control_points3 = {
        point3(0.8, 0.35, -1.0),
        point3(1.05, 1.15, -0.8),
        point3(1.55, 0.2, 0.5),
        point3(1.8, 0.65, 0.9)
    };


    world.add(make_shared<bezier_hair_curve>(control_points1, 0.06, 0.01, metallic));
    //world.add(make_shared<bezier_hair_curve>(control_points2, 0.06, 0.01, glass));
    //world.add(make_shared<bezier_hair_curve>(control_points3, 0.06, 0.01, lambert));

    camera cam;
    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 1920;
    cam.samples_per_pixel = 100;
    cam.max_depth = 40;

    cam.vfov = 30;
    cam.lookfrom = point3(4, 0.7, 0);
    cam.lookat   = point3(0, 0.7, 0);
    cam.vup      = vec3(0,1,0);
    cam.defocus_angle = 0;
    cam.focus_dist = 4.0;
    cam.render(world);
    
   /*
   hittable_list world;

    auto ground_material = make_shared<lambertian>(color(0.5, 0.5, 0.5));
    world.add(make_shared<sphere>(point3(0,-1000,0), 1000, ground_material));

    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            auto choose_mat = random_double();
            point3 center(a + 0.9*random_double(), 0.2, b + 0.9*random_double());

            if ((center - point3(4, 0.2, 0)).length() > 0.9) {
                shared_ptr<material> sphere_material;

                if (choose_mat < 0.8) {
                    // diffuse
                    auto albedo = color::random() * color::random();
                    sphere_material = make_shared<lambertian>(albedo);
                    world.add(make_shared<sphere>(center, 0.2, sphere_material));
                } else if (choose_mat < 0.95) {
                    // metal
                    auto albedo = color::random(0.5, 1);
                    auto fuzz = random_double(0, 0.5);
                    sphere_material = make_shared<metal>(albedo, fuzz);
                    world.add(make_shared<sphere>(center, 0.2, sphere_material));
                } else {
                    // glass
                    sphere_material = make_shared<dielectric>(1.5);
                    world.add(make_shared<sphere>(center, 0.2, sphere_material));
                }
            }
        }
    }

    auto material1 = make_shared<dielectric>(1.5);
    world.add(make_shared<sphere>(point3(0, 1, 0), 1.0, material1));

    auto material2 = make_shared<lambertian>(color(0.4, 0.2, 0.1));
    world.add(make_shared<sphere>(point3(-4, 1, 0), 1.0, material2));

    auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
    world.add(make_shared<sphere>(point3(4, 1, 0), 1.0, material3));

    camera cam;

    cam.aspect_ratio      = 16.0 / 9.0;
    cam.image_width       = 1200;
    cam.samples_per_pixel = 100;
    cam.max_depth         = 20;

    cam.vfov     = 20;
    cam.lookfrom = point3(13,2,3);
    cam.lookat   = point3(0,0,0);
    cam.vup      = vec3(0,1,0);

    cam.defocus_angle = 0.6;
    cam.focus_dist    = 10.0;

    cam.render(world);
    */
}