#include "rtweekend.h"

#include "bvh.h"
#include "camera.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "sphere.h"
#include "quad.h"
#include "texture.h"

#include "curves/bezier_hair_curve.h"


void spheres() {
    hittable_list world;
    auto checker = make_shared<checker_texture>(0.32, color(.2, .3, .1), color(.9, .9, .9));
    world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(checker)));

    for (int a = -14; a < 14; a++) {
        for (int b = -14; b < 14; b++) {
            auto choose_mat = random_double();
            point3 center(a + 0.9*random_double(), 0.2, b + 0.9*random_double());

            if ((center - point3(4, 0.2, 0)).length() > 0.9) {
                shared_ptr<material> sphere_material;

                if (choose_mat < 0.8) {
                    // diffuse
                    auto albedo = color::random() * color::random();
                    sphere_material = make_shared<lambertian>(albedo);
                    //auto center2 = center + vec3(0, random_double(0,.5), 0);
                    auto center2 = center;
                    world.add(make_shared<sphere>(center, center2, 0.2, sphere_material));
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

    world = hittable_list(make_shared<bvh_node>(world));

    camera cam;

    cam.aspect_ratio      = 16.0 / 9.0;
    cam.image_width       = 1200;
    cam.samples_per_pixel = 50;
    cam.max_depth         = 20;
    cam.background        = color(0.70, 0.80, 1.00);

    cam.vfov     = 20;
    cam.lookfrom = point3(13,2,3);
    cam.lookat   = point3(0,0,0);
    cam.vup      = vec3(0,1,0);

    cam.defocus_angle = 0.8;
    cam.focus_dist    = 10.0;

    cam.render(world);
}

void checkered_spheres() {
    hittable_list world;

    auto checker = make_shared<checker_texture>(0.32, color(.2, .3, .1), color(.9, .9, .9));

    world.add(make_shared<sphere>(point3(0,-10, 0), 10, make_shared<lambertian>(checker)));
    world.add(make_shared<sphere>(point3(0, 10, 0), 10, make_shared<lambertian>(checker)));

    camera cam;

    cam.aspect_ratio      = 16.0 / 9.0;
    cam.image_width       = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth         = 50;
    cam.background        = color(0.70, 0.80, 1.00);

    cam.vfov     = 20;
    cam.lookfrom = point3(13,2,3);
    cam.lookat   = point3(0,0,0);
    cam.vup      = vec3(0,1,0);

    cam.defocus_angle = 0;

    cam.render(world);
}

void perlin_spheres() {
    hittable_list world;

    auto pertext = make_shared<noise_texture>(4);
    world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(pertext)));
    world.add(make_shared<sphere>(point3(0,2,0), 2, make_shared<lambertian>(pertext)));

    camera cam;

    cam.aspect_ratio      = 16.0 / 9.0;
    cam.image_width       = 1200;
    cam.samples_per_pixel = 100;
    cam.max_depth         = 50;
    cam.background        = color(0.70, 0.80, 1.00);

    cam.vfov     = 20;
    cam.lookfrom = point3(13,2,3);
    cam.lookat   = point3(0,0,0);
    cam.vup      = vec3(0,1,0);

    cam.defocus_angle = 0;

    cam.render(world);
}

void quads() {
    hittable_list world;

    // Materials
    auto left_red     = make_shared<lambertian>(color(1.0, 0.2, 0.2));
    auto back_green   = make_shared<lambertian>(color(0.2, 1.0, 0.2));
    auto right_blue   = make_shared<lambertian>(color(0.2, 0.2, 1.0));
    auto upper_orange = make_shared<lambertian>(color(1.0, 0.5, 0.0));
    auto lower_teal   = make_shared<lambertian>(color(0.2, 0.8, 0.8));

    // Quads
    world.add(make_shared<triangle>(point3(-3,-2, 5), vec3(0, 0,-4), vec3(0, 4, 0), left_red));
    world.add(make_shared<disc>(point3(0,0, 0), vec3(4, 0, 0), vec3(0, 4, 0), 2.0, back_green));
    world.add(make_shared<quad>(point3( 3,-2, 1), vec3(0, 0, 4), vec3(0, 4, 0), right_blue));
    world.add(make_shared<disc>(point3(0, 3, 3), vec3(4, 0, 0), vec3(0, 0, 4), 2.0, upper_orange));
    world.add(make_shared<triangle>(point3(-2,-3, 5), vec3(4, 0, 0), vec3(0, 0,-4), lower_teal));

    camera cam;

    cam.aspect_ratio      = 1.0;
    cam.image_width       = 1000;
    cam.samples_per_pixel = 150;
    cam.max_depth         = 70;
    cam.background        = color(0.70, 0.80, 1.00);

    cam.vfov     = 80;
    cam.lookfrom = point3(0,0,9);
    cam.lookat   = point3(0,0,0);
    cam.vup      = vec3(0,1,0);

    cam.defocus_angle = 0;

    cam.render(world);
}

void simple_light() {
    hittable_list world;

    auto pertext = make_shared<noise_texture>(4);
    world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(pertext)));
    world.add(make_shared<sphere>(point3(0,2,0), 2, make_shared<lambertian>(pertext)));

    auto difflight = make_shared<diffuse_light>(color(4,4,4));
    auto redlight = make_shared<diffuse_light>(color(6,0,0));
    auto greenlight = make_shared<diffuse_light>(color(0,6,0));

    world.add(make_shared<sphere>(point3(-1,2,-3), 1.0, redlight));
    world.add(make_shared<sphere>(point3(-1,2,3), 1.0, greenlight));
    //world.add(make_shared<quad>(point3(3,1,-2), vec3(2,0,0), vec3(0,2,0), difflight));

    camera cam;

    cam.aspect_ratio      = 16.0 / 9.0;
    cam.image_width       = 800;
    cam.samples_per_pixel = 1500;
    cam.max_depth         = 50;
    cam.background        = color(0,0,0);

    cam.vfov     = 20;
    cam.lookfrom = point3(26,3,6);
    cam.lookat   = point3(0,2,0);
    cam.vup      = vec3(0,1,0);

    cam.defocus_angle = 0;

    world = hittable_list(make_shared<bvh_node>(world));

    cam.render(world);
}

void cornell_box() {
    hittable_list world;

    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));

    world.add(make_shared<quad>(point3(555,0,0), vec3(0,555,0), vec3(0,0,555), green));
    world.add(make_shared<quad>(point3(0,0,0), vec3(0,555,0), vec3(0,0,555), red));
    world.add(make_shared<quad>(point3(343, 554, 332), vec3(-130,0,0), vec3(0,0,-105), light));
    world.add(make_shared<quad>(point3(0,0,0), vec3(555,0,0), vec3(0,0,555), white));
    world.add(make_shared<quad>(point3(555,555,555), vec3(-555,0,0), vec3(0,0,-555), white));
    world.add(make_shared<quad>(point3(0,0,555), vec3(555,0,0), vec3(0,555,0), white));

    world.add(box(point3(130, 0, 65), point3(295, 165, 230), white));
    world.add(box(point3(265, 0, 295), point3(430, 330, 460), white));
    camera cam;

    cam.aspect_ratio      = 1.0;
    cam.image_width       = 600;
    cam.samples_per_pixel = 1000;
    cam.max_depth         = 50;
    cam.background        = color(0,0,0);

    cam.vfov     = 40;
    cam.lookfrom = point3(278, 278, -800);
    cam.lookat   = point3(278, 278, 0);
    cam.vup      = vec3(0,1,0);

    cam.defocus_angle = 0;

    //world = hittable_list(make_shared<bvh_node>(world));

    cam.render(world);
}

void earth() {
    auto earth_texture = make_shared<image_texture>("../textures/2k_earth_daymap.jpg");
    auto earth_surface = make_shared<lambertian>(earth_texture);
    auto globe = make_shared<sphere>(point3(0,0,0), 2, earth_surface);

    
    camera cam;

    cam.aspect_ratio      = 16.0 / 9.0;
    cam.image_width       = 1200;
    cam.samples_per_pixel = 100;
    cam.max_depth         = 50;
    cam.background        = color(0.70, 0.80, 1.00);

    cam.vfov     = 20;
    cam.lookfrom = point3(12,0,-3);
    cam.lookat   = point3(0,0,0);
    cam.vup      = vec3(0,1,0);

    cam.defocus_angle = 0;

    cam.render(hittable_list(globe));
}

void strands() {
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
        point3(1.9, 0.35, -0.75),
        point3(1.55, 0.35, -0.75),
        point3(1.1, 0.6, -0.65),
        point3(0.7, 0.45, -0.55)
    };

    auto lambert = make_shared<lambertian>(color(0.8, 0.2, 1.0));
    std::array<point3, 4> control_points3 = {
        point3(1.9, 0.3, 0.5),
        point3(1.55, 0.3, 0.5),
        point3(1.0, 0.5, 1.0),
        point3(0.6, 0.35, 1.2)
    };

    add_bezier_segments(world, control_points1, 0.06, 0.01, metallic);
    add_bezier_segments(world, control_points2, 0.06, 0.01, metallic);
    add_bezier_segments(world, control_points3, 0.06, 0.01, lambert);
    //world.add(make_shared<bezier_hair_curve>(control_points1, 0.06, 0.01, metallic));
    //world.add(make_shared<bezier_hair_curve>(control_points2, 0.06, 0.01, metallic));
    //world.add(make_shared<bezier_hair_curve>(control_points3, 0.06, 0.01, lambert));
    world = hittable_list(make_shared<bvh_node>(world));

    camera cam;
    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 1920;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;
    cam.background        = color(0.70, 0.80, 1.00);

    cam.vfov = 30;
    cam.lookfrom = point3(4, 0.7, 0);
    cam.lookat   = point3(0, 0.7, 0);
    cam.vup      = vec3(0,1,0);
    cam.defocus_angle = 0;
    cam.focus_dist = 4.0;
    cam.render(world);
}
void bezier_test() {
    hittable_list world;
    auto ground_material = make_shared<lambertian>(color(0.45, 0.45, 0.45));
    world.add(make_shared<sphere>(point3(0, -100.5, 0), 100.0, ground_material));

    for (int row = 0; row < 10; ++row) {
        for (int column = 0; column < 10; ++column) {
            double x = (column - 4.5) * 0.45;
            double z = (row - 4.5) * 0.45;
            double height = 0.8 + 0.08 * ((row * 3 + column * 7) % 10);
            double bend = 0.08 + 0.025 * (column % 5);
            double radius = 0.015 + 0.005 * ((row * 7 + column * 3) % 10);

            std::array<point3, 4> control_points = {
                point3(x, -0.48, z),
                point3(x + bend, -0.48 + height / 3.0, z - bend * 0.5),
                point3(x - bend, -0.48 + 2.0 * height / 3.0, z + bend * 0.5),
                point3(x + 0.04 * (row % 3 - 1), -0.48 + height, z)
            };
            auto strand_material = make_shared<lambertian>(color(
                0.18 + 0.065 * column,
                0.2 + 0.06 * row,
                0.75 - 0.035 * column));

            add_bezier_segments(world, control_points, radius, radius * 0.55, strand_material);
        }
    }

    world = hittable_list(make_shared<bvh_node>(world));

    camera cam;
    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 1200;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;
    cam.background        = color(0.70, 0.80, 1.00);

    cam.vfov = 20;
    cam.lookfrom = point3(7, 5, 9);
    cam.lookat = point3(0, 0.2, 0);
    cam.vup = vec3(0, 1, 0);
    cam.defocus_angle = 0;
    cam.focus_dist = 10;
    cam.render(world);
}
void curly_sphere() {
    hittable_list world;
    constexpr double sphere_radius = 1.0;
    constexpr int strand_count = 1000;
    constexpr int curve_pieces = 10;
    constexpr double golden_angle = 2.399963229728653;

    auto sphere_material = make_shared<metal>(color(0.28, 0.22, 0.18), 0.15);
    world.add(make_shared<sphere>(point3(0, 0, 0), sphere_radius, sphere_material));

    std::array<shared_ptr<material>, 6> strand_materials = {
        make_shared<lambertian>(color(0.8, 0.2, 0.12)),
        make_shared<lambertian>(color(0.82, 0.48, 0.12)),
        make_shared<lambertian>(color(0.48, 0.68, 0.2)),
        make_shared<lambertian>(color(0.16, 0.62, 0.68)),
        make_shared<lambertian>(color(0.24, 0.38, 0.78)),
        make_shared<lambertian>(color(0.72, 0.24, 0.58))
    };

    for (int i = 0; i < strand_count; ++i) {
        double y = 1.0 - 2.0 * (i + 0.5) / strand_count;
        double ring_radius = std::sqrt(1.0 - y * y);
        double azimuth = golden_angle * i;
        vec3 outward(ring_radius * std::cos(azimuth), y,
                     ring_radius * std::sin(azimuth));

        vec3 reference_axis = std::fabs(outward.y()) < 0.9
            ? vec3(0, 1, 0) : vec3(1, 0, 0);
        vec3 tangent_axis = unit_vector(cross(reference_axis, outward));
        vec3 bitangent_axis = cross(outward, tangent_axis);

        double strand_length = 1.7 + 0.9 * ((i * 37) % strand_count) / strand_count;
        double curl_radius = 0.18 + 0.34 * ((i * 73) % strand_count) / strand_count;
        double turns = 1.6 + 0.9 * ((i * 19) % strand_count) / strand_count;
        double phase = golden_angle * (i % 23);
        double angular_speed = 2.0 * pi * turns;
        double strand_radius = 0.008 + 0.009 * ((i * 53) % strand_count) / strand_count;

        auto centerline = [&](double t) {
            double angle = phase + angular_speed * t;
            vec3 swirl = std::cos(angle) * tangent_axis + std::sin(angle) * bitangent_axis;
            return outward * (sphere_radius + strand_length * t)
                 + swirl * (curl_radius * t);
        };
        auto tangent = [&](double t) {
            double angle = phase + angular_speed * t;
            vec3 swirl = std::cos(angle) * tangent_axis + std::sin(angle) * bitangent_axis;
            vec3 swirl_tangent = -std::sin(angle) * tangent_axis + std::cos(angle) * bitangent_axis;
            return outward * strand_length + swirl * curl_radius
                 + swirl_tangent * (curl_radius * t * angular_speed);
        };

        auto strand_material = strand_materials[i % strand_materials.size()];
        for (int piece = 0; piece < curve_pieces; ++piece) {
            double t0 = double(piece) / curve_pieces;
            double t1 = double(piece + 1) / curve_pieces;
            double dt = t1 - t0;
            point3 p0 = centerline(t0);
            point3 p1 = centerline(t1);
            vec3 d0 = tangent(t0);
            vec3 d1 = tangent(t1);
            std::array<point3, 4> control_points = {
                p0,
                p0 + d0 * (dt / 3.0),
                p1 - d1 * (dt / 3.0),
                p1
            };

            add_bezier_segments(world, control_points,
                strand_radius * (1.0 - 0.65 * t0),
                strand_radius * (1.0 - 0.65 * t1),
                strand_material, 1);
        }
    }

    world = hittable_list(make_shared<bvh_node>(world));

    camera cam;
    cam.aspect_ratio = 9.0 / 9.0;
    cam.image_width = 2000;
    cam.samples_per_pixel = 1;
    cam.max_depth = 50;
    cam.background        = color(0.70, 0.80, 1.00);
    cam.vfov = 42;
    cam.lookfrom = point3(0, 2.0, 8.5);
    cam.lookat = point3(0, 0, 0);
    cam.vup = vec3(0, 1, 0);
    cam.defocus_angle = 0;
    cam.focus_dist = 8.8;
    cam.render(world);
}


int main() {
    int i = 10;
    switch (i) {
        case 1: spheres();                  break;
        case 2: checkered_spheres();        break;
        case 3: strands();                  break;
        case 4: bezier_test();              break;
        case 5: curly_sphere();             break;
        case 6: earth();                    break;    
        case 7: perlin_spheres();           break;
        case 8: quads();                    break;
        case 9: simple_light();             break;
        case 10: cornell_box();             break;

    }
}

