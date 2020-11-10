
#include "sphere_collision.hpp"

#include <random>

#ifdef SCENE_SPHERE_COLLISION

using namespace vcl;







void scene_model::frame_draw(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
    float dt = 0.02f * timer.scale;
    timer.update();

    set_gui();

    create_new_particle();
    compute_time_step(dt);

    display_particles(scene);
    draw(borders, scene.camera);

}

void scene_model::compute_time_step(float dt)
{
    // Set forces
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
        particles[k].f = vec3(0,-9.81f,0);


    // Integrate position and speed of particles through time
    for(size_t k=0; k<N; ++k) {
        particle_structure& particle = particles[k];
        vec3& v = particle.v;
        vec3& p = particle.p;
        vec3 const& f = particle.f;

        v = (1-0.9f*dt) * v + dt * f; // gravity + friction force
        p = p + dt * v;
    }

    const float bounce = 0.8f;
    // Collisions with cube

    // Collisions between spheres
    // ... to do
    for (size_t i = 0; i < N; i++) {
        particle_structure& particle1 = particles[i];
        vec3& p1 = particle1.p;
        vec3& v1 = particle1.v;

        for (size_t j = 0; j < N; j++) {
            if (j == i)
                continue;
            particle_structure &particle2 = particles[j];
            vec3& p2 = particle2.p;
            vec3& v2 = particle2.v;

            vec3 delta_p = p1 - p2;
            if (norm(delta_p) <= particle1.r + particle2.r)
            {
                float rel_speed = norm(dot(normalize(v1), normalize(v2)) * (v1 + v2));
                //std::cout << rel_speed << "\n";
                vec3 delta_norm = normalize(delta_p);
                if (rel_speed > 0.1f) {
                    auto norm_v1 = dot(v1, delta_norm);
                    auto norm_v2 = dot(v2, delta_norm);
                    //std::cout << norm_v1 << " " << norm_v2 << "\n";
                    v1 -= delta_norm * (norm_v1 - norm_v2 * bounce);
                    v2 -= delta_norm * (norm_v2 - norm_v1 * bounce);
                    //std::cout << dot(v1, delta_norm) << " " << dot(v2, delta_norm) << "\n";
                } else
                {
                    auto norm_v1 = dot(v1, delta_norm);
                    auto norm_v2 = dot(v2, delta_norm);
                    //std::cout << norm_v1 << " " << norm_v2 << "\n";
                    v1 -= delta_norm * (norm_v1);
                    v2 -= delta_norm * (norm_v2);
                }
                float delta_d = (particle1.r + particle2.r) - norm(delta_p);

                p1 += delta_norm * (delta_d / 2);
                p2 -= delta_norm * (delta_d / 2);
                //p1.x = clamp(p1.x, -1 + particle1.r + 0.001f, 1 - particle1.r - 0.001f);
                //p1.y = clamp(p1.y, -1 + particle1.r + 0.001f, 1 - particle1.r - 0.001f);
                //p1.z = clamp(p1.z, -1 + particle1.r + 0.001f, 1 - particle1.r - 0.001f);
            }
        }
    }

    for (size_t i = 0; i < N; i++) {
        particle_structure& particle = particles[i];
        vec3& p = particle.p;
        vec3& v = particle.v;

        if (p.x + particle.r >= 1 or p.x - particle.r <= -1)
            v.x = -v.x * bounce;
        p.x = clamp(p.x, -1 + particle.r + 0.001f, 1 - particle.r - 0.001f);
        if (p.y + particle.r >= 1 or p.y - particle.r <= -1)
            v.y = -v.y * bounce;
        p.y = clamp(p.y, -1 + particle.r + 0.001f, 1 - particle.r - 0.001f);
        if (p.z + particle.r >= 1 or p.z - particle.r <= -1)
            v.z = -v.z * bounce;
        p.z = clamp(p.z, -1 + particle.r + 0.001f, 1 - particle.r - 0.001f);
        //p = p + dt * v;
    }
}


void scene_model::create_new_particle()
{
    // Emission of new particle if needed
    timer.periodic_event_time_step = gui_scene.time_interval_new_sphere;
    const bool is_new_particle = timer.event;
    static const std::vector<vec3> color_lut = {{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1}};

    if( is_new_particle && gui_scene.add_sphere)
    {
        particle_structure new_particle;

        new_particle.r = 0.08f;
        new_particle.c = color_lut[int(rand_interval()*color_lut.size())];

        // Initial position
        new_particle.p = vec3(0,0,0);

        // Initial speed
        const float theta = rand_interval(0, 2*3.14f);
        new_particle.v = vec3( 2*std::cos(theta), 5.0f, 2*std::sin(theta));

        particles.push_back(new_particle);

    }
}
void scene_model::display_particles(scene_structure& scene)
{
    const size_t N = particles.size();
    for(size_t k=0; k<N; ++k)
    {
        const particle_structure& part = particles[k];

        sphere.uniform.transform.translation = part.p;
        sphere.uniform.transform.scaling = part.r;
        sphere.uniform.color = part.c;
        draw(sphere, scene.camera);
    }
}




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    sphere = mesh_drawable( mesh_primitive_sphere(1.0f));
    sphere.shader = shaders["mesh"];

    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);

    bool stop_anim  = ImGui::Button("Stop"); ImGui::SameLine();
    bool start_anim = ImGui::Button("Start");

    if(stop_anim)  timer.stop();
    if(start_anim) timer.start();
}





#endif
