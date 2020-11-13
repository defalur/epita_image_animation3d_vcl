
#include "trampoline.h"

#include <random>

#ifdef TRAMPOLINE

using namespace vcl;

void scene_model::particle_collision()
{
    // Set forces
    const size_t N = particles.size();


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
                particle2.f += particle1.f * clamp(vcl::dot(delta_p, particle1.f), 0.f, 1.f);
            }
        }
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
        new_particle.v = vec3( std::cos(theta), 5.0f, std::sin(theta));

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




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& gui)
{
    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = segments_gpu(borders_segments);
    borders.uniform.color = {0,0,0};
    borders.shader = shaders["curve"];

    gui.show_frame_camera = false;

    gui_display_texture = true;
    gui_display_wireframe = false;

    // Load textures
    texture_cloth = create_texture_gpu(image_load_png("scenes/animation/02_simulation/assets/cloth.png"));
    texture_wood  = create_texture_gpu(image_load_png("scenes/animation/02_simulation/assets/wood.png"));
    shader_mesh = shaders["mesh_bf"];

    // Initialize cloth geometry and particles
    initialize();

    // Default value for simulation parameters
    user_parameters.K    = 100.0f;
    user_parameters.m    = 5.0f;
    user_parameters.wind = 0.0f;
    user_parameters.mu   = 0.02f;

    // Set collision shapes
    collision_shapes.sphere_p = {0,0.1f,0};
    collision_shapes.sphere_r = 0.2f;
    collision_shapes.ground_height = 0.1f;

    // Init visual models
    sphere = mesh_drawable(mesh_primitive_sphere(1.0f,{0,0,0},60,60));
    sphere.shader = shaders["mesh"];
    sphere.uniform.color = {1,0,0};

    ground = mesh_drawable(mesh_primitive_quad({-1,collision_shapes.ground_height-1e-3f,-1}, {1,collision_shapes.ground_height-1e-3f,-1}, {1,collision_shapes.ground_height-1e-3f,1}, {-1,collision_shapes.ground_height-1e-3f,1}));
    ground.shader = shaders["mesh_bf"];
    ground.texture_id = texture_wood;
}



void scene_model::set_gui()
{
    // Can set the speed of the animation
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Interval create sphere", &gui_scene.time_interval_new_sphere, 0.05f, 2.0f, "%.2f s");
    ImGui::SliderFloat("Stiffness", &user_parameters.K, 1.0f, 400.0f, "%.2f s");
    ImGui::SliderFloat("Damping", &user_parameters.mu, 0.0f, 0.1f, "%.3f s");
    ImGui::SliderFloat("Mass", &user_parameters.m, 1.0f, 15.0f, "%.2f s");
    ImGui::SliderFloat("Sphere Mass", &sphere_mass, 1.0f, 50.0f, "%.2f s");
    ImGui::SliderFloat("Wind", &user_parameters.wind, 0.0f, 400.0f, "%.2f s");
    ImGui::SliderFloat("Cloth Bounciness", &bounciness, 0.0f, 1.0f, "%.2f s");
    ImGui::Checkbox("Add sphere", &gui_scene.add_sphere);

    ImGui::Checkbox("Wireframe",&gui_display_wireframe);
    ImGui::Checkbox("Texture",&gui_display_texture);

    bool const stop  = ImGui::Button("Stop anim"); ImGui::SameLine();
    bool const start = ImGui::Button("Start anim");

    if(stop)  timer.stop();
    if(start) {
        if( simulation_diverged )
            force_simulation=true;
        timer.start();
    }

    bool const restart = ImGui::Button("Restart");
    if(restart) initialize();
}


vec3 spring_force(const vec3& pi, const vec3& pj, float L0, float K)
{
    vec3 const pji = pj - pi;
    float const L = norm(pji);
    return K * (L - L0) * pji / L;
}

bool check_bounds(int u, int v, int dim)
{
    return u >= 0 and v >= 0 and u < dim and v < dim;
}

// Fill value of force applied on each particle
// - Gravity
// - Drag
// - Spring force
// - Wind force
void scene_model::compute_forces()
{
    const size_t N = force.size();        // Total number of particles of the cloth Nu x Nv
    const int N_dim = int(force.dimension[0]); // Number of particles along one dimension (square dimension)

    simulation_parameters.m = user_parameters.m / float(N); // Constant total mass

    // Get simuation parameters
    const float K  = user_parameters.K;
    const float m  = simulation_parameters.m;
    const float L0 = simulation_parameters.L0;

    // Gravity
    const vec3 g = {0,-9.81f,0};
    for(size_t k=0; k<N; ++k)
        force[k] = m*g + force_collision[k];


    {
        const size_t N = particles.size();
        for (size_t k = 0; k < N; ++k)
            particles[k].f = g * sphere_mass;
    }

    // Drag
    const float mu = user_parameters.mu;
    for(size_t k=0; k<N; ++k)
        force[k] = force[k]-mu*speed[k];

    struct Pos
    {
        int x;
        int y;
    };

    const Pos neighbours[] = {{-1, 0},
                              {1, 0},
                              {0, -1},
                              {0, 1},
                              {-1, -1},
                              {-1, 1},
                              {1, -1},
                              {1, 1},
                              {-2, 0},
                              {2, 0},
                              {0, -2},
                              {0, 2}};

    // Springs
    for(int ku=0; ku<N_dim; ++ku) {
        for(int kv=0; kv<N_dim; ++kv) {
            for (unsigned i = 0; i < 12; i++)
            {
                auto neighbour = neighbours[i];
                if (not check_bounds(ku + neighbour.x, kv + neighbour.y, N_dim))
                {
                    continue;
                }

                if (neighbour.x != 0 and neighbour.y != 0)
                    force[ku + kv * N_dim] += spring_force(position[ku + kv * N_dim],
                                                           position[ku + neighbour.x + (kv + neighbour.y) * N_dim], L0 * sqrt(2), K);
                else
                    force[ku + kv * N_dim] += spring_force(position[ku + kv * N_dim],
                                                           position[ku + neighbour.x + (kv + neighbour.y) * N_dim], L0 * (abs(neighbour.x) + abs(neighbour.y)), K);
            }

            //auto delta_u = 1;
            //auto delta_v = 1;

            //if (ku + delta_u >= N_dim)
            //    delta_u = -1;
            //if (kv + delta_v >= N_dim)
            //    delta_v = -1;

            //auto normal = normalize(cross(position[(ku + delta_u) + kv * N_dim], position[ku + (kv + delta_v) * N_dim]));
            auto normal = normals[ku + kv * N_dim];
            force[ku + kv * N_dim] += vec3(-0.001, 0, 0) * user_parameters.wind * fabs(dot(vec3(1, 0, 0), normal));
        }
    }
}


// Handle detection and response to collision with the shape described in "collision_shapes" variable
void scene_model::collision_constraints(float h)
{
    // Handle collisions here (with the ground and the sphere)
    // ...
    const size_t N = position.size();
    for (size_t i = 0; i < N; i++)
    {
        force_collision[i] = vec3(0.f,0.f,0.f);
    }
    const size_t H = particles.size();
    //for(size_t k=0; k<N; ++k) {
        /*if (position[k].y < collision_shapes.ground_height)
            position[k].y = collision_shapes.ground_height;
        if (norm(position[k] - collision_shapes.sphere_p) < collision_shapes.sphere_r + 0.005f)
        {
            auto normal = normalize(position[k] - collision_shapes.sphere_p);
            position[k] = normal * (collision_shapes.sphere_r + 0.005f) + collision_shapes.sphere_p;
            speed[k] -= normal * dot(speed[k], normal);
        }*/

    //}
    float bounce = 0.5f;
    for (size_t i = 0; i < N; i++) {
        vec3& p1 = position[i];
        vec3& v1 = speed[i];

        for (size_t j = 0; j < H; j++) {
            particle_structure &particle2 = particles[j];
            vec3& p2 = particle2.p;
            vec3& v2 = particle2.v;

            vec3 delta_p = p1 - p2;
            if (norm(delta_p) <= particle2.r + 0.01f)
            {
                float rel_speed = norm(dot(normalize(v1), normalize(v2)) * (v1 + v2));
                //std::cout << rel_speed << "\n";
                vec3 delta_norm = normalize(delta_p);
                if (rel_speed > 0.01f) {
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
                float delta_d = (particle2.r + 0.01f) - norm(delta_p);

                p1 += delta_norm * (delta_d / 2);
                p2 -= delta_norm * (delta_d / 2);
                force_collision[i] += delta_norm * norm(particle2.f)
                                      * clamp(vcl::dot(delta_norm, normalize(particle2.f)),
                                              0.f, 1.f) * bounciness;
            }
        }
    }
}



// Initialize the geometrical model
void scene_model::initialize()
{
    // Number of samples of the model (total number of particles is N_cloth x N_cloth)
    const size_t N_cloth = 50;

    // Rest length (length of an edge)
    simulation_parameters.L0 = 2.0f/float(N_cloth-1);

    // Create cloth mesh in its initial position
    // Horizontal grid of length 1 x 1
    const mesh base_cloth = mesh_primitive_grid(N_cloth,N_cloth,{-1.f,-1.f,-1.f},{2.f,0,0},{0,0,2.f});

    // Set particle position from cloth geometry
    position = buffer2D_from_vector(base_cloth.position, N_cloth, N_cloth);

    // Set hard positional constraints
    positional_constraints[0] = position[0];
    positional_constraints[N_cloth*(N_cloth-1)] = position[N_cloth*(N_cloth-1)];
    positional_constraints[N_cloth - 1] = position[N_cloth - 1];
    positional_constraints[N_cloth*N_cloth - 1] = position[N_cloth * N_cloth - 1];

    // Init particles data (speed, force)
    speed.resize(position.dimension); speed.fill({0,0,0});
    force.resize(position.dimension); force.fill({0,0,0});
    force_collision.resize(position.dimension); force_collision.fill({0,0,0});


    // Store connectivity and normals
    connectivity = base_cloth.connectivity;
    normals      = normal(position.data,connectivity);

    cloth.clear();
    cloth = mesh_drawable(base_cloth);
    cloth.uniform.shading.specular = 0.0f;
    cloth.shader = shader_mesh;
    cloth.texture_id = texture_cloth;

    simulation_diverged = false;
    // Send data to GPU
    force_simulation    = false;

    timer.update();

    particles.clear();
}

void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
    const float dt = timer.update();
    //float dt = 0.02f * timer.scale;
    //timer.update();
    set_gui();

    create_new_particle();
    particle_collision();

    // Force constant simulation time step
    float h = dt<=1e-6f? 0.0f : timer.scale*0.001f;

    if( (!simulation_diverged || force_simulation) && h>0)
    {
        // Iterate over a fixed number of substeps between each frames
        const size_t number_of_substeps = 4;
        for(size_t k=0; (!simulation_diverged  || force_simulation) && k<number_of_substeps; ++k)
        {
            compute_forces();
            numerical_integration(h);
            collision_constraints(h);                 // Detect and solve collision with other shapes

            hard_constraints();                      // Enforce hard positional constraints

            normal(position.data, connectivity, normals); // Update normals of the cloth
            detect_simulation_divergence();               // Check if the simulation seems to diverge
        }
    }


    cloth.update_position(position.data);
    cloth.update_normal(normals.data);

    display_elements(shaders, scene, gui);
    display_particles(scene);

    //draw(borders, scene.camera);

    {
        for (size_t i = 0; i < particles.size(); i++)
        {
            if (norm(particles[i].p) > 20.0f)
            {
                particles[i] = particles[particles.size() - 1];
                particles.pop_back();
            }
        }
    }
}

void scene_model::numerical_integration(float h)
{
    const size_t NN = position.size();
    const float m = simulation_parameters.m;

    for(size_t k=0; k<NN; ++k)
    {
        vec3& p = position[k];
        vec3& v = speed[k];
        const vec3& f = force[k];

        v = v + h*f/m;
        p = p + h*v;
    }

    for (size_t i = 0; i < particles.size(); i++)
    {
        vec3& p = particles[i].p;
        vec3& v = particles[i].v;
        const vec3& f = particles[i].f;

        v = (1 - 0.8f * h) * v + h*f/sphere_mass;
        p = p + h*v;
    }
}

void scene_model::hard_constraints()
{
    // Fixed positions of the cloth
    for(const auto& constraints : positional_constraints)
        position[constraints.first] = constraints.second;
}


void scene_model::display_elements(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    glEnable( GL_POLYGON_OFFSET_FILL );

    // Display cloth
    GLuint texture = cloth.texture_id;
    if(!gui_display_texture)
        texture = scene.texture_white;

    glPolygonOffset( 1.0, 1.0 );
    draw(cloth, scene.camera, cloth.shader, texture);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);

    if(gui_display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        draw(cloth, scene.camera, shaders["wireframe_quads"]);
    }

    // Display positional constraint using spheres
    sphere.uniform.transform.scaling = 0.02f;
    for(const auto& constraints : positional_constraints)  {
        sphere.uniform.color = vec3{0, 1, 0};
        sphere.uniform.transform.translation = constraints.second;
        draw(sphere, scene.camera, shaders["mesh"]);
    }

    // Display ground
    //draw(ground, scene.camera);
    glBindTexture(GL_TEXTURE_2D, scene.texture_white);
}

void scene_model::display(std::map<std::string,GLuint>& shaders,
             scene_structure& scene,
             gui_structure& gui)
{
    display_particles(scene);
    display_elements(shaders, scene, gui);
}

// Automatic detection of divergence: stop the simulation if detected
void scene_model::detect_simulation_divergence()
{
    const size_t NN = position.size();
    for(size_t k=0; simulation_diverged==false && k<NN; ++k)
    {
        const float f = norm(force[k]);
        const vec3& p = position[k];

        if( std::isnan(f) ) // detect NaN in force
        {
            std::cout<<"NaN detected in forces"<<std::endl;
            simulation_diverged = true;
        }

        if( f>1000.0f ) // detect strong force magnitude
        {
            std::cout<<" **** Warning : Strong force magnitude detected "<<f<<" at vertex "<<k<<" ****"<<std::endl;
            simulation_diverged = true;
        }

        if( std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) ) // detect NaN in position
        {
            std::cout<<"NaN detected in positions"<<std::endl;
            simulation_diverged = true;
        }

        if(simulation_diverged==true)
        {
            std::cerr<<" **** Simulation has diverged **** "<<std::endl;
            std::cerr<<" > Stop simulation iterations"<<std::endl;
            timer.stop();
        }
    }

}


#endif
