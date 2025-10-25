// /**
//  * N-Body Spaceflight Simulator using EnTT
//  *
//  * This program simulates N-body gravitational interactions using an
//  * Entity Component System (ECS) architecture provided by the EnTT library.
//  *
//  * Features:
//  * - ECS design via EnTT for modularity.
//  * - N-Body gravitation ($F = G * (m1*m2) / r^2).
//  * - Specifiable integrators:
//  * - 4th Order Runge-Kutta (RK4)
//  * - Velocity Verlet (a symplectic integrator)
//  * - Object loading from an external file (`input.txt`).
//  * - Trajectory path logging for all objects.
//  * - File output (`output.dat`) for parsing and plotting.
//  * - Configurable simulation rate (unlocked or fixed FPS).
//  *
//  * Compile (as requested):
//  * g++ .\src\main.cpp -o main.exe -std=c++17 -I ..\entt -O3
//  * (Assuming 'entt' folder is one level up from 'src')
//  * Or use the provided compile.bat
//  */

// #include <entt.hpp>
// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <fstream>
// #include <sstream>
// #include <iomanip>
// #include <chrono>
// #include <thread>
// #include <string>
// #include <map>
// #include <functional>

// // --- Constants ---
// // Gravitational Constant (m^3 kg^-1 s^-2)
// constexpr double G = 6.67430e-11;

// // A softening factor to prevent singularities (division by zero)
// // when particles get too close. This is common in N-body simulations.
// constexpr double SOFTENING_FACTOR_SQUARED = 1e-9; // (meters^2)

// // --- Helper: 3D Vector ---
// // Using a simple Vec3 struct with operator overloading
// // makes the integrator code *much* cleaner and more readable.
// struct Vec3 {
//     double x = 0.0, y = 0.0, z = 0.0;

//     Vec3& operator+=(const Vec3& rhs) {
//         x += rhs.x; y += rhs.y; z += rhs.z;
//         return *this;
//     }
//     Vec3& operator-=(const Vec3& rhs) {
//         x -= rhs.x; y -= rhs.y; z -= rhs.z;
//         return *this;
//     }
//     Vec3& operator*=(double scalar) {
//         x *= scalar; y *= scalar; z *= scalar;
//         return *this;
//     }
//     Vec3& operator/=(double scalar) {
//         x /= scalar; y /= scalar; z /= scalar;
//         return *this;
//     }
// };

// // Non-member operators
// Vec3 operator+(Vec3 lhs, const Vec3& rhs) { return lhs += rhs; }
// Vec3 operator-(Vec3 lhs, const Vec3& rhs) { return lhs -= rhs; }
// Vec3 operator*(Vec3 lhs, double scalar) { return lhs *= scalar; }
// Vec3 operator*(double scalar, Vec3 rhs) { return rhs *= scalar; }
// Vec3 operator/(Vec3 lhs, double scalar) { return lhs /= scalar; }


// // --- Components ---
// // Components are simple data-only structs.

// struct Position { Vec3 p; };
// struct Velocity { Vec3 v; };
// struct Acceleration { Vec3 a; };
// struct Mass { double m; };
// struct Name { std::string name; };

// // This component will store the object's path for output.
// struct Trajectory {
//     std::vector<Vec3> path;
// };


// // --- Systems ---
// // Systems are free functions that operate on entities
// // possessing a certain set of components.

// /**
//  * @brief Calculates gravitational forces for all entities.
//  * This is the core $O(n^2)$ physics calculation.
//  * It reads Position and Mass, and writes Acceleration.
//  */
// void GravitySystem(entt::registry& registry) {
//     // Get a view of all entities that have Position, Mass, and Acceleration
//     auto view = registry.view<Position, Mass, Acceleration>();

//     // 1. Reset all accelerations to zero
//     for (auto entity : view) {
//         auto& acc = view.get<Acceleration>(entity);
//         acc.a = {0.0, 0.0, 0.0};
//     }

//     // 2. Iterate over all unique pairs of entities
//     for (auto entity_i : view) {
//         // Get components for object 'i'
//         auto& pos_i = view.get<Position>(entity_i);
//         auto& mass_i = view.get<Mass>(entity_i);
//         auto& acc_i = view.get<Acceleration>(entity_i);

//         for (auto entity_j : view) {
//             if (entity_i == entity_j) continue; // Don't interact with self

//             // Get components for object 'j'
//             const auto& pos_j = view.get<Position>(entity_j);
//             const auto& mass_j = view.get<Mass>(entity_j);

//             // Calculate vector from i to j
//             Vec3 r_vec = pos_j.p - pos_i.p;
            
//             // Calculate squared distance
//             double r_sq = r_vec.x * r_vec.x + 
//                           r_vec.y * r_vec.y + 
//                           r_vec.z * r_vec.z;

//             // Add softening factor to avoid singularity
//             r_sq += SOFTENING_FACTOR_SQUARED;

//             // Calculate magnitude of the force: F = G * (m1*m2) / r^2
//             double r = std::sqrt(r_sq);
//             double force_mag = G * mass_i.m * mass_j.m / r_sq;
            
//             // Calculate force vector: F_vec = F_mag * (r_vec / r)
//             Vec3 force_vec = (r_vec / r) * force_mag;

//             // Calculate acceleration for 'i': a = F / m
//             Vec3 acc_vec = force_vec / mass_i.m;
            
//             // Accumulate acceleration on 'i'
//             acc_i.a += acc_vec;
//         }
//     }
// }

// /**
//  * @brief Logs the current position to the entity's Trajectory component.
//  */
// void UpdateTrajectorySystem(entt::registry& registry) {
//     auto view = registry.view<Position, Trajectory>();
//     for (auto entity : view) {
//         const auto& pos = view.get<Position>(entity);
//         auto& traj = view.get<Trajectory>(entity);
//         traj.path.push_back(pos.p);
//     }
// }


// // --- Integrator Implementations ---

// /**
//  * @brief Integrator: 4th Order Runge-Kutta (RK4)
//  * This function performs one full RK4 step. It is complex because
//  * it must evaluate the forces (run GravitySystem) 4 times at
//  * intermediate states.
//  */
// void IntegrateRK4(entt::registry& registry, double dt) {
//     auto view = registry.view<Position, Velocity, Mass, Acceleration>();

//     // We need to store intermediate states for all entities.
//     // Using maps is a clean way to do this without adding/removing
//     // temporary components to the registry.
//     std::map<entt::entity, Vec3> k1_vel, k1_accel;
//     std::map<entt::entity, Vec3> k2_vel, k2_accel;
//     std::map<entt::entity, Vec3> k3_vel, k3_accel;
//     std::map<entt::entity, Vec3> k4_vel, k4_accel;
//     std::map<entt::entity, Position> original_pos;
//     std::map<entt::entity, Velocity> original_vel;

//     // --- 1. Get k1 (state at t) ---
//     // Store original state
//     for (auto entity : view) {
//         original_pos[entity] = view.get<Position>(entity);
//         original_vel[entity] = view.get<Velocity>(entity);
//     }
    
//     // Calculate accelerations at t
//     GravitySystem(registry);
//     for (auto entity : view) {
//         k1_vel[entity] = view.get<Velocity>(entity).v;
//         k1_accel[entity] = view.get<Acceleration>(entity).a;
//     }

//     // --- 2. Get k2 (state at t + dt/2, using k1) ---
//     // Update all entities to intermediate state
//     for (auto entity : view) {
//         view.get<Position>(entity).p = original_pos[entity].p + k1_vel[entity] * (dt / 2.0);
//         view.get<Velocity>(entity).v = original_vel[entity].v + k1_accel[entity] * (dt / 2.0);
//     }

//     // Calculate accelerations at t + dt/2
//     GravitySystem(registry);
//     for (auto entity : view) {
//         k2_vel[entity] = view.get<Velocity>(entity).v;
//         k2_accel[entity] = view.get<Acceleration>(entity).a;
//     }

//     // --- 3. Get k3 (state at t + dt/2, using k2) ---
//     // Revert and update to new intermediate state
//     for (auto entity : view) {
//         view.get<Position>(entity).p = original_pos[entity].p + k2_vel[entity] * (dt / 2.0);
//         view.get<Velocity>(entity).v = original_vel[entity].v + k2_accel[entity] * (dt / 2.0);
//     }

//     // Calculate accelerations
//     GravitySystem(registry);
//     for (auto entity : view) {
//         k3_vel[entity] = view.get<Velocity>(entity).v;
//         k3_accel[entity] = view.get<Acceleration>(entity).a;
//     }

//     // --- 4. Get k4 (state at t + dt, using k3) ---
//     // Revert and update to final intermediate state
//     for (auto entity : view) {
//         view.get<Position>(entity).p = original_pos[entity].p + k3_vel[entity] * dt;
//         view.get<Velocity>(entity).v = original_vel[entity].v + k3_accel[entity] * dt;
//     }

//     // Calculate accelerations
//     GravitySystem(registry);
//     for (auto entity : view) {
//         k4_vel[entity] = view.get<Velocity>(entity).v;
//         k4_accel[entity] = view.get<Acceleration>(entity).a;
//     }

//     // --- 5. Apply final RK4 update ---
//     // Revert to original state, then apply the full weighted average
//     for (auto entity : view) {
//         auto& pos = view.get<Position>(entity);
//         auto& vel = view.get<Velocity>(entity);

//         pos = original_pos[entity]; // Revert
//         vel = original_vel[entity]; // Revert

//         // y_new = y_old + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
//         pos.p += (k1_vel[entity] + 2.0 * k2_vel[entity] + 2.0 * k3_vel[entity] + k4_vel[entity]) * (dt / 6.0);
//         vel.v += (k1_accel[entity] + 2.0 * k2_accel[entity] + 2.0 * k3_accel[entity] + k4_accel[entity]) * (dt / 6.0);
//     }
// }


// /**
//  * @brief Integrator: Velocity Verlet (Part 1)
//  * This is a 3-part system (Part1, Gravity, Part2)
//  * Part 1:
//  * - Update velocities to mid-step: v(t + dt/2) = v(t) + a(t) * dt/2
//  * - Update positions to full-step: x(t + dt) = x(t) + v(t + dt/2) * dt
//  */
// void IntegrateVelocityVerlet_Part1(entt::registry& registry, double dt) {
//     auto view = registry.view<Position, Velocity, Acceleration>();
//     for (auto entity : view) {
//         auto& pos = view.get<Position>(entity);
//         auto& vel = view.get<Velocity>(entity);
//         const auto& acc = view.get<Acceleration>(entity);

//         // v(t + dt/2)
//         vel.v += acc.a * (dt / 2.0);
        
//         // x(t + dt)
//         pos.p += vel.v * dt;
//     }
// }

// /**
//  * @brief Integrator: Velocity Verlet (Part 2)
//  * Part 2 (must be called *after* GravitySystem has been run):
//  * - Update velocities to full-step: v(t + dt) = v(t + dt/2) + a(t + dt) * dt/2
//  */
// void IntegrateVelocityVerlet_Part2(entt::registry& registry, double dt) {
//     auto view = registry.view<Velocity, Acceleration>();
//     for (auto entity : view) {
//         auto& vel = view.get<Velocity>(entity);
//         const auto& acc = view.get<Acceleration>(entity); // This is the *new* accel at t+dt

//         // v(t + dt)
//         vel.v += acc.a * (dt / 2.0);
//     }
// }


// // --- File I/O ---

// /**
//  * @brief Loads celestial bodies from a text file.
//  * Format: name x y z vx vy vz mass
//  */
// bool loadObjectsFromFile(entt::registry& registry, const std::string& filename) {
//     std::ifstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Error: Could not open input file: " << filename << std::endl;
//         return false;
//     }

//     std::cout << "Loading objects from " << filename << "..." << std::endl;
//     std::string line;
//     int count = 0;
//     while (std::getline(file, line)) {
//         // Skip comments and empty lines
//         if (line.empty() || line[0] == '#') {
//             continue;
//         }

//         std::stringstream ss(line);
//         std::string name;
//         double x, y, z, vx, vy, vz, mass;

//         if (ss >> name >> x >> y >> z >> vx >> vy >> vz >> mass) {
//             auto entity = registry.create();
//             registry.emplace<Name>(entity, name);
//             registry.emplace<Position>(entity, Vec3{x, y, z});
//             registry.emplace<Velocity>(entity, Vec3{vx, vy, vz});
//             registry.emplace<Mass>(entity, mass);
//             registry.emplace<Acceleration>(entity, Vec3{0, 0, 0});
//             registry.emplace<Trajectory>(entity); // Add empty trajectory log
            
//             std::cout << "  Loaded: " << name << " (Mass: " << mass << ")" << std::endl;
//             count++;
//         } else {
//             std::cerr << "Warning: Skipping malformed line: " << line << std::endl;
//         }
//     }
//     std::cout << "Loaded " << count << " objects." << std::endl;
//     return true;
// }

// /**
//  * @brief Writes the simulation results to an output file.
//  * Format is designed to be easily parsable.
//  */
// void writeOutput(entt::registry& registry, const std::string& filename) {
//     std::ofstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Error: Could not open output file: " << filename << std::endl;
//         return;
//     }

//     std::cout << "Writing simulation output to " << filename << "..." << std::endl;
//     // Use high precision for output data
//     file << std::fixed << std::setprecision(8);

//     auto view = registry.view<Name, Mass, Trajectory>();
//     for (auto entity : view) {
//         const auto& name = view.get<Name>(entity);
//         const auto& mass = view.get<Mass>(entity);
//         const auto& traj = view.get<Trajectory>(entity);

//         file << "ObjectName: " << name.name << "\n";
//         file << "Mass: " << mass.m << "\n";
//         file << "Path:\n";
        
//         for (const auto& pos : traj.path) {
//             file << pos.x << ", " << pos.y << ", " << pos.z << "\n";
//         }
        
//         file << "EndObject\n\n";
//     }
//     std::cout << "Output file written." << std::endl;
// }


// // --- Main ---
// int main() {
//     // --- Simulation Parameters ---
    
//     // Total simulation time (seconds)
//     double t_max = 3.154e7; // ~1 year
    
//     // Time step (seconds)
//     double dt = 3600; // 1 hour
    
//     // Log position every N steps
//     int steps_per_output = 24; // Every 24 hours (1 day)

//     // Desired simulation rate (frames per second)
//     // 0 = unlocked (run as fast as possible for computation)
//     // 60 = run at 60 simulation steps per real-time second
//     double sim_rate_fps = 0.0;
    
//     // Integrator Choice: "rk4" or "verlet"
//     std::string integrator_choice = "rk4";
    
//     // --- End Parameters ---


//     entt::registry registry;

//     // Load initial conditions from file
//     if (!loadObjectsFromFile(registry, "input.txt")) {
//         std::cerr << "Failed to load objects. Exiting." << std::endl;
//         // As a fallback, you could hard-code objects here
//         return 1;
//     }

//     // Main simulation loop
//     std::cout << "Starting simulation with integrator: " << integrator_choice << std::endl;
//     std::cout << "Simulating " << t_max << "s with dt=" << dt << "s" << std::endl;
    
//     auto sim_start_time = std::chrono::high_resolution_clock::now();
    
//     std::chrono::duration<double, std::milli> desired_frame_time_ms(0);
//     if (sim_rate_fps > 0) {
//         desired_frame_time_ms = std::chrono::duration<double, std::milli>(1000.0 / sim_rate_fps);
//     }

//     int step_count = 0;
//     for (double t = 0; t < t_max; t += dt) {
//         auto loop_start_time = std::chrono::high_resolution_clock::now();

//         // --- Core Simulation Step ---

//         if (integrator_choice == "verlet") {
//             // Velocity Verlet is a 3-part process
//             IntegrateVelocityVerlet_Part1(registry, dt);
//             GravitySystem(registry); // Calculates new a(t + dt)
//             IntegrateVelocityVerlet_Part2(registry, dt);
        
//         } else { // Default to RK4
//             // RK4 is a single (but complex) function
//             IntegrateRK4(registry, dt);
//             // GravitySystem is called *inside* IntegrateRK4, so
//             // the accelerations in the registry are for t+dt
//         }
        
//         // --- End Core Step ---

//         // Log trajectory
//         if (step_count % steps_per_output == 0) {
//             UpdateTrajectorySystem(registry);
//         }

//         step_count++;

//         // Handle simulation rate
//         if (sim_rate_fps > 0) {
//             auto loop_end_time = std::chrono::high_resolution_clock::now();
//             std::chrono::duration<double, std::milli> work_time_ms = loop_end_time - loop_start_time;
//             auto sleep_duration = desired_frame_time_ms - work_time_ms;
            
//             if (sleep_duration.count() > 0) {
//                 std::this_thread::sleep_for(sleep_duration);
//             }
//         }

//         // Simple progress indicator
//         if (step_count % 100 == 0) {
//             std::cout << "Sim time: " << std::fixed << std::setprecision(2) 
//                       << t / (3600.0 * 24.0) << " days (" 
//                       << (t / t_max) * 100.0 << "%)\r" << std::flush;
//         }
//     }

//     auto sim_end_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> sim_duration = sim_end_time - sim_start_time;
    
//     std::cout << "\nSimulation finished." << std::endl;
//     std::cout << "Total real time elapsed: " << sim_duration.count() << " seconds." << std::endl;

//     // Write final data to output file
//     writeOutput(registry, "output.dat");

//     return 0;
// }


































/**
 * N-Body Spaceflight Simulator (Refactored)
 *
 * This refactor introduces a Simulation class to manage the main loop,
 * supporting both "headless" (max-speed) and "real-time" (wall-clock-based)
 * operation.
 *
 * It also abstracts the physics calculations into a "force pipeline",
 * allowing new physics (like thrust, drag, etc.) to be "bolted on"
 * without modifying the integrators.
 *
 * Compile:
 * g++ -c ./src/systems.cpp -o ./obj/systems.o -std=c++17 -I ../entt -O3
 * g++ -c ./src/integrators.cpp -o ./obj/integrators.o -std=c++17 -I ../entt -O3
 * g++ -c ./src/main.cpp -o ./obj/main.o -std=c++17 -I ../entt -O3
 * g++ ./obj/main.o ./obj/systems.o ./obj/integrators.o -o main.exe -std=c++17
 * (Or use the provided compile.bat)
 */

#include <entt.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <functional>
#include <chrono>
#include <thread>
#include <iomanip>

#include "components.hpp"
#include "systems.hpp"
#include "integrators.hpp"

// --- Type Definitions for a Pluggable Architecture ---

// A function that calculates all forces and updates accelerations in the registry.
// This is our "pluggable" pipeline.
using ForcePipelineFn = std::function<void(entt::registry&)>;

// A function that performs one complete integration step.
// It takes the physics 'dt' and the force pipeline to use.
using IntegratorStepFn = std::function<void(entt::registry&, double, ForcePipelineFn)>;


// --- Simulation Configuration ---

enum class SimMode {
    HEADLESS, // Run as fast as possible
    REALTIME  // Run based on wall-clock
};

struct SimConfig {
    std::string input_file = "input.txt";
    std::string output_file = "output.dat";
    std::string integrator_name = "rk4";
    
    double t_max = 3.154e7; // ~1 year
    double dt = 3600;       // 1 hour (this is the *physics* timestep)
    
    // How many 'dt' steps pass before we log to the Trajectory component?
    int steps_per_log = 24; // Every 24 hours (1 day)

    // --- Realtime-Mode-Only Settings ---
    SimMode mode = SimMode::HEADLESS;
    
    // Time scale multiplier.
    // 1.0 = real-time (1 sim second per 1 wall second)
    // 3600.0 = 1 hour sim per 1 wall second
    // 0.0 = paused
    double time_scale = 3600.0 * 24.0; // Default: 1 sim day per real second

    // Target *render* FPS for the main loop (only in REALTIME mode)
    // This is for the *visual* update rate, not the physics rate.
    double render_fps = 60.0;
};


// --- Simulation Class ---

/**
 * @brief Manages the entire simulation state and main loop.
 */
class Simulation {
public:
    Simulation(const SimConfig& config) : m_config(config) {
        // Store global sim state in the registry's "context"
        // This makes it available to any system that needs it.
        m_registry.ctx().emplace<SimState>(
            SimState{0.0, m_config.dt, m_config.time_scale}
        );
    }

    /**
     * @brief Set up the force pipeline and available integrators.
     * This is where you "bolt on" functionality.
     */
    void Initialize() {
        std::cout << "Initializing simulation..." << std::endl;

        // --- 1. Register Integrators ---
        // This map makes the integrator choice fully dynamic.
        m_integrators["rk4"] = IntegrateRK4;
        m_integrators["verlet"] = 
            [](entt::registry& reg, double dt, ForcePipelineFn forces) {
                IntegrateVelocityVerlet_Part1(reg, dt);
                forces(reg); // Call the *entire* force pipeline
                IntegrateVelocityVerlet_Part2(reg, dt);
            };
        
        // Select the active integrator
        if (m_integrators.find(m_config.integrator_name) == m_integrators.end()) {
            std::cerr << "Warning: Integrator '" << m_config.integrator_name 
                      << "' not found. Defaulting to rk4." << std::endl;
            m_config.integrator_name = "rk4";
        }
        m_active_integrator_fn = m_integrators.at(m_config.integrator_name);
        std::cout << "Using integrator: " << m_config.integrator_name << std::endl;

        // --- 2. Build the Force Pipeline ---
        // This is the "pluggable" part for physics.
        // Add systems in the order they should run.
        m_force_pipeline.push_back(ClearAccelerationSystem);
        m_force_pipeline.push_back(GravitySystem);
        
        // ** YOUR FUTURE GOALS GO HERE **
        // e.g., m_force_pipeline.push_back(ThrustSystem);
        // e.g., m_force_pipeline.push_back(RadiationPressureSystem);
        // e.g., m_force_pipeline.push_back(AtmosphericDragSystem);

        // Build the final pipeline function that calls all registered systems
        m_force_pipeline_fn = [this](entt::registry& reg) {
            for (auto& system_fn : m_force_pipeline) {
                system_fn(reg);
            }
        };

        // --- 3. Build the Trajectory Logging Pipeline ---
        m_trajectory_pipeline.push_back(UpdateTrajectorySystem);
    }

    /**
     * @brief Load initial object data from file.
     */
    bool LoadObjects() {
        return loadObjectsFromFile(m_registry, m_config.input_file);
    }

    /**
     * @brief Run the simulation using the configured mode.
     */
    void Run() {
        if (!LoadObjects()) {
            std::cerr << "Failed to load objects. Exiting." << std::endl;
            return;
        }

        m_running = true;
        if (m_config.mode == SimMode::HEADLESS) {
            RunHeadless();
        } else {
            RunRealtime();
        }
    }

    /**
     * @brief Write final simulation output.
     */
    void Shutdown() {
        std::cout << "\nSimulation finished." << std::endl;
        writeOutput(m_registry, m_config.output_file);
    }

private:
    /**
     * @brief Performs a single, complete physics step.
     * @param dt The *physics* delta-time (e.g., 3600.0s)
     */
    void StepSimulation(double dt) {
        // Run the chosen integrator, which will call the force pipeline
        m_active_integrator_fn(m_registry, dt, m_force_pipeline_fn);

        // Update the global simulation time
        m_registry.ctx().get<SimState>().current_time += dt;

        // Log trajectory if it's time
        if (m_step_count % m_config.steps_per_log == 0) {
            for(auto& system_fn : m_trajectory_pipeline) {
                system_fn(m_registry);
            }
        }
        m_step_count++;
    }

    /**
     * @brief Headless mode: Run as fast as possible, fixed loop.
     */
    void RunHeadless() {
        std::cout << "Running in HEADLESS mode (max speed)..." << std::endl;
        std::cout << "Simulating " << m_config.t_max << "s with dt=" 
                  << m_config.dt << "s" << std::endl;
        
        auto sim_start_time = std::chrono::high_resolution_clock::now();
        
        double& t = m_registry.ctx().get<SimState>().current_time;
        for (t = 0; t < m_config.t_max; /* t incremented in StepSimulation */) {
            
            StepSimulation(m_config.dt);

            // Simple progress indicator
            if (m_step_count % 100 == 0) {
                std::cout << "Sim time: " << std::fixed << std::setprecision(2) 
                          << t / (3600.0 * 24.0) << " days (" 
                          << (t / m_config.t_max) * 100.0 << "%)\r" << std::flush;
            }
        }
        
        auto sim_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> sim_duration = sim_end_time - sim_start_time;
        std::cout << "\nTotal real time elapsed: " 
                  << sim_duration.count() << " seconds." << std::endl;
    }

    /**
     * @brief Real-time mode: Run based on wall clock.
     * Implements a fixed-dt game loop.
     */
    void RunRealtime() {
        std::cout << "Running in REALTIME mode..." << std::endl;
        std::cout << "Time scale: " << m_config.time_scale 
                  << "x (0.0 = pause)" << std::endl;
        
        using clock = std::chrono::high_resolution_clock;
        
        const std::chrono::duration<double> render_frame_time(1.0 / m_config.render_fps);
        double accumulator = 0.0;
        
        auto last_wall_time = clock::now();

        while (m_running) {
            // --- 1. Calculate Wall-Clock Delta Time ---
            auto current_wall_time = clock::now();
            std::chrono::duration<double> real_dt_chrono = current_wall_time - last_wall_time;
            last_wall_time = current_wall_time;
            
            // Get delta as a double, and apply the time scale
            double real_dt = real_dt_chrono.count();
            double scaled_dt = real_dt * m_config.time_scale;
            
            // Update time scale in context (in case it's changed by input)
            m_registry.ctx().get<SimState>().time_scale = m_config.time_scale;

            // --- 2. Handle Input ---
            // (e.g., check for 'P' to pause, 'Q' to quit)
            // This is where you'd poll your GUI library
            HandleInput(); 

            // --- 3. Update Physics (Fixed-Step Loop) ---
            // Add the scaled wall time to the accumulator
            accumulator += scaled_dt;

            // Consume the accumulator in fixed-size physics steps
            const double physics_dt = m_config.dt;
            while (accumulator >= physics_dt) {
                StepSimulation(physics_dt);
                accumulator -= physics_dt;
                
                // Safety break for "spiral of death"
                // if sim can't keep up with wall time
                if (accumulator > physics_dt * 100) { 
                    std::cout << "Warning: Simulation can't keep up! Resetting time accumulator." << std::endl;
                    accumulator = 0.0;
                }
            }

            // --- 4. Render / Output ---
            // `accumulator / physics_dt` gives an interpolation factor (0.0-1.0)
            // for smooth graphics between physics steps.
            double interpolation_alpha = accumulator / physics_dt;
            Render(interpolation_alpha);

            // --- 5. Cap Render FPS ---
            auto work_time = clock::now() - current_wall_time;
            if (work_time < render_frame_time) {
                std::this_thread::sleep_for(render_frame_time - work_time);
            }

            // --- 6. Check Exit Conditions ---
            if (m_registry.ctx().get<SimState>().current_time >= m_config.t_max) {
                m_running = false;
            }
        }
    }

    /**
     * @brief Stub for handling real-time input
     */
    void HandleInput() {
        // In a real app, you would poll a library like SDL, SFML, or ImGui
        // For example:
        // if (IsKeyPressed('P')) {
        //     // Toggle pause
        //     m_config.time_scale = (m_config.time_scale == 0.0) ? m_previous_time_scale : 0.0;
        // }
        // if (IsKeyPressed('Q')) {
        //     m_running = false;
        // }
        // if (IsMouseButtonClicked()) {
        //     // Fire Lambert solver, add new entity, etc.
        //     // auto new_entity = m_registry.create();
        //     // ...
        // }
    }

    /**
     * @brief Stub for rendering graphics
     * @param alpha Interpolation factor for smooth motion
     */
    void Render(double alpha) {
        // This is where you would draw your GUI or 3D scene.
        // You would draw each object at:
        // `previous_position + (current_position - previous_position) * alpha`
        // (This requires storing the previous position in a component)
        
        // For now, just print status to the console
        if (m_step_count % (int)m_config.render_fps == 0) { // Update console ~1/sec
             double t = m_registry.ctx().get<SimState>().current_time;
             std::cout << "Sim time: " << std::fixed << std::setprecision(2) 
                       << t / (3600.0 * 24.0) << " days (" 
                       << (t / m_config.t_max) * 100.0 << "%)\r" << std::flush;
        }
    }

private:
    entt::registry m_registry;
    SimConfig m_config;
    bool m_running = false;
    int m_step_count = 0;

    // Pluggable pipelines
    std::vector<std::function<void(entt::registry&)>> m_force_pipeline;
    std::vector<std::function<void(entt::registry&)>> m_trajectory_pipeline;
    
    // The "master" function for all forces
    ForcePipelineFn m_force_pipeline_fn;

    // Integrator management
    std::map<std::string, IntegratorStepFn> m_integrators;
    IntegratorStepFn m_active_integrator_fn;
};


// --- Main ---
int main() {
    
    // --- Simulation Parameters ---
    SimConfig config;
    
    // == Run in Headless (computation) mode ==
    config.mode = SimMode::HEADLESS;
    config.t_max = 3.154e7; // 1 year
    config.dt = 3600;       // 1 hour
    config.steps_per_log = 24;
    config.integrator_name = "rk4";

    // == Run in Realtime (visualization) mode ==
    // config.mode = SimMode::REALTIME;
    // config.t_max = 3.154e7; // 1 year
    // config.dt = 3600;       // 1 hour
    // config.steps_per_log = 1; // Log every step
    // config.integrator_name = "verlet";
    // config.time_scale = 3600.0 * 24.0 * 7; // 1 sim week per real second
    // config.render_fps = 60.0;

    // --- End Parameters ---

    Simulation sim(config);
    
    sim.Initialize();
    sim.Run();
    sim.Shutdown();

    return 0;
}
