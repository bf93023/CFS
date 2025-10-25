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
