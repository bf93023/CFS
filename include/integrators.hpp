#ifndef INTEGRATORS_HPP
#define INTEGRATORS_HPP

#include <entt.hpp>
#include <functional>

// Define the "Force Pipeline" function type again for clarity in this file
using ForcePipelineFn = std::function<void(entt::registry&)>;


// --- Integrator Implementations ---

/**
 * @brief Integrator: 4th Order Runge-Kutta (RK4)
 * This function performs one full RK4 step. It now accepts the
 * force pipeline as an argument, which it calls 4 times.
 */
void IntegrateRK4(entt::registry& registry, double dt, ForcePipelineFn calculate_forces);


/**
 * @brief Integrator: Velocity Verlet (Part 1)
 * Part 1:
 * - Update velocities to mid-step: v(t + dt/2) = v(t) + a(t) * dt/2
 * - Update positions to full-step: x(t + dt) = x(t) + v(t + dt/2) * dt
 */
void IntegrateVelocityVerlet_Part1(entt::registry& registry, double dt);

/**
 * @brief Integrator: Velocity Verlet (Part 2)
 * Part 2 (must be called *after* force pipeline has been run):
 * - Update velocities to full-step: v(t + dt) = v(t + dt/2) + a(t + dt) * dt/2
 */
void IntegrateVelocityVerlet_Part2(entt::registry& registry, double dt);


#endif // INTEGRATORS_HPP
