#include "integrators.hpp"
#include "components.hpp"

#include <map>

// --- RK4 Implementation ---

void IntegrateRK4(entt::registry& registry, double dt, ForcePipelineFn calculate_forces) {
    auto view = registry.view<Position, Velocity, Mass, Acceleration>();

    // We need to store intermediate states for all entities.
    std::map<entt::entity, Vec3> k1_vel, k1_accel;
    std::map<entt::entity, Vec3> k2_vel, k2_accel;
    std::map<entt::entity, Vec3> k3_vel, k3_accel;
    std::map<entt::entity, Vec3> k4_vel, k4_accel;
    std::map<entt::entity, Position> original_pos;
    std::map<entt::entity, Velocity> original_vel;

    // --- 1. Get k1 (state at t) ---
    // Store original state
    for (auto entity : view) {
        original_pos[entity] = view.get<Position>(entity);
        original_vel[entity] = view.get<Velocity>(entity);
    }
    
    // Calculate accelerations at t
    calculate_forces(registry); // <-- Use the pipeline
    for (auto entity : view) {
        k1_vel[entity] = view.get<Velocity>(entity).v;
        k1_accel[entity] = view.get<Acceleration>(entity).a;
    }

    // --- 2. Get k2 (state at t + dt/2, using k1) ---
    // Update all entities to intermediate state
    for (auto entity : view) {
        view.get<Position>(entity).p = original_pos[entity].p + k1_vel[entity] * (dt / 2.0);
        view.get<Velocity>(entity).v = original_vel[entity].v + k1_accel[entity] * (dt / 2.0);
    }

    // Calculate accelerations at t + dt/2
    calculate_forces(registry); // <-- Use the pipeline
    for (auto entity : view) {
        k2_vel[entity] = view.get<Velocity>(entity).v;
        k2_accel[entity] = view.get<Acceleration>(entity).a;
    }

    // --- 3. Get k3 (state at t + dt/2, using k2) ---
    // Revert and update to new intermediate state
    for (auto entity : view) {
        view.get<Position>(entity).p = original_pos[entity].p + k2_vel[entity] * (dt / 2.0);
        view.get<Velocity>(entity).v = original_vel[entity].v + k2_accel[entity] * (dt / 2.0);
    }

    // Calculate accelerations
    calculate_forces(registry); // <-- Use the pipeline
    for (auto entity : view) {
        k3_vel[entity] = view.get<Velocity>(entity).v;
        k3_accel[entity] = view.get<Acceleration>(entity).a;
    }

    // --- 4. Get k4 (state at t + dt, using k3) ---
    // Revert and update to final intermediate state
    for (auto entity : view) {
        view.get<Position>(entity).p = original_pos[entity].p + k3_vel[entity] * dt;
        view.get<Velocity>(entity).v = original_vel[entity].v + k3_accel[entity] * dt;
    }

    // Calculate accelerations
    calculate_forces(registry); // <-- Use the pipeline
    for (auto entity : view) {
        k4_vel[entity] = view.get<Velocity>(entity).v;
        k4_accel[entity] = view.get<Acceleration>(entity).a;
    }

    // --- 5. Apply final RK4 update ---
    // Revert to original state, then apply the full weighted average
    for (auto entity : view) {
        auto& pos = view.get<Position>(entity);
        auto& vel = view.get<Velocity>(entity);

        pos = original_pos[entity]; // Revert
        vel = original_vel[entity]; // Revert

        // y_new = y_old + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
        pos.p += (k1_vel[entity] + 2.0 * k2_vel[entity] + 2.0 * k3_vel[entity] + k4_vel[entity]) * (dt / 6.0);
        vel.v += (k1_accel[entity] + 2.0 * k2_accel[entity] + 2.0 * k3_accel[entity] + k4_accel[entity]) * (dt / 6.0);
    }
}


// --- Velocity Verlet Implementation ---

void IntegrateVelocityVerlet_Part1(entt::registry& registry, double dt) {
    auto view = registry.view<Position, Velocity, Acceleration>();
    for (auto entity : view) {
        auto& pos = view.get<Position>(entity);
        auto& vel = view.get<Velocity>(entity);
        // This is a(t)
        const auto& acc = view.get<Acceleration>(entity);

        // v(t + dt/2)
        vel.v += acc.a * (dt / 2.0);
        
        // x(t + dt)
        pos.p += vel.v * dt;
    }
}

void IntegrateVelocityVerlet_Part2(entt::registry& registry, double dt) {
    auto view = registry.view<Velocity, Acceleration>();
    for (auto entity : view) {
        auto& vel = view.get<Velocity>(entity);
        // This is the *new* a(t + dt) calculated by the force pipeline
        const auto& acc = view.get<Acceleration>(entity); 

        // v(t + dt)
        vel.v += acc.a * (dt / 2.0);
    }
}
