#include "systems.hpp"
#include "components.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>

// --- Force Systems ---

void ClearAccelerationSystem(entt::registry& registry) {
    auto view = registry.view<Acceleration>();
    for (auto entity : view) {
        auto& acc = view.get<Acceleration>(entity);
        acc.a = {0.0, 0.0, 0.0};
    }
}

void GravitySystem(entt::registry& registry) {
    // Get a view of all entities that have Position, Mass, and Acceleration
    auto view = registry.view<Position, Mass, Acceleration>();

    // Iterate over all unique pairs of entities
    for (auto entity_i : view) {
        // Get components for object 'i'
        auto& pos_i = view.get<Position>(entity_i);
        auto& mass_i = view.get<Mass>(entity_i);
        auto& acc_i = view.get<Acceleration>(entity_i);

        for (auto entity_j : view) {
            if (entity_i == entity_j) continue; // Don't interact with self

            // Get components for object 'j'
            const auto& pos_j = view.get<Position>(entity_j);
            const auto& mass_j = view.get<Mass>(entity_j);

            // Calculate vector from i to j
            Vec3 r_vec = pos_j.p - pos_i.p;
            
            // Calculate squared distance
            double r_sq = r_vec.x * r_vec.x + 
                          r_vec.y * r_vec.y + 
                          r_vec.z * r_vec.z;

            // Add softening factor to avoid singularity
            r_sq += SOFTENING_FACTOR_SQUARED;

            // Calculate magnitude of the force: F = G * (m1*m2) / r^2
            double r = std::sqrt(r_sq);
            double force_mag = G * mass_i.m * mass_j.m / r_sq;
            
            // Calculate force vector: F_vec = F_mag * (r_vec / r)
            Vec3 force_vec = (r_vec / r) * force_mag;

            // Calculate acceleration for 'i': a = F / m
            Vec3 acc_vec = force_vec / mass_i.m;
            
            // *Accumulate* (add to) acceleration on 'i'
            acc_i.a += acc_vec;
        }
    }
}


// --- Other Systems ---

void UpdateTrajectorySystem(entt::registry& registry) {
    auto view = registry.view<Position, Trajectory>();
    for (auto entity : view) {
        const auto& pos = view.get<Position>(entity);
        auto& traj = view.get<Trajectory>(entity);
        traj.path.push_back(pos.p);
    }
}


// --- File I/O Systems ---

bool loadObjectsFromFile(entt::registry& registry, const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open input file: " << filename << std::endl;
        return false;
    }

    std::cout << "Loading objects from " << filename << "..." << std::endl;
    std::string line;
    int count = 0;
    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string name;
        double x, y, z, vx, vy, vz, mass;

        if (ss >> name >> x >> y >> z >> vx >> vy >> vz >> mass) {
            auto entity = registry.create();
            registry.emplace<Name>(entity, name);
            registry.emplace<Position>(entity, Vec3{x, y, z});
            registry.emplace<Velocity>(entity, Vec3{vx, vy, vz});
            registry.emplace<Mass>(entity, mass);
            registry.emplace<Acceleration>(entity, Vec3{0, 0, 0});
            registry.emplace<Trajectory>(entity); // Add empty trajectory log
            
            std::cout << "  Loaded: " << name << " (Mass: " << mass << ")" << std::endl;
            count++;
        } else {
            std::cerr << "Warning: Skipping malformed line: " << line << std::endl;
        }
    }
    std::cout << "Loaded " << count << " objects." << std::endl;
    return true;
}

void writeOutput(entt::registry& registry, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open output file: " << filename << std::endl;
        return;
    }

    std::cout << "Writing simulation output to " << filename << "..." << std::endl;
    // Use high precision for output data
    file << std::fixed << std::setprecision(8);

    auto view = registry.view<Name, Mass, Trajectory>();
    for (auto entity : view) {
        const auto& name = view.get<Name>(entity);
        const auto& mass = view.get<Mass>(entity);
        const auto& traj = view.get<Trajectory>(entity);

        file << "ObjectName: " << name.name << "\n";
        file << "Mass: " << mass.m << "\n";
        file << "Path:\n";
        
        for (const auto& pos : traj.path) {
            file << pos.x << ", " << pos.y << ", " << pos.z << "\n";
        }
        
        file << "EndObject\n\n";
    }
    std::cout << "Output file written." << std::endl;
}
