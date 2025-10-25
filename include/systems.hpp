#ifndef SYSTEMS_HPP
#define SYSTEMS_HPP

#include <entt.hpp>
#include <string>

// --- System Declarations ---
// Systems are free functions that operate on entities.

// --- Force Systems ---
// These systems *add* to the Acceleration component.

/**
 * @brief Resets all accelerations to zero.
 * MUST be run first in the force pipeline.
 */
void ClearAccelerationSystem(entt::registry& registry);

/**
 * @brief Calculates N-body gravitational forces.
 * Reads Position and Mass, and *adds* to Acceleration.
 */
void GravitySystem(entt::registry& registry);

// ** Future System Examples **
// void ThrustSystem(entt::registry& registry);
// void AtmosphericDragSystem(entt::registry& registry);


// --- Other Systems ---

/**
 * @brief Logs the current position to the entity's Trajectory component.
 */
void UpdateTrajectorySystem(entt::registry& registry);


// --- File I/O Systems ---

/**
 * @brief Loads celestial bodies from a text file.
 */
bool loadObjectsFromFile(entt::registry& registry, const std::string& filename);

/**
 * @brief Writes the simulation results to an output file.
 */
void writeOutput(entt::registry& registry, const std::string& filename);


#endif // SYSTEMS_HPP
