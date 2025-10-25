#ifndef COMPONENTS_HPP
#define COMPONENTS_HPP

#include <vector>
#include <string>

// --- Constants ---
// Gravitational Constant (m^3 kg^-1 s^-2)
constexpr double G = 6.67430e-11;

// A softening factor to prevent singularities (division by zero)
constexpr double SOFTENING_FACTOR_SQUARED = 1e-9; // (meters^2)


// --- Helper: 3D Vector ---
struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }
    Vec3& operator*=(double scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }
    Vec3& operator/=(double scalar) {
        x /= scalar; y /= scalar; z /= scalar;
        return *this;
    }
};

// Non-member operators
inline Vec3 operator+(Vec3 lhs, const Vec3& rhs) { return lhs += rhs; }
inline Vec3 operator-(Vec3 lhs, const Vec3& rhs) { return lhs -= rhs; }
inline Vec3 operator*(Vec3 lhs, double scalar) { return lhs *= scalar; }
inline Vec3 operator*(double scalar, Vec3 rhs) { return rhs *= scalar; }
inline Vec3 operator/(Vec3 lhs, double scalar) { return lhs /= scalar; }


// --- Components ---
// Components are simple data-only structs.

struct Position { Vec3 p; };
struct Velocity { Vec3 v; };
struct Acceleration { Vec3 a; };
struct Mass { double m; };
struct Name { std::string name; };

// This component will store the object's path for output.
struct Trajectory {
    std::vector<Vec3> path;
};

// ** Future Component Examples **
// struct Thrust {
//     Vec3 direction; // Normalized
//     double magnitude_newtons;
// };
// struct Controllable {}; // Tag for player-controlled object
// struct HasAtmosphere {
//     double scale_height;
//     double surface_density;
// };


// --- Global Simulation State ---
// Stored in the registry's "context" via registry.ctx().
// This allows any system to access global state without
// passing it around.
struct SimState {
    double current_time = 0.0;
    const double physics_dt; // The fixed physics time step
    double time_scale = 1.0; // The wall-clock multiplier
};


#endif // COMPONENTS_HPP
