#pragma once
#include <Eigen/Dense>

namespace astrasim {
namespace physics {

class Environment {
public:
    // Earth radius in meters
    static constexpr double EARTH_RADIUS = 6371000.0;
    // Base gravity at sea level (m/s^2)
    static constexpr double G0 = 9.80665;

    // Get gravity vector at a given altitude (z represents altitude up)
    static Eigen::Vector3d get_gravity(double altitude);

    // Get air density at a given altitude (kg/m^3) using US Standard Atmosphere approximation
    static double get_air_density(double altitude);
};

} // namespace physics
} // namespace astrasim
