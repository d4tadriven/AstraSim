#include "astrasim/physics/environment.hpp"
#include <cmath>

namespace astrasim {
namespace physics {

Eigen::Vector3d Environment::get_gravity(double altitude) {
    if (altitude < 0) altitude = 0;
    double r = EARTH_RADIUS + altitude;
    double g = G0 * std::pow(EARTH_RADIUS / r, 2);
    // Gravity points down in Z direction
    return Eigen::Vector3d(0, 0, -g);
}

double Environment::get_air_density(double altitude) {
    if (altitude < 0) altitude = 0;
    
    // Simplified barometric formula
    const double rho0 = 1.225; // Sea level density kg/m^3
    const double H = 8500.0; // Scale height meters
    
    return rho0 * std::exp(-altitude / H);
}

} // namespace physics
} // namespace astrasim
