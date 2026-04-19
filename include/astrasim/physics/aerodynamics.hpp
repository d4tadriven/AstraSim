#pragma once
#include <Eigen/Dense>

namespace astrasim {
namespace physics {

class Aerodynamics {
public:
    // Calculate drag force
    static Eigen::Vector3d calculate_drag(const Eigen::Vector3d& velocity, double density, double area, double cd);
    
    // Calculate lift force
    static Eigen::Vector3d calculate_lift(const Eigen::Vector3d& velocity, const Eigen::Vector3d& up_vector, double density, double area, double cl);
};

} // namespace physics
} // namespace astrasim
