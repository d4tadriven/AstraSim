#include "astrasim/physics/aerodynamics.hpp"

namespace astrasim {
namespace physics {

Eigen::Vector3d Aerodynamics::calculate_drag(const Eigen::Vector3d& velocity, double density, double area, double cd) {
    double v_sq = velocity.squaredNorm();
    if (v_sq < 1e-6) return Eigen::Vector3d::Zero();
    
    double drag_mag = 0.5 * density * v_sq * cd * area;
    Eigen::Vector3d drag_dir = -velocity.normalized();
    
    return drag_dir * drag_mag;
}

Eigen::Vector3d Aerodynamics::calculate_lift(const Eigen::Vector3d& velocity, const Eigen::Vector3d& up_vector, double density, double area, double cl) {
    double v_sq = velocity.squaredNorm();
    if (v_sq < 1e-6) return Eigen::Vector3d::Zero();
    
    double lift_mag = 0.5 * density * v_sq * cl * area;
    
    Eigen::Vector3d v_dir = velocity.normalized();
    // Lift is orthogonal to velocity in the plane defined by velocity and up_vector
    Eigen::Vector3d lift_dir = (up_vector - up_vector.dot(v_dir) * v_dir);
    
    if (lift_dir.squaredNorm() < 1e-6) {
        return Eigen::Vector3d::Zero();
    }
    
    return lift_dir.normalized() * lift_mag;
}

} // namespace physics
} // namespace astrasim
