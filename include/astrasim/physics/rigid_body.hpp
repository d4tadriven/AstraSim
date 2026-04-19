#pragma once
#include <Eigen/Dense>

namespace astrasim {
namespace physics {

struct State {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;

    State() : position(Eigen::Vector3d::Zero()),
              velocity(Eigen::Vector3d::Zero()),
              orientation(Eigen::Quaterniond::Identity()),
              angular_velocity(Eigen::Vector3d::Zero()) {}
};

class RigidBody {
public:
    RigidBody(double mass, const Eigen::Matrix3d& inertia_tensor);

    void apply_force(const Eigen::Vector3d& force, const Eigen::Vector3d& point = Eigen::Vector3d::Zero());
    void apply_torque(const Eigen::Vector3d& torque);
    
    // Advance simulation by dt (using semi-implicit Euler)
    void update(double dt);

    const State& get_state() const { return state_; }
    
    void set_mass(double mass) { mass_ = mass; }
    double get_mass() const { return mass_; }

private:
    double mass_;
    Eigen::Matrix3d inertia_tensor_;
    Eigen::Matrix3d inertia_tensor_inv_;
    State state_;

    Eigen::Vector3d force_accumulator_;
    Eigen::Vector3d torque_accumulator_;
};

} // namespace physics
} // namespace astrasim
