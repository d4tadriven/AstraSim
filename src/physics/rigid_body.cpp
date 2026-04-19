#include "astrasim/physics/rigid_body.hpp"

namespace astrasim {
namespace physics {

RigidBody::RigidBody(double mass, const Eigen::Matrix3d& inertia_tensor)
    : mass_(mass),
      inertia_tensor_(inertia_tensor),
      inertia_tensor_inv_(inertia_tensor.inverse()),
      force_accumulator_(Eigen::Vector3d::Zero()),
      torque_accumulator_(Eigen::Vector3d::Zero()) {}

void RigidBody::apply_force(const Eigen::Vector3d& force, const Eigen::Vector3d& point) {
    force_accumulator_ += force;
    if (!point.isZero()) {
        torque_accumulator_ += point.cross(force);
    }
}

void RigidBody::apply_torque(const Eigen::Vector3d& torque) {
    torque_accumulator_ += torque;
}

void RigidBody::update(double dt) {
    // Semi-implicit Euler integration
    Eigen::Vector3d acceleration = force_accumulator_ / mass_;
    
    // Euler equations of motion for rigid body rotation
    Eigen::Vector3d angular_acceleration = inertia_tensor_inv_ * 
        (torque_accumulator_ - state_.angular_velocity.cross(inertia_tensor_ * state_.angular_velocity));

    // Update velocities
    state_.velocity += acceleration * dt;
    state_.angular_velocity += angular_acceleration * dt;
    
    // Update position
    state_.position += state_.velocity * dt;

    // Update orientation (quaternion derivative)
    Eigen::Quaterniond q_dot;
    q_dot.w() = 0;
    q_dot.vec() = state_.angular_velocity;
    
    Eigen::Quaterniond delta_q;
    delta_q.w() = q_dot.w() * state_.orientation.w() - q_dot.vec().dot(state_.orientation.vec());
    delta_q.vec() = q_dot.w() * state_.orientation.vec() + state_.orientation.w() * q_dot.vec() + q_dot.vec().cross(state_.orientation.vec());

    state_.orientation.w() += 0.5 * delta_q.w() * dt;
    state_.orientation.vec() += 0.5 * delta_q.vec() * dt;
    state_.orientation.normalize(); // Prevent drift

    // Reset accumulators
    force_accumulator_.setZero();
    torque_accumulator_.setZero();
}

} // namespace physics
} // namespace astrasim
