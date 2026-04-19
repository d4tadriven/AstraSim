#include <catch2/catch_test_macros.hpp>
#include "astrasim/physics/rigid_body.hpp"

using namespace astrasim::physics;

TEST_CASE("RigidBody Initialization", "[rigid_body]") {
    RigidBody rb(10.0, Eigen::Matrix3d::Identity());
    
    const State& state = rb.get_state();
    REQUIRE(state.position.isZero());
    REQUIRE(state.velocity.isZero());
    REQUIRE(state.angular_velocity.isZero());
    REQUIRE(state.orientation.w() == 1.0);
}

TEST_CASE("RigidBody Force Application (Linear)", "[rigid_body]") {
    RigidBody rb(1.0, Eigen::Matrix3d::Identity()); // 1 kg mass
    rb.apply_force(Eigen::Vector3d(1.0, 0.0, 0.0)); // 1 N force in X
    rb.update(1.0); // 1 second timestep
    
    const State& state = rb.get_state();
    // v = v0 + a*t = 0 + 1*1 = 1 m/s
    REQUIRE(state.velocity.x() == 1.0);
    // p = p0 + v*t = 0 + 1*1 = 1 m (Euler step)
    REQUIRE(state.position.x() == 1.0);
}

TEST_CASE("RigidBody Torque Application (Angular)", "[rigid_body]") {
    RigidBody rb(1.0, Eigen::Matrix3d::Identity()); // 1 kg, Identity inertia
    rb.apply_torque(Eigen::Vector3d(0.0, 1.0, 0.0)); // 1 Nm torque in Y
    rb.update(1.0); // 1 second timestep
    
    const State& state = rb.get_state();
    REQUIRE(state.angular_velocity.y() == 1.0);
    REQUIRE(state.angular_velocity.x() == 0.0);
}
