#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "astrasim/physics/rigid_body.hpp"
#include "astrasim/physics/environment.hpp"
#include "astrasim/physics/aerodynamics.hpp"

namespace py = pybind11;

PYBIND11_MODULE(core, m) {
    m.doc() = "AstraSim: Real-time interactive aerospace simulation platform";

    // Bind State
    py::class_<astrasim::physics::State>(m, "State")
        .def(py::init<>())
        .def_readwrite("position", &astrasim::physics::State::position)
        .def_readwrite("velocity", &astrasim::physics::State::velocity)
        .def_readwrite("angular_velocity", &astrasim::physics::State::angular_velocity);

    // Bind RigidBody
    py::class_<astrasim::physics::RigidBody>(m, "RigidBody")
        .def(py::init<double, const Eigen::Matrix3d&>(), py::arg("mass"), py::arg("inertia_tensor"))
        .def("apply_force", &astrasim::physics::RigidBody::apply_force, 
             py::arg("force"), py::arg("point") = Eigen::Vector3d::Zero(),
             "Apply force to the rigid body")
        .def("apply_torque", &astrasim::physics::RigidBody::apply_torque, 
             py::arg("torque"),
             "Apply torque to the rigid body")
        .def("update", &astrasim::physics::RigidBody::update, 
             py::arg("dt"),
             "Advance the simulation by dt seconds")
        .def_property("mass", &astrasim::physics::RigidBody::get_mass, &astrasim::physics::RigidBody::set_mass,
             "Get or set the mass of the rigid body (for fuel depletion)")
        .def_property_readonly("state", &astrasim::physics::RigidBody::get_state,
             "Get the current 6-DOF state");

    // Bind Environment
    py::class_<astrasim::physics::Environment>(m, "Environment")
        .def_static("get_gravity", &astrasim::physics::Environment::get_gravity, py::arg("altitude"), "Get gravity vector at altitude")
        .def_static("get_air_density", &astrasim::physics::Environment::get_air_density, py::arg("altitude"), "Get air density at altitude");

    // Bind Aerodynamics
    py::class_<astrasim::physics::Aerodynamics>(m, "Aerodynamics")
        .def_static("calculate_drag", &astrasim::physics::Aerodynamics::calculate_drag, 
                    py::arg("velocity"), py::arg("density"), py::arg("area"), py::arg("cd"))
        .def_static("calculate_lift", &astrasim::physics::Aerodynamics::calculate_lift, 
                    py::arg("velocity"), py::arg("up_vector"), py::arg("density"), py::arg("area"), py::arg("cl"));
}
