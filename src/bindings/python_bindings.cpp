#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "astrasim/physics/rigid_body.hpp"
#include "astrasim/physics/environment.hpp"
#include "astrasim/physics/aerodynamics.hpp"

namespace py = pybind11;

PYBIND11_MODULE(core, m) {
    m.doc() = "AstraSim: Real-time interactive aerospace simulation platform";

    // Bind Quaternion for orientation
    py::class_<Eigen::Quaterniond>(m, "Quaternion")
        .def(py::init<double, double, double, double>(), py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_property("w", [](const Eigen::Quaterniond& q) { return q.w(); }, [](Eigen::Quaterniond& q, double v) { q.w() = v; })
        .def_property("x", [](const Eigen::Quaterniond& q) { return q.x(); }, [](Eigen::Quaterniond& q, double v) { q.x() = v; })
        .def_property("y", [](const Eigen::Quaterniond& q) { return q.y(); }, [](Eigen::Quaterniond& q, double v) { q.y() = v; })
        .def_property("z", [](const Eigen::Quaterniond& q) { return q.z(); }, [](Eigen::Quaterniond& q, double v) { q.z() = v; });

    // Bind State
    py::class_<astrasim::physics::State>(m, "State")
        .def(py::init<>())
        .def_readwrite("position", &astrasim::physics::State::position)
        .def_readwrite("velocity", &astrasim::physics::State::velocity)
        .def_readwrite("orientation", &astrasim::physics::State::orientation)
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
        .def_property("state", &astrasim::physics::RigidBody::get_mutable_state, [](astrasim::physics::RigidBody& self, const astrasim::physics::State& s) { self.get_mutable_state() = s; },
             py::return_value_policy::reference_internal,
             "Get or set the current 6-DOF state");

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
