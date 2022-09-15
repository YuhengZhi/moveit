/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Peter Mitrano
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The name of Peter Mitrano may not be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yuheng Zhi, Peter Mitrano */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/collision_detection/collision_matrix.h>
// #include <moveit/collision_detection/collision_common.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

namespace py = pybind11;

using namespace collision_detection;


void def_collision_distance_field_bindings(py::module& m)
{
  m.doc() = "contains collision distance field";
  py::class_<CollisionSphere>(m, "CollisionSphere")
    .def(py::init<const Eigen::Vector3d&, double>(),
          py::arg("rel"), py::arg("radius"))
    //
    ;
  py::class_<GradientInfo>(m, "GradientInfo")
    .def(py::init<>())
    .def_readwrite("gradients", &GradientInfo::gradients)
    .def_readwrite("closest_distance", &GradientInfo::closest_distance)
    .def_readwrite("collision", &GradientInfo::collision)
    .def_readwrite("sphere_locations", &GradientInfo::sphere_locations)
    .def_readwrite("distances", &GradientInfo::distances)
    .def_readwrite("sphere_radii", &GradientInfo::sphere_radii)
    .def_readwrite("joint_name", &GradientInfo::joint_name)
    .def("clear", &GradientInfo::clear)
    //
    ;
  py::class_<GroupStateRepresentation, GroupStateRepresentationPtr>(m, "GroupStateRepresentation")
    .def(py::init<>())
    // .def_readwrite("link_body_decompositions_", &GroupStateRepresentation::link_body_decompositions_)
    // .def_readwrite("attached_body_decompositions_", &GroupStateRepresentation::attached_body_decompositions_)
    // .def_readwrite("link_distance_fields_", &GroupStateRepresentation::link_distance_fields_)
    .def_readwrite("gradients", &GroupStateRepresentation::gradients_)
    ;
  // py::class_<>(m, "GroupStateRepresentationPtr")
  //   .def(py::init<>())
  //   ;
  // py::class_<GroupStateRepresentation>
  py::class_<CollisionEnvDistanceField>(m, "CollisionEnvDistanceField")
    .def(py::init<const moveit::core::RobotModelConstPtr&, const WorldPtr&,
    const std::map<std::string, std::vector<CollisionSphere>>&, double, double,
    double, const Eigen::Vector3d&, bool, double,
    double, double, double, double>(),
        py::arg("robot_model"), py::arg("world"), 
        py::arg("link_body_decompositions") = std::map<std::string, std::vector<CollisionSphere>>(),
        py::arg("size_x") = DEFAULT_SIZE_X, py::arg("size_y") = DEFAULT_SIZE_Y, py::arg("size_z") = DEFAULT_SIZE_Z, 
        py::arg("origin") = Eigen::Vector3d(0, 0, 0), 
        py::arg("use_signed_distance_field") = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
        py::arg("resolution") = DEFAULT_RESOLUTION, 
        py::arg("collision_tolerance") = DEFAULT_COLLISION_TOLERANCE, 
        py::arg("max_propogation_distance") = DEFAULT_MAX_PROPOGATION_DISTANCE, 
        py::arg("padding") = 0.0,
        py::arg("scale") = 1.0)
    .def("checkCollision",
      py::overload_cast<const CollisionRequest&, CollisionResult&, const moveit::core::RobotState&,
        const AllowedCollisionMatrix&, GroupStateRepresentationPtr&>(
      &CollisionEnvDistanceField::checkCollision, py::const_), 
      py::arg("req"), py::arg("res"), py::arg("state"),
      py::arg("acm"), py::arg("gsr"))
    .def("checkCollision",
      py::overload_cast<const CollisionRequest&, CollisionResult&, const moveit::core::RobotState&,
        const AllowedCollisionMatrix&>(
      &CollisionEnvDistanceField::checkCollision, py::const_), 
      py::arg("req"), py::arg("res"), py::arg("state"),
      py::arg("acm"))
    .def("getCollisionGradients",
      py::overload_cast<const CollisionRequest&, CollisionResult&, const moveit::core::RobotState&,
        const AllowedCollisionMatrix*, GroupStateRepresentationPtr&>(
      &CollisionEnvDistanceField::getCollisionGradients, py::const_), 
      py::arg("req"), py::arg("res"), py::arg("state"),
      py::arg("acm"), py::arg("gsr"))
  //
  ;
}
