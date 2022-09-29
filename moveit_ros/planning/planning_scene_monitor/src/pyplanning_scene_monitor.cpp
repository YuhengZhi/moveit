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

/* Author: Peter Mitrano */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace py = pybind11;
using namespace planning_scene_monitor;

void def_planning_scene_monitor_bindings(py::module& m)
{
     m.doc() = "The planning scene monitor listens to a topic of planning scene, "
            "and update its underlying planning scene accordingly. ";

     py::enum_<PlanningSceneMonitor::SceneUpdateType>(m, "SceneUpdateType")
            .value("UPDATE_NONE", PlanningSceneMonitor::UPDATE_NONE)
            .value("UPDATE_STATE", PlanningSceneMonitor::UPDATE_STATE)
            .value("UPDATE_TRANSFORMS", PlanningSceneMonitor::UPDATE_TRANSFORMS)
            .value("UPDATE_GEOMETRY", PlanningSceneMonitor::UPDATE_GEOMETRY)
            .value("UPDATE_SCENE", PlanningSceneMonitor::UPDATE_SCENE)
            .export_values();
     py::class_<PlanningSceneMonitor, PlanningSceneMonitorPtr>(m, "PlanningSceneMonitor")
          .def(py::init<const std::string&>(),
               py::arg("robot_description") = "robot_description")
          .def("getName", &PlanningSceneMonitor::getName)
          .def("getPlanningScene", py::overload_cast<>(
               &PlanningSceneMonitor::getPlanningScene), py::return_value_policy::reference)
          .def("startSceneMonitor", &PlanningSceneMonitor::startSceneMonitor,
               py::arg("scene_topic") = "planning_scene")
          .def("startWorldGeometryMonitor", &PlanningSceneMonitor::startWorldGeometryMonitor,
               py::arg("collision_objects_topic") = PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
               py::arg("planning_scene_world_topic") = PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
               py::arg("load_octomap_monitoring") = true)
          .def("startStateMonitor", &PlanningSceneMonitor::startStateMonitor,
               py::arg("joint_states_topic") = PlanningSceneMonitor::DEFAULT_JOINT_STATES_TOPIC,
               py::arg("attached_collision_objects_topic") = PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC)
          .def("stopSceneMonitor", &PlanningSceneMonitor::stopSceneMonitor)
          .def("stopWorldGeometryMonitor", &PlanningSceneMonitor::stopWorldGeometryMonitor)
          .def("stopStateMonitor", &PlanningSceneMonitor::stopStateMonitor)
          .def("triggerSceneUpdateEvent", &PlanningSceneMonitor::triggerSceneUpdateEvent, 
               py::arg("update_type") = PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE)
          .def("requestPlanningSceneState", &PlanningSceneMonitor::requestPlanningSceneState,
               py::arg("service_name") = PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE)
          .def("enableChangeDetection", &PlanningSceneMonitor::enableChangeDetection)
          .def("disableChangeDetection", &PlanningSceneMonitor::disableChangeDetection)
          .def("resetChangeDetection", &PlanningSceneMonitor::resetChangeDetection)
          .def("getChangedDetectionCoordinate", &PlanningSceneMonitor::getChangeDetectionCoordinate)
      //
      ;
}
