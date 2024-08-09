// This is the pybind11 wrapper for the ManipServer class. It is used to expose the class to Python.

// clang-format off
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
// #include <pybind11/eigen/tensor.h>
#include "table_top_manip/manip_server.h"

namespace py = pybind11;
using namespace RUT;
PYBIND11_MODULE(manip_server_pybind, m)
{
    py::class_<ManipServer>(m, "ManipServer")
        .def(py::init<>())
        .def("initialize", &ManipServer::initialize)
        .def("join_threads", &ManipServer::join_threads)
        .def("is_running", &ManipServer::is_running)
        .def("is_ready", &ManipServer::is_ready)
        .def("get_camera_rgb", &ManipServer::get_camera_rgb)
        .def("get_wrench", &ManipServer::get_wrench)
        .def("get_pose", &ManipServer::get_pose)
        .def("get_camera_rgb_timestamps_ms", &ManipServer::get_camera_rgb_timestamps_ms)
        .def("get_wrench_timestamps_ms", &ManipServer::get_wrench_timestamps_ms)
        .def("get_pose_timestamps_ms", &ManipServer::get_pose_timestamps_ms)
        .def("get_timestamp_now_ms", &ManipServer::get_timestamp_now_ms)
        .def("set_high_level_maintain_position", &ManipServer::set_high_level_maintain_position)
        .def("set_high_level_free_jogging", &ManipServer::set_high_level_free_jogging)
        .def("set_target_pose", &ManipServer::set_target_pose)
        .def("set_force_controlled_axis", &ManipServer::set_force_controlled_axis)
        .def("set_stiffness_matrix", &ManipServer::set_stiffness_matrix)
        .def("schedule_waypoints", &ManipServer::schedule_waypoints)
        .def("schedule_stiffness", &ManipServer::schedule_stiffness);
}
// clang-format on