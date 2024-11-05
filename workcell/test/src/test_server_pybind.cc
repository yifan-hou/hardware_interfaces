// This is the pybind11 wrapper for the TestServer class. It is used to expose the class to Python.

// clang-format off
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "test/test_server.h"

namespace py = pybind11;
using namespace RUT;
PYBIND11_MODULE(test_server_pybind, m)
{
    py::class_<TestServer>(m, "TestServer")
        .def(py::init<>())
        .def("get_test", &TestServer::get_test);
}
// clang-format on