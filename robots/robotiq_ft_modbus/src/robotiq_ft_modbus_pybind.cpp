// This is the pybind11 wrapper for the RobotiqFTModbus class. It is used to expose the class to Python.

// clang-format off
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "hardware_interfaces/ft_interfaces.h"
#include "robotiq_ft_modbus/robotiq_ft_modbus.h"

namespace py = pybind11;
using namespace RUT;
PYBIND11_MODULE(robotiq_ft_modbus_pybind, m)
{
    py::class_<RobotiqFTModbus>(m, "RobotiqFTModbus")
        .def(py::init<>())
        .def("init", &RobotiqFTModbus::init)
        .def("getWrenchSensor", &RobotiqFTModbus::getWrenchSensor)
        .def("getWrenchTool", &RobotiqFTModbus::getWrenchTool)
        .def("getWrenchNetTool", &RobotiqFTModbus::getWrenchNetTool)
        .def("is_data_ready", &RobotiqFTModbus::is_data_ready);
    py::class_<RobotiqFTModbus::RobotiqFTModbusConfig>(m, "RobotiqFTModbusConfig")
        .def(py::init<>())
        .def_readwrite("sensor_name", &RobotiqFTModbus::RobotiqFTModbusConfig::sensor_name)
        .def_readwrite("fullpath", &RobotiqFTModbus::RobotiqFTModbusConfig::fullpath)
        .def_readwrite("print_flag", &RobotiqFTModbus::RobotiqFTModbusConfig::print_flag)
        .def_readwrite("publish_rate", &RobotiqFTModbus::RobotiqFTModbusConfig::publish_rate)
        .def_readwrite("noise_level", &RobotiqFTModbus::RobotiqFTModbusConfig::noise_level)
        .def_readwrite("stall_threshold", &RobotiqFTModbus::RobotiqFTModbusConfig::stall_threshold)
        .def_readwrite("Foffset", &RobotiqFTModbus::RobotiqFTModbusConfig::Foffset)
        .def_readwrite("Toffset", &RobotiqFTModbus::RobotiqFTModbusConfig::Toffset)
        .def_readwrite("Gravity", &RobotiqFTModbus::RobotiqFTModbusConfig::Gravity)
        .def_readwrite("Pcom", &RobotiqFTModbus::RobotiqFTModbusConfig::Pcom)
        .def_readwrite("WrenchSafety", &RobotiqFTModbus::RobotiqFTModbusConfig::WrenchSafety)
        .def_readwrite("PoseSensorTool", &RobotiqFTModbus::RobotiqFTModbusConfig::PoseSensorTool)
        .def("deserialize", &RobotiqFTModbus::RobotiqFTModbusConfig::deserialize);
}
// clang-format on