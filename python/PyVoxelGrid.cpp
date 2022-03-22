#include <iostream>

#include "uasrisk/environment/VoxelGrid.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;
template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void hello()
{
    std::cout << "Hello, World!" << std::endl;
}

PYBIND11_MODULE(pyuasrisk, topModule)
{
    topModule.doc() = "Risk estimation for uncrewed aerial systems";
    topModule.attr("__version__") = "0.0.1"; //TODO grab from CMake defines

    topModule.def("hello", &hello, "Prints \"Hello, World!\"");

    // auto voxelGridModule = topModule.def_submodule("environment");
    py::class_<ur::VoxelGrid>(topModule, "VoxelGrid")
        .def(py::init<const std::array<ur::FPScalar, 6>, ur::FPScalar, ur::FPScalar, const char*>())
        .def("add", overload_cast_<const std::string&, const ur::FPScalar>()(&ur::VoxelGrid::add),
             "Add a layer with a constant value")
        .def("add", overload_cast_<const std::string&, const ur::Matrix&>()(&ur::VoxelGrid::add),
             "Add a layer with a prefilled grid")
        .def("at", overload_cast_<const std::string&, const ur::Index&>()(&ur::VoxelGrid::at),
             py::return_value_policy::reference_internal, "Return the value of the layer at a given index")
        .def("at", overload_cast_<const std::string&, const ur::Index&>()(&ur::VoxelGrid::at, py::const_),
             "Return the value of the layer at a given index")
        .def("atPosition", overload_cast_<const std::string&, const ur::Position&>()(&ur::VoxelGrid::atPosition),
             py::return_value_policy::reference_internal,
             "Return the value of the layer at the given world coordinates")
        .def("atPosition",
             overload_cast_<const std::string&, const ur::Position&>()(&ur::VoxelGrid::atPosition, py::const_),
             "Return the value of the layer at the given world coordinates")
        .def("get", overload_cast_<const std::string&>()(&ur::VoxelGrid::get),
             py::return_value_policy::reference_internal, "Return the entire tensor for a layer")
        .def("get", overload_cast_<const std::string&>()(&ur::VoxelGrid::get, py::const_),
             "Return the entire tensor for a layer")
        .def("world2Local", overload_cast_<const ur::Position&>()(&ur::VoxelGrid::world2Local, py::const_),
             "Reproject world (EPSG:4326) coordinates to local indices")
        .def("world2Local",
             overload_cast_<ur::FPScalar, ur::FPScalar, ur::FPScalar>()(&ur::VoxelGrid::world2Local, py::const_),
             "Reproject world (EPSG:4326) coordinates to local indices")
        .def("isInBounds", &ur::VoxelGrid::isInBounds, "Test if the given local indices is within bounds")
        .def("writeToNetCDF", &ur::VoxelGrid::writeToNetCDF, "Write the risk layers to netCDF format for export.")
        .def_property_readonly("size", &ur::VoxelGrid::getSize, "Return the lengths of the grid")
        .def("local2World", overload_cast_<const ur::Index&>()(&ur::VoxelGrid::local2World, py::const_),
             "Reproject local indices to world coordinates")
        .def("local2World", overload_cast_<int, int, int>()(&ur::VoxelGrid::local2World, py::const_),
             "Reproject local indices to world coordinates");
}
