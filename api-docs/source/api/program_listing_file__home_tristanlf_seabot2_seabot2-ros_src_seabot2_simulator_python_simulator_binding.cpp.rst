
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_simulator_python_simulator_binding.cpp:

Program Listing for File simulator_binding.cpp
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_simulator_python_simulator_binding.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_simulator/python/simulator_binding.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include <pybind11/pybind11.h>
   #include "seabot2_simulator/simulator.h"
   #include <pybind11/stl.h>
   #include <pybind11/complex.h>
   #include <pybind11/functional.h>
   #include <pybind11/chrono.h>
   
   #define STRINGIFY(x) #x
   #define MACRO_STRINGIFY(x) STRINGIFY(x)
   
   namespace py = pybind11;
   
   PYBIND11_MODULE(seabot2py, m) {
       m.doc() = "Python binding of seabot2_simulator";
   
       py::class_<Simulator>(m, "Simulator", py::dynamic_attr())
               .def(py::init<>())
               .def("run_simulation", &Simulator::run_simulation)
               .def_readwrite("memory_time", &Simulator::memory_time)
               .def_readwrite("memory_piston_position", &Simulator::memory_piston_position)
               .def_readwrite("memory_piston_velocity", &Simulator::memory_piston_velocity)
               .def_readwrite("memory_velocity", &Simulator::memory_velocity)
               .def_readwrite("memory_depth", &Simulator::memory_depth)
               ;
   
   #ifdef VERSION_INFO
       m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
   #else
       m.attr("__version__") = "dev";
   #endif
   }
