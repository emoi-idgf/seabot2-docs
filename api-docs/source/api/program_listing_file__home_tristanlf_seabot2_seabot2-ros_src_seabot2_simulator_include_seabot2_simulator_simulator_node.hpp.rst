
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_simulator_include_seabot2_simulator_simulator_node.hpp:

Program Listing for File simulator_node.hpp
===========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_simulator_include_seabot2_simulator_simulator_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_simulator/include/seabot2_simulator/simulator_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_SIMULATOR_NODE_HPP
   #define BUILD_SIMULATOR_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include "simulator.h"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class SimulatorNode final : public rclcpp::Node {
   public:
       SimulatorNode();
   
   private:
       Simulator s_;
   
   
       void init_parameters();
   
   };
   #endif //BUILD_SIMULATOR_NODE_HPP
