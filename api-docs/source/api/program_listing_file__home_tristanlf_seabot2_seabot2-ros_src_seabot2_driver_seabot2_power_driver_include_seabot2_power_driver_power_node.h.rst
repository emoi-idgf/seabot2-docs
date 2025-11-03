
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_power_driver_include_seabot2_power_driver_power_node.h:

Program Listing for File power_node.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_power_driver_include_seabot2_power_driver_power_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_power_driver/include/seabot2_power_driver/power_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_POWER_NODE_H
   #define BUILD_POWER_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <memory>
   #include "seabot2_power_driver/power.h"
   #include "seabot2_msgs/msg/power_state.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class PowerNode final : public rclcpp::Node {
   public:
       PowerNode();
   
   private:
   
       rclcpp::TimerBase::SharedPtr timer_;
       std::chrono::milliseconds loop_dt_ = 500ms; 
   
       Power power_;
   
       rclcpp::Publisher<seabot2_msgs::msg::PowerState>::SharedPtr publisher_power_state_;
   
       rclcpp::Time time_turn_off_light_ = this->now();
       bool light_is_on_ = false;
   
       void timer_callback();
   
       void init_parameters();
   
       void init_interfaces();
   };
   
   #endif //BUILD_POWER_NODE_H
