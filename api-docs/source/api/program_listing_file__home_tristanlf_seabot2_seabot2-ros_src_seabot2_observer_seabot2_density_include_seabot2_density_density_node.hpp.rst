
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_density_include_seabot2_density_density_node.hpp:

Program Listing for File density_node.hpp
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_density_include_seabot2_density_density_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_density/include/seabot2_density/density_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_DENSITY_NODE_HPP
   #define BUILD_DENSITY_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_msgs/msg/density.hpp"
   #include "seabot2_msgs/msg/depth_pose.hpp"
   #include "seabot2_msgs/msg/temperature_sensor_data.hpp"
   #include "TeosCpp/TeosSea.h"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class DensityNode final : public rclcpp::Node {
   public:
     DensityNode();
   
   private:
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 1s;   
   
     double sea_pressure_ = 0.;
     double temperature_ = 12.0;
     double salinity_ = 0.;
     double water_density_ = 1000.0;
     double water_sound_speed_ = 1500.0;
   
     TeosSea ts;
   
     rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
     rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr
       subscriber_temperature_data_;
   
     rclcpp::Publisher<seabot2_msgs::msg::Density>::SharedPtr publisher_density_;
   
   
     void timer_callback();
   
     void init_parameters();
   
     void init_interfaces();
   
     void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData & msg);
   
     void pressure_callback(const seabot2_msgs::msg::DepthPose & msg);
   
   private:
   
   };
   #endif //BUILD_DENSITY_NODE_HPP
