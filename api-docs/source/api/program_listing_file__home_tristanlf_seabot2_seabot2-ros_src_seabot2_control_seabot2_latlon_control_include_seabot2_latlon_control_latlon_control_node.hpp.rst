
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_latlon_control_include_seabot2_latlon_control_latlon_control_node.hpp:

Program Listing for File latlon_control_node.hpp
================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_latlon_control_include_seabot2_latlon_control_latlon_control_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_latlon_control/include/seabot2_latlon_control/latlon_control_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_LATLON_CONTROL_NODE_HPP
   #define BUILD_LATLON_CONTROL_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   
   #include "seabot2_latlon_control/latlon_control.h"
   
   #include "seabot2_msgs/msg/gps_fix.hpp"
   #include "seabot2_msgs/msg/raw_data.hpp"
   #include "seabot2_msgs/msg/safety_status2.hpp"
   
   #include "geometry_msgs/msg/twist.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class LatLonControlNode : public rclcpp::Node {
   public:
     LatLonControlNode();
     ~LatLonControlNode();
   
   private:
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 200ms;   
   
     LatLonControl llc_;
   
     rclcpp::Subscription<seabot2_msgs::msg::GpsFix>::SharedPtr subscriber_gps_fix_data_;
     rclcpp::Subscription<seabot2_msgs::msg::RawData>::SharedPtr subscriber_imu_data_;
     rclcpp::Subscription<seabot2_msgs::msg::SafetyStatus2>::SharedPtr subscriber_safety_data_;
   
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_thrusters_cmd_;
     void init_parameters();
   
     void init_interfaces();
   
     void timer_callback();
   
     void imu_data_callback(const seabot2_msgs::msg::RawData & msg);
   
     void gps_fix_callback(const seabot2_msgs::msg::GpsFix & msg);
   
     void safety_callback(const seabot2_msgs::msg::SafetyStatus2 & msg);
   
   };
   #endif //BUILD_LATLON_CONTROL_NODE_HPP
