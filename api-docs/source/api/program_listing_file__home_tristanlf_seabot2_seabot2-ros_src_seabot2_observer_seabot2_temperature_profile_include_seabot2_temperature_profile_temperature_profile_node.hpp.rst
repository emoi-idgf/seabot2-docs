
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_temperature_profile_include_seabot2_temperature_profile_temperature_profile_node.hpp:

Program Listing for File temperature_profile_node.hpp
=====================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_temperature_profile_include_seabot2_temperature_profile_temperature_profile_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_temperature_profile/include/seabot2_temperature_profile/temperature_profile_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_TEMPERATURE_PROFILE_NODE_HPP
   #define BUILD_TEMPERATURE_PROFILE_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_temperature_profile/temperature_profile.h"
   
   #include "seabot2_msgs/msg/depth_pose.hpp"
   #include "seabot2_msgs/msg/temperature_sensor_data.hpp"
   #include "seabot2_msgs/msg/temperature_profile.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class TemperatureProfile;
   
   class TemperatureProfileNode final : public rclcpp::Node {
   public:
       TemperatureProfileNode();
   
   private:
   
       rclcpp::TimerBase::SharedPtr timer_;
       std::chrono::milliseconds loop_dt_ = 200ms; 
   
       TemperatureProfile t_;
   
       double depth_ = 0.0;
   
       rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
       rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_;
   
       rclcpp::Publisher<seabot2_msgs::msg::TemperatureProfile>::SharedPtr publisher_temperature_profile_;
   
   
       void init_parameters();
   
       void init_interfaces();
   
       void depth_callback(const seabot2_msgs::msg::DepthPose &msg);
   
       void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData &msg);
   
   };
   #endif //BUILD_TEMPERATURE_PROFILE_NODE_HPP
