
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_screen_driver_include_seabot2_screen_driver_screen_node.h:

Program Listing for File screen_node.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_screen_driver_include_seabot2_screen_driver_screen_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_screen_driver/include/seabot2_screen_driver/screen_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_SCREEN_NODE_H
   #define BUILD_SCREEN_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_screen_driver/screen.h"
   #include <memory>
   #include "seabot2_msgs/msg/bme280_data.hpp"
   #include "seabot2_msgs/msg/mission_state.hpp"
   #include "seabot2_msgs/msg/power_state.hpp"
   #include "seabot2_msgs/msg/safety_status2.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class ScreenNode final: public rclcpp::Node {
   public:
     ScreenNode();
   
     ~ScreenNode() override;
   
   private:
   
     Screen screen_;
   
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 2s;   // loop dt
   
     std::array < unsigned char, 4 > ip_ = {0, 0, 0, 0};
     double pressure_ = 9999.;
     double temperature_ = 99.9;
     double hygro_ = 99;
     double voltage_ = 25.4;
     string robot_name_ = "NoName";
     string mission_name_ = "NoMission";
     unsigned int wp_id_ = 0;
     unsigned int wp_max_ = 0;
     rclcpp::Time time_next_wp_ = this->now() + rclcpp::Duration(5, 0);
     Screen::Robot_Status status_ = Screen::Robot_Status::WARNING;
   
     double depth_ = 0.0;
     double depth_no_update_ = 1.0;
   
     rclcpp::Subscription < seabot2_msgs::msg::Bme280Data > ::SharedPtr subscriber_sensor_internal_;
     rclcpp::Subscription < seabot2_msgs::msg::MissionState > ::SharedPtr subscriber_mission_;
     rclcpp::Subscription < seabot2_msgs::msg::PowerState > ::SharedPtr subscriber_power_;
     rclcpp::Subscription < seabot2_msgs::msg::SafetyStatus2 > ::SharedPtr subscriber_safety_;
   
     void timer_callback();
   
     void init_parameters();
   
     void init_topics();
   
   private:
     void get_hostname();
   
     void get_ip();
   
     void topic_internal_pressure_callback(const seabot2_msgs::msg::Bme280Data & msg);
   
     void waypoint_callback(const seabot2_msgs::msg::MissionState & msg);
   
     void power_callback(const seabot2_msgs::msg::PowerState & msg);
   
     void safety_callback(const seabot2_msgs::msg::SafetyStatus2 & msg);
   };
   
   #endif //BUILD_SCREEN_NODE_H
