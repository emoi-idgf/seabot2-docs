
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_mission_include_seabot2_mission_mission_node.hpp:

Program Listing for File mission_node.hpp
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_mission_include_seabot2_mission_mission_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_mission/include/seabot2_mission/mission_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef SEABOT2_MISSION__MISSION_NODE_HPP_
   #define SEABOT2_MISSION__MISSION_NODE_HPP_
   
   #include <string>
   #include <memory>
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_mission/mission.hpp"
   
   #include "seabot2_msgs/msg/depth_control_set_point.hpp"
   #include "seabot2_msgs/msg/depth_pose.hpp"
   #include "seabot2_msgs/msg/temperature_sensor_data.hpp"
   #include "seabot2_msgs/msg/temperature_keeping_debug.hpp"
   
   #include "seabot2_srvs/srv/light.hpp"
   
   #include "std_srvs/srv/set_bool.hpp"
   #include "std_srvs/srv/trigger.hpp"
   
   
   class MissionNode final : public rclcpp::Node {
   public:
     MissionNode();
   
   private:
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = std::chrono::milliseconds(1000);
   
     rclcpp::CallbackGroup::SharedPtr callback_group_;
   
     Mission mission_;
     std::string mission_file_name_ = "mission.xml";
     std::string mission_path_ = "./";
   
     double flash_next_waypoint_time_ = 5.0;
     int flash_number_ = 2;
   
     // Default limit velocity m/s
     double limit_velocity_default_ = 0.02;
   
     rclcpp::Publisher<seabot2_msgs::msg::MissionState>::SharedPtr publisher_mission_state_;
     rclcpp::Publisher<seabot2_msgs::msg::DepthControlSetPoint>::SharedPtr
       publisher_depth_control_set_point_;
     rclcpp::Publisher<seabot2_msgs::msg::TemperatureKeepingDebug>::SharedPtr
       publisher_temperature_keeping_debug_;
   
     rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
     rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr
       subscriber_temperature_data_;
   
     rclcpp::Client<seabot2_srvs::srv::Light>::SharedPtr client_light_;
     rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_log_parameters_;
     rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_bag_recorder_;
   
     rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_mission_reload_;
   
     void init_parameters();
   
     void init_interfaces();
   
     void timer_callback();
   
     void depth_callback(const seabot2_msgs::msg::DepthPose::SharedPtr msg);
   
     void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData::SharedPtr msg);
   
     void service_mission_reload_callback(
       const std::shared_ptr<rmw_request_id_t> request_header,
       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
   
     void call_light();
   
     void call_log_params() const;
   
     void call_restart_bag() const;
   
     int load_mission();
   };
   
   #endif  // SEABOT2_MISSION__MISSION_NODE_HPP_
