
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_heading_control_include_seabot2_heading_control_heading_control_node.hpp:

Program Listing for File heading_control_node.hpp
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_heading_control_include_seabot2_heading_control_heading_control_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_heading_control/include/seabot2_heading_control/heading_control_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_HEADING_CONTROL_NODE_HPP
   #define BUILD_HEADING_CONTROL_NODE_HPP
   
   
   #include <rclcpp/rclcpp.hpp>
   #include "seabot2_msgs/msg/rpy.hpp"
   #include "seabot2_msgs/msg/velocity.hpp"
   #include "geometry_msgs/msg/twist.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   
   class HeadingControlNode : public rclcpp::Node
   {
   public:
     HeadingControlNode();
   
   private:
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 200ms;   
   
       // Parameters
     double angular_velocity_max_ = 1.0;
     std::chrono::milliseconds delay_stop_ = 200ms;
   
       // Variables
     double heading_ = 0.0;
     double angular_cmd_ = 0.0;
     double linear_cmd_ = 0.0;
     rclcpp::Time velocity_heading_time_last_;
   
     void init_parameters();
   
     void init_interfaces();
   
     void timer_callback();
   
     void imu_callback(const seabot2_msgs::msg::RPY & msg);
   
     void cmd_vel_heading_callback(const geometry_msgs::msg::Twist & msg);
   
     rclcpp::Subscription<seabot2_msgs::msg::RPY>::SharedPtr imu_sub_;
     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_heading_sub_;
     rclcpp::Publisher<seabot2_msgs::msg::Velocity>::SharedPtr control_pub_;
   };
   
   #endif  // BUILD_HEADING_CONTROL_NODE_HPP
