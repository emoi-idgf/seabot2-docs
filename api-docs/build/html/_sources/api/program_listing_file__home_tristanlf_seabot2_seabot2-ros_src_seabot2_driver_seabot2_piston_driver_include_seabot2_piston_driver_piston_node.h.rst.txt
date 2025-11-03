
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_piston_driver_include_seabot2_piston_driver_piston_node.h:

Program Listing for File piston_node.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_piston_driver_include_seabot2_piston_driver_piston_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_piston_driver/include/seabot2_piston_driver/piston_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_PISTON_NODE_H
   #define BUILD_PISTON_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_piston_driver/piston.h"
   #include <memory>
   #include "seabot2_msgs/msg/piston_state.hpp"
   #include "seabot2_msgs/msg/piston_set_point.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class PistonNode final : public rclcpp::Node {
   public:
       PistonNode();
   
   private:
   
       rclcpp::TimerBase::SharedPtr timer_;
       std::chrono::milliseconds loop_dt_ = 100ms; 
   
       bool is_detected_issue_reset_ = false;
       rclcpp::Time time_detected_issue_reset_ = this->now();
       std::chrono::milliseconds delay_detected_issue_reset_ = 5s;
   
       Piston piston_;
   
       std::chrono::seconds delay_no_data_ = 30s;
       rclcpp::Time time_last_cmd_received_ = this->now();
       int last_cmd_ = -1;
   
       int cpt_piston_error_ = 0;
       const int cpt_piston_error_max_reset_ = 100;
   
       std::chrono::seconds delay_reset_piston_ = 10s;
   
       rclcpp::Publisher<seabot2_msgs::msg::PistonState>::SharedPtr publisher_piston_state_;
       rclcpp::Subscription<seabot2_msgs::msg::PistonSetPoint>::SharedPtr subscription_position_set_point_;
   
       void timer_callback();
   
       void init_parameters();
   
       void init_interfaces();
   
       void topic_position_set_point_callback(const seabot2_msgs::msg::PistonSetPoint &msg);
   
   };
   
   #endif //BUILD_PISTON_NODE_H
