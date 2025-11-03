
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_thruster_driver_include_seabot2_thruster_driver_thruster_node.h:

Program Listing for File thruster_node.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_thruster_driver_include_seabot2_thruster_driver_thruster_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_thruster_driver/include/seabot2_thruster_driver/thruster_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_THRUSTER_NODE_H
   #define BUILD_THRUSTER_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <memory>
   #include "seabot2_thruster_driver/thruster.h"
   #include "seabot2_msgs/msg/engine.hpp"
   #include "seabot2_msgs/msg/velocity.hpp"
   #include "geometry_msgs/msg/twist.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class ThrusterNode final : public rclcpp::Node {
   public:
       ThrusterNode();
   
   private:
       bool state_enable_ = true;
       float velocity_linear_ = 0.0;
       float velocity_angular_ = 0.0;
       rclcpp::Time velocity_time_last;
       float manual_velocity_linear_ = 0.0;
       float manual_velocity_angular_ = 0.0;
       rclcpp::Time manual_velocity_time_last_;
   
       uint8_t cmd_left_last_ = Thruster::MOTOR_PWM_STOP;
       uint8_t cmd_right_last_ = Thruster::MOTOR_PWM_STOP;
   
       rclcpp::TimerBase::SharedPtr timer_;
       std::chrono::milliseconds loop_dt_ = 100ms; 
   
       double coeff_cmd_to_pwm_ = 9.0;
       std::chrono::milliseconds delay_stop_ = 500ms;
       double max_angular_velocity_ = 1.0;
       double max_linear_velocity_ = 1.0;
       double max_velocity_pwm_ = 5.0; 
       rclcpp::Time last_regulation_time_;
   
       bool allow_backward_ = true;
       bool reverse_angular_velocity_ = false;
       bool reverse_left_ = false;
       bool reverse_right_ = false;
   
       Thruster thruster_;
   
       rclcpp::Publisher<seabot2_msgs::msg::Engine>::SharedPtr publisher_engine_;
       rclcpp::Subscription<seabot2_msgs::msg::Velocity>::SharedPtr subscription_velocity_;
       rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_manual_velocity_;
   
       void timer_callback();
   
       void topic_velocity_callback(const seabot2_msgs::msg::Velocity &msg);
       void topic_manual_velocity_callback(const geometry_msgs::msg::Twist &msg);
   
       uint8_t convert_to_pwm(const double &u) const;
   
       void init_parameters();
   
       void init_topics();
   
       inline uint8_t invert_cmd(const uint8_t &cmd){
           const int tmp = -(static_cast<int>(cmd) - static_cast<int>(Thruster::MOTOR_PWM_STOP));
           return static_cast<uint8_t>(static_cast<uint8_t>(Thruster::MOTOR_PWM_STOP) + tmp);
       }
   };
   
   #endif //BUILD_THRUSTER_NODE_H
