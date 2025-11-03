
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_heading_control_src_heading_control_node.cpp:

Program Listing for File heading_control_node.cpp
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_heading_control_src_heading_control_node.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_heading_control/src/heading_control_node.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "seabot2_heading_control/heading_control_node.hpp"
   
   using namespace std::placeholders;
   
   double sawtooth(const double & x)
   {
       // Normalize x to the range [-pi, pi]
     return std::fmod(x + M_PI, 2 * M_PI) - M_PI;
   }
   
   HeadingControlNode::HeadingControlNode()
   : Node("heading_control_node")
   {
   
     init_parameters();
     init_interfaces();
   
     timer_ = this->create_wall_timer(
           loop_dt_, std::bind(&HeadingControlNode::timer_callback, this));
   
     velocity_heading_time_last_ = this->now();
     rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(delay_stop_));
   
     RCLCPP_INFO(this->get_logger(), "[Heading_control_node] Start Ok");
   }
   
   void HeadingControlNode::init_parameters()
   {
     this->declare_parameter<int>("loop_dt_", static_cast<int>(loop_dt_.count()));
     this->declare_parameter<double>("angular_velocity_max", angular_velocity_max_);
     this->declare_parameter<int>("delay_stop", delay_stop_.count());
   
     loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("loop_dt", loop_dt_.count()));
     angular_velocity_max_ = this->get_parameter_or("angular_velocity_max", angular_velocity_max_);
     delay_stop_ = std::chrono::milliseconds(this->get_parameter_or("delay_stop",
       delay_stop_.count()));
   }
   
   void HeadingControlNode::init_interfaces()
   {
     imu_sub_ = this->create_subscription<seabot2_msgs::msg::RPY>(
           "/driver/rpy", 10, std::bind(&HeadingControlNode::imu_callback, this, _1));
   
     cmd_vel_heading_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
           "/driver/cmd_vel_heading", 10,
       std::bind(&HeadingControlNode::cmd_vel_heading_callback, this, _1));
   
     control_pub_ = this->create_publisher<seabot2_msgs::msg::Velocity>("/driver/cmd_engine", 10);
   }
   
   void HeadingControlNode::timer_callback()
   {
   
     const rclcpp::Time new_regulation_time = this->now();
   
     double linear, angular;
     if(new_regulation_time - velocity_heading_time_last_ < delay_stop_) { 
       linear = linear_cmd_;
       angular = angular_cmd_;
     } else {
       linear = 0.0;
       angular = 0.0;
     }
   
     seabot2_msgs::msg::Velocity control_cmd;
     control_cmd.linear = linear;
     control_cmd.angular = angular;
   
     control_pub_->publish(control_cmd);
   }
   
   void HeadingControlNode::imu_callback(const seabot2_msgs::msg::RPY & msg)
   {
     heading_ = msg.yaw * M_PI / 180.;
       // RCLCPP_INFO(this->get_logger(), "[Heading_control_node] Heading: %f", heading_ * 180./M_PI);
   }
   
   void HeadingControlNode::cmd_vel_heading_callback(const geometry_msgs::msg::Twist & msg)
   {
     float heading_setpoint = sawtooth(heading_ + std::clamp(M_PI / 2 * msg.angular.z, -0.1, 0.1));
     angular_cmd_ = angular_velocity_max_ * 1. / M_PI * sawtooth(heading_setpoint - heading_);
     linear_cmd_ = msg.linear.x;
       // linear_cmd_ = 0.0; // disable linear command for safety !!! TO REMOVE
     velocity_heading_time_last_ = this->get_clock()->now();
     RCLCPP_INFO(this->get_logger(),
       "----------------------------------------------------------------");
     RCLCPP_INFO(this->get_logger(), "[Heading_control_node] Heading: %f", heading_ * 180. / M_PI);
     RCLCPP_INFO(this->get_logger(), "[Heading_control_node] Heading setpoint: %f",
       heading_setpoint * 180. / M_PI);
     RCLCPP_INFO(this->get_logger(), "[Heading_control_node] Angular cmd: %f", angular_cmd_);
     RCLCPP_INFO(this->get_logger(), "[Heading_control_node] Angular velocity max: %f",
       angular_velocity_max_);
     RCLCPP_INFO(this->get_logger(),
       "----------------------------------------------------------------");
   }
   
   
   int main(int argc, char *argv[])
   {
     rclcpp::init(argc, argv);
   
     auto node = std::make_shared<HeadingControlNode>();
   
     rclcpp::executors::MultiThreadedExecutor executor;
     executor.add_node(node);
     executor.spin();
   
     rclcpp::shutdown();
     return 0;
   }
