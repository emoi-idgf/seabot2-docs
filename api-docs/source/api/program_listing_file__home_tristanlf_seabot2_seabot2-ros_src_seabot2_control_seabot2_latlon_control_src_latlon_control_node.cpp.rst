
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_latlon_control_src_latlon_control_node.cpp:

Program Listing for File latlon_control_node.cpp
================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_latlon_control_src_latlon_control_node.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_latlon_control/src/latlon_control_node.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "seabot2_latlon_control/latlon_control_node.hpp"
   
   using namespace std::placeholders;
   
   LatLonControlNode::LatLonControlNode()
   : Node("latlon_control_node"), llc_()
   {
   
     init_parameters();
     init_interfaces();
   
     timer_ = this->create_wall_timer(
           loop_dt_, std::bind(&LatLonControlNode::timer_callback, this));
   
     RCLCPP_INFO(this->get_logger(), "[LatLon_control_node] Start Ok");
   }
   
   LatLonControlNode::~LatLonControlNode()
   {
     RCLCPP_INFO(this->get_logger(), "[LatLon_control_node] Shutdown");
   }
   
   void LatLonControlNode::init_parameters()
   {
     this->declare_parameter<int>("loop_dt_", static_cast<int>(loop_dt_.count()));
     loop_dt_ = std::chrono::milliseconds(this->get_parameter_or("loop_dt", loop_dt_.count()));
   
     this->declare_parameter<double>("lat_set_point", llc_.lat_set_point_);
     this->declare_parameter<double>("lon_set_point", llc_.lon_set_point_);
     this->declare_parameter<double>("linear_velocity_max", llc_.linear_velocity_max_);
     this->declare_parameter<double>("angular_velocity_max", llc_.angular_velocity_max_);
     this->declare_parameter<double>("distance_threshold_slowing_down",
       llc_.distance_threshold_slowing_down_);
   
     llc_.lat_set_point_ = this->get_parameter_or("lat_set_point", llc_.lat_set_point_);
     llc_.lon_set_point_ = this->get_parameter_or("lon_set_point", llc_.lon_set_point_);
     llc_.linear_velocity_max_ = this->get_parameter_or("linear_velocity_max",
       llc_.linear_velocity_max_);
     llc_.angular_velocity_max_ = this->get_parameter_or("angular_velocity_max",
       llc_.angular_velocity_max_);
     llc_.distance_threshold_slowing_down_ = this->get_parameter_or("distance_threshold_slowing_down",
       llc_.distance_threshold_slowing_down_);
   }
   
   void LatLonControlNode::init_interfaces()
   {
     subscriber_gps_fix_data_ = this->create_subscription<seabot2_msgs::msg::GpsFix>(
           "/observer/gps_fix", 10, std::bind(&LatLonControlNode::gps_fix_callback, this, _1));
     subscriber_imu_data_ = this->create_subscription<seabot2_msgs::msg::RawData>(
           "/observer/imu", 10, std::bind(&LatLonControlNode::imu_data_callback, this, _1));
     subscriber_safety_data_ = this->create_subscription<seabot2_msgs::msg::SafetyStatus2>(
           "/safety/safety", 10, std::bind(&LatLonControlNode::safety_callback, this, _1));
   
     publisher_thrusters_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_engine", 10);
   }
   
   void LatLonControlNode::timer_callback()
   {
   
     llc_.compute_control_commands();
   
       // Publish thruster commands based on the lat/lon control logic
     geometry_msgs::msg::Twist thrusters_cmd;
     thrusters_cmd.linear.x = llc_.get_linear_velocity();
     thrusters_cmd.angular.z = llc_.get_angular_velocity();
   
     publisher_thrusters_cmd_->publish(thrusters_cmd);
   }
   
   void LatLonControlNode::imu_data_callback(const seabot2_msgs::msg::RawData & msg)
   {
       // Update the lat/lon control with IMU data
     llc_.update_heading(msg.mag.z);
   }
   
   void LatLonControlNode::gps_fix_callback(const seabot2_msgs::msg::GpsFix & msg)
   {
       // Update the lat/lon control with GPS data
     llc_.update_lat_lon(msg.latitude, msg.longitude);
   }
   
   void LatLonControlNode::safety_callback(const seabot2_msgs::msg::SafetyStatus2 & msg)
   {
     llc_.update_safety(msg.global_safety_valid);
   }
   
   int main(int argc, char *argv[])
   {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<LatLonControlNode>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }
