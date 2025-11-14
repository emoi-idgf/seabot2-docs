
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_mission_src_waypoint.cpp:

Program Listing for File waypoint.cpp
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_mission_src_waypoint.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_mission/src/waypoint.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #include "seabot2_mission/waypoint.hpp"
   
   void WaypointDepth::process(const rclcpp::Time & time)
   {
     seabot2_msgs::msg::DepthControlSetPoint & dc_msg = mission_->depth_control_set_point();
   
     dc_msg.depth = depth;
     dc_msg.limit_velocity = velocity;
     dc_msg.enable_control = true;
     dc_msg.header.stamp = time;
   
   }
   
   void WaypointSeafloorLanding::process(const rclcpp::Time & time)
   {
     seabot2_msgs::msg::DepthControlSetPoint & dc_msg = mission_->depth_control_set_point();
   
       // Wait until the seabot2 reached 0.5m above the seafloor (for instance) and save piston volume value
       // Then shutdown kalman and depth control, and set the piston to minimum volume
       // When the waypoint is near ending, set back the piston to the previous volume
       // and then restart kalman and depth control
   }
   
   void WaypointTemperatureKeeping::process(const rclcpp::Time & time)
   {
     seabot2_msgs::msg::DepthControlSetPoint & dc_msg = mission_->depth_control_set_point();
   
     double depth_set_point;
     if(mission_->get_temperature() > this->temperature_) {
       depth_set_point = 100.0;
     } else {
       depth_set_point = 2.0;
     }
   
     error_temperature_ = abs(this->temperature_ - mission_->get_temperature());
     double velocity = fmin(mission_->temperature_keeping_k_ * error_temperature_, this->velocity);
   
     dc_msg.depth = depth_set_point;
     dc_msg.limit_velocity = velocity;
     dc_msg.enable_control = true;
     dc_msg.header.stamp = time;
   }
   
   void WaypointTemperatureProfile::process(const rclcpp::Time & time)
   {
     seabot2_msgs::msg::DepthControlSetPoint & dc_msg = mission_->depth_control_set_point();
   
     if(first_init_) {
       time_last_transition_ = time;
       first_init_ = false;
   
       dc_msg.limit_velocity = this->velocity;
       dc_msg.enable_control = true;
       dc_msg.depth = depth_max_;
       state_ = PROFILE_GO_DOWN;
     }
     dc_msg.header.stamp = time;
   
     switch (state_) {
       case PROFILE_GO_DOWN:
         if(mission_->get_temperature() < temperature_low_ ||
           mission_->get_depth() > depth_max_ ||
           (time - time_last_transition_) > max_delay_)
         {
           state_ = PROFILE_GO_UP;
           dc_msg.depth = depth_min_;
           time_last_transition_ = time;
         }
         break;
       case PROFILE_GO_UP:
         if(mission_->get_temperature() > temperature_high_ ||
           mission_->get_depth() < depth_min_ ||
           (time - time_last_transition_) > max_delay_)
         {
           state_ = PROFILE_GO_DOWN;
           dc_msg.depth = depth_max_;
           time_last_transition_ = time;
         }
         break;
     }
   }
   
   void WaypointGNSSProfile::process(const rclcpp::Time & time)
   {
     seabot2_msgs::msg::DepthControlSetPoint & dc_msg = mission_->depth_control_set_point();
   
     dc_msg.depth = 0.0;
     dc_msg.limit_velocity = 0.1;
     dc_msg.enable_control = true;
     dc_msg.header.stamp = time;
   }
