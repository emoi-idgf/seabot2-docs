
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_depth_filter_include_seabot2_depth_filter_depth_pose_node.hpp:

Program Listing for File depth_pose_node.hpp
============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_depth_filter_include_seabot2_depth_filter_depth_pose_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_depth_filter/include/seabot2_depth_filter/depth_pose_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 05/09/22.
   //
   
   #ifndef BUILD_DEPTH_POSE_NODE_HPP
   #define BUILD_DEPTH_POSE_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include <memory>
   #include "seabot2_msgs/msg/depth_pose.hpp"
   #include "seabot2_msgs/msg/pressure_sensor_data.hpp"
   #include "std_srvs/srv/trigger.hpp"
   #include <deque>
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class DepthPoseNode : public rclcpp::Node {
   public:
       DepthPoseNode();
   
   private:
   
       double rho_ = 1025.0;
       double g_ = 9.81;
       double velocity_limit_ = 0.5;
   
       double zero_depth_ = 1.0;
   
       deque<double> pressure_deque_;
       size_t filter_window_size_ = 5;
       int filter_median_remove_side_samples_ = 1;
   
       double threshold_wrong_depth_measure_ = 1.0;
   
       deque<pair<double, rclcpp::Time>> depth_memory_;
       size_t filter_velocity_window_size_ = 6;
       size_t velocity_dt_gap_sample_ = 5;
       size_t filter_velocity_median_remove_side_samples_ = 1;
   
   // Zero depth
       deque<double> pressure_zero_depth_deque_;
       size_t zero_depth_window_size_ = 50;
   
       bool new_data_ = false;
   
       rclcpp::Subscription<seabot2_msgs::msg::PressureSensorData>::SharedPtr subscriber_pressure_data_;
       rclcpp::Publisher<seabot2_msgs::msg::DepthPose>::SharedPtr publisher_depth_data_;
       rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_zero_depth_ ;
   
   
       void init_parameters();
   
       void init_interfaces();
   
       void pressure_callback(const seabot2_msgs::msg::PressureSensorData &msg);
   
       void service_zero_pressure_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
   
   private:
   
   };
   #endif //BUILD_DEPTH_POSE_NODE_HPP
