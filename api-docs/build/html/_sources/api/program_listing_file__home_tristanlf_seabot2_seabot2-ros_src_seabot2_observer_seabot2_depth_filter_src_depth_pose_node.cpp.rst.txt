
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_depth_filter_src_depth_pose_node.cpp:

Program Listing for File depth_pose_node.cpp
============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_depth_filter_src_depth_pose_node.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_depth_filter/src/depth_pose_node.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "seabot2_depth_filter/depth_pose_node.hpp"
   #include <algorithm>    // std::sort
   
   using namespace placeholders;
   
   DepthPoseNode::DepthPoseNode()
   : Node("depth_pose_node")
   {
   
     init_parameters();
     init_interfaces();
   
     RCLCPP_INFO(this->get_logger(), "[Depth_pose_node] Start Ok");
   }
   
   void DepthPoseNode::init_parameters()
   {
   
     this->declare_parameter<double>("physics_rho", rho_);
     this->declare_parameter<double>("physics_g", g_);
     this->declare_parameter<double>("physics_velocity_limit", velocity_limit_);
   
     rho_ = this->get_parameter_or("physics_rho", rho_);
     g_ = this->get_parameter_or("physics_g", g_);
     velocity_limit_ = this->get_parameter_or("physics_velocity_limit", velocity_limit_);
   
     this->declare_parameter<long>("velocity_dt_gap_sample", velocity_dt_gap_sample_);
     this->declare_parameter<long>("filter_velocity_window_size", filter_velocity_window_size_);
     this->declare_parameter<long>("filter_velocity_median_remove_side_samples",
       filter_velocity_median_remove_side_samples_);
     this->declare_parameter<long>("filter_window_size", filter_window_size_);
     this->declare_parameter<long>("filter_median_remove_side_samples",
       filter_median_remove_side_samples_);
   
     this->declare_parameter<double>("threshold_wrong_depth_measure", threshold_wrong_depth_measure_);
   
     this->declare_parameter<int>("zero_depth_window_size", zero_depth_window_size_);
   
     filter_velocity_window_size_ = this->get_parameter_or("filter_velocity_window_size",
       filter_velocity_window_size_);
     velocity_dt_gap_sample_ = this->get_parameter_or("velocity_dt_gap_sample",
       velocity_dt_gap_sample_);
     filter_velocity_median_remove_side_samples_ =
       this->get_parameter_or("filter_velocity_median_remove_side_samples",
       filter_velocity_median_remove_side_samples_);
     filter_window_size_ = this->get_parameter_or("filter_window_size", filter_window_size_);
     filter_median_remove_side_samples_ = this->get_parameter_or("filter_median_remove_side_samples",
       filter_median_remove_side_samples_);
     threshold_wrong_depth_measure_ = this->get_parameter_or("threshold_wrong_depth_measure",
       threshold_wrong_depth_measure_);
   
     zero_depth_window_size_ = this->get_parameter_or("zero_depth_window_size",
       zero_depth_window_size_);
   }
   
   void DepthPoseNode::service_zero_pressure_callback(
     const std::shared_ptr<rmw_request_id_t> request_header,
     const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
   {
     if(!pressure_zero_depth_deque_.empty()) {
       zero_depth_ = std::accumulate(pressure_zero_depth_deque_.begin(),
         pressure_zero_depth_deque_.end(), 0.0);
       zero_depth_ /= static_cast<double>(pressure_zero_depth_deque_.size());
   //        RCLCPP_INFO(this->get_logger(),"[Depth_pose_node] Zero_depth = %f", zero_depth_);
       response->success = true;
     } else {
       response->success = false;
     }
   }
   
   void DepthPoseNode::pressure_callback(const seabot2_msgs::msg::PressureSensorData & msg)
   {
     seabot2_msgs::msg::DepthPose msg_pose;
     msg_pose.header.stamp = msg.header.stamp;
     msg_pose.zero_depth_pressure = zero_depth_;
   
     pressure_deque_.push_front(msg.pressure);
     if(pressure_deque_.size() > filter_window_size_) {
       pressure_deque_.pop_back();
     }
   
     pressure_zero_depth_deque_.push_front(msg.pressure);
     if(pressure_zero_depth_deque_.size() > zero_depth_window_size_) {
       pressure_zero_depth_deque_.pop_back();
     }
   
     if(pressure_deque_.size() == filter_window_size_) {
       seabot2_msgs::msg::DepthPose msg_depth;
       deque<double> pressure_deque_sort(pressure_deque_);
       sort(pressure_deque_sort.begin(), pressure_deque_sort.end());
       deque<double> pressure_deque_median(pressure_deque_sort.begin() +
         filter_median_remove_side_samples_,
         pressure_deque_sort.end() - filter_median_remove_side_samples_);
       const double pressure_sum = std::accumulate(pressure_deque_median.begin(),
         pressure_deque_median.end(), 0.0);
       const double pressure_mean = pressure_sum / static_cast<double>(pressure_deque_median.size());
   
       double pressure = (pressure_mean - zero_depth_);
       double depth_filtered = pressure / (g_ * rho_ / 1e5); 
       double depth_measured = msg.pressure / (g_ * rho_ / 1e5);
   
       msg_pose.pressure = pressure;
   
       if(abs(depth_measured - depth_filtered) < threshold_wrong_depth_measure_) {
         msg_pose.depth = depth_measured;
       } else {
         msg_pose.depth = depth_filtered;
       }
   
       depth_memory_.push_front(std::pair<double, rclcpp::Time>(depth_filtered, msg.header.stamp));
       if(depth_memory_.size() > (velocity_dt_gap_sample_ + filter_velocity_window_size_)) {
         depth_memory_.pop_back();
       }
   
       if(depth_memory_.size() == (velocity_dt_gap_sample_ + filter_velocity_window_size_)) {
         vector<double> velocities;
         for(size_t i = 0; i < filter_velocity_window_size_; i++) {
                   // Delta_depth / Delta_dt
           rclcpp::Duration dt = (depth_memory_[i].second -
             depth_memory_[velocity_dt_gap_sample_ + i].second);
           if(dt.seconds() != 0.) {
             velocities.push_back((depth_memory_[i].first -
               depth_memory_[velocity_dt_gap_sample_ + i].first) / dt.seconds());
           }
         }
         if(velocities.size() > (2 * filter_velocity_median_remove_side_samples_ + 1)) {
           sort(velocities.begin(), velocities.end());
           vector<double> velocity_median(velocities.begin() +
             filter_velocity_median_remove_side_samples_,
             velocities.end() - filter_velocity_median_remove_side_samples_);
           double velocity =
             std::accumulate(velocity_median.begin(), velocity_median.end(),
             0.0) / velocity_median.size();
           velocity = std::clamp(velocity, -velocity_limit_, velocity_limit_);
           msg_pose.velocity = velocity;
         }
       }
       publisher_depth_data_->publish(msg_pose);
     }
   }
   
   void DepthPoseNode::init_interfaces()
   {
     publisher_depth_data_ = this->create_publisher<seabot2_msgs::msg::DepthPose>("depth", 1);
   
     subscriber_pressure_data_ = this->create_subscription<seabot2_msgs::msg::PressureSensorData>(
               "/driver/pressure_external", 10,
       std::bind(&DepthPoseNode::pressure_callback, this, _1));
   
     service_zero_depth_ = this->create_service<std_srvs::srv::Trigger>("zero_pressure",
                                                                               std::bind(
       &DepthPoseNode::service_zero_pressure_callback, this, _1, _2, _3));
   }
   
   int main(const int argc, char *argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<DepthPoseNode>());
     rclcpp::shutdown();
     return 0;
   }
