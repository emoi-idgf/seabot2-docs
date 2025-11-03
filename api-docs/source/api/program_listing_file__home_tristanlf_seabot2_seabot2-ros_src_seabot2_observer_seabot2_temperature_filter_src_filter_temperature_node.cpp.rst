
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_temperature_filter_src_filter_temperature_node.cpp:

Program Listing for File filter_temperature_node.cpp
====================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_temperature_filter_src_filter_temperature_node.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_temperature_filter/src/filter_temperature_node.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "seabot2_temperature_filter/filter_temperature_node.hpp"
   #include <algorithm>    // std::sort
   
   using namespace placeholders;
   
   TemperatureFilterNode::TemperatureFilterNode()
           : Node("filter_temperature_node"){
   
       init_parameters();
       init_interfaces();
   
       RCLCPP_INFO(this->get_logger(), "[Filter_internal_sensor_node] Start Ok");
   }
   
   void TemperatureFilterNode::init_parameters() {
       this->declare_parameter<long>("filter_window_size", filter_window_size_);
       this->declare_parameter<long>("filter_median_remove_side_samples", filter_median_remove_side_samples_);
   
       filter_window_size_ = this->get_parameter_or("filter_window_size", filter_window_size_);
       filter_median_remove_side_samples_ = this->get_parameter_or("filter_median_remove_side_samples", filter_median_remove_side_samples_);
   }
   
   double TemperatureFilterNode::compute_filter(deque<double> queue) const {
       sort(queue.begin(), queue.end());
       deque<double> queue_median(queue.begin()+filter_median_remove_side_samples_, queue.end()-filter_median_remove_side_samples_);
       const double data_sum = std::accumulate(queue_median.begin(), queue_median.end(), 0.0);
       return data_sum / static_cast<double>(queue_median.size());
   }
   
   void TemperatureFilterNode::temperature_callback(const seabot2_msgs::msg::TemperatureSensorData &msg) {
       temperature_memory_.push_front(msg.temperature);
   
       if(temperature_memory_.size()>filter_window_size_) {
           temperature_memory_.pop_back();
       }
   
       if(temperature_memory_.size()==filter_window_size_){
           seabot2_msgs::msg::TemperatureSensorData msg_filter;
           msg_filter.temperature = compute_filter(temperature_memory_);
           publisher_temperature_data_->publish(msg_filter);
       }
   }
   
   void TemperatureFilterNode::init_interfaces() {
       publisher_temperature_data_ = this->create_publisher<seabot2_msgs::msg::TemperatureSensorData>("temperature", 1);
   
       subscriber_temperature_data_ = this->create_subscription<seabot2_msgs::msg::TemperatureSensorData>(
               "/driver/temperature", 10, std::bind(&TemperatureFilterNode::temperature_callback, this, _1));
   }
   
   int main(int argc, char *argv[]) {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<TemperatureFilterNode>());
       rclcpp::shutdown();
       return 0;
   }
