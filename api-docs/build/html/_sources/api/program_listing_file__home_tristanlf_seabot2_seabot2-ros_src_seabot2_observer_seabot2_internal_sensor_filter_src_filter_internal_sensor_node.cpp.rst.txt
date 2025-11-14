
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_internal_sensor_filter_src_filter_internal_sensor_node.cpp:

Program Listing for File filter_internal_sensor_node.cpp
========================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_internal_sensor_filter_src_filter_internal_sensor_node.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_internal_sensor_filter/src/filter_internal_sensor_node.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "seabot2_internal_sensor_filter/filter_internal_sensor_node.hpp"
   #include <algorithm>    // std::sort
   
   using namespace placeholders;
   
   InternalSensorFilterNode::InternalSensorFilterNode()
   : Node("filter_internal_sensor_node")
   {
   
     init_parameters();
     init_interfaces();
   
     RCLCPP_INFO(this->get_logger(), "[Filter_internal_sensor_node] Start Ok");
   }
   
   void InternalSensorFilterNode::init_parameters()
   {
     this->declare_parameter<long>("filter_window_size", filter_window_size_);
     this->declare_parameter<long>("filter_median_remove_side_samples",
       filter_median_remove_side_samples_);
   
     filter_window_size_ = this->get_parameter_or("filter_window_size", filter_window_size_);
     filter_median_remove_side_samples_ = this->get_parameter_or("filter_median_remove_side_samples",
       filter_median_remove_side_samples_);
   }
   
   double InternalSensorFilterNode::compute_filter(deque<double> queue) const
   {
     sort(queue.begin(), queue.end());
     deque<double> queue_median(queue.begin() + filter_median_remove_side_samples_,
       queue.end() - filter_median_remove_side_samples_);
     const double data_sum = std::accumulate(queue_median.begin(), queue_median.end(), 0.0);
     return data_sum / static_cast<double>(queue_median.size());
   }
   
   void InternalSensorFilterNode::pressure_callback(const seabot2_msgs::msg::Bme280Data & msg)
   {
     pressure_memory_.push_front(msg.pressure);
     temperature_memory_.push_front(msg.temperature);
     humidity_memory_.push_front(msg.humidity);
   
     if(pressure_memory_.size() > filter_window_size_) {
       pressure_memory_.pop_back();
       temperature_memory_.pop_back();
       humidity_memory_.pop_back();
     }
   
     if(pressure_memory_.size() == filter_window_size_) {
       seabot2_msgs::msg::Bme280Data msg_filter;
       msg_filter.pressure = compute_filter(pressure_memory_);
       msg_filter.temperature = compute_filter(temperature_memory_);
       msg_filter.humidity = compute_filter(humidity_memory_);
       publisher_pressure_data_->publish(msg_filter);
     }
   }
   
   void InternalSensorFilterNode::init_interfaces()
   {
     publisher_pressure_data_ =
       this->create_publisher<seabot2_msgs::msg::Bme280Data>("pressure_internal", 1);
   
     subscriber_pressure_data_ = this->create_subscription<seabot2_msgs::msg::Bme280Data>(
               "/driver/pressure_internal", 10,
       std::bind(&InternalSensorFilterNode::pressure_callback, this, _1));
   
   }
   
   int main(const int argc, char *argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<InternalSensorFilterNode>());
     rclcpp::shutdown();
     return 0;
   }
