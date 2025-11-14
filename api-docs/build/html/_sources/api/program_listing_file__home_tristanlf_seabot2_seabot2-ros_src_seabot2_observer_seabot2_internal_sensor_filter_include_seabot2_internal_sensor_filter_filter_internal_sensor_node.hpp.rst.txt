
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_internal_sensor_filter_include_seabot2_internal_sensor_filter_filter_internal_sensor_node.hpp:

Program Listing for File filter_internal_sensor_node.hpp
========================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_internal_sensor_filter_include_seabot2_internal_sensor_filter_filter_internal_sensor_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_internal_sensor_filter/include/seabot2_internal_sensor_filter/filter_internal_sensor_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_INTERNAL_SENSOR_FILTER_NODE_HPP
   #define BUILD_INTERNAL_SENSOR_FILTER_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_msgs/msg/bme280_data.hpp"
   #include <deque>
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class InternalSensorFilterNode : public rclcpp::Node {
   public:
     InternalSensorFilterNode();
   
   private:
     size_t filter_window_size_ = 5;
     size_t filter_median_remove_side_samples_ = 1;
   
     deque<double> pressure_memory_;
     deque<double> temperature_memory_;
     deque<double> humidity_memory_;
   
     rclcpp::Subscription<seabot2_msgs::msg::Bme280Data>::SharedPtr subscriber_pressure_data_;
     rclcpp::Publisher<seabot2_msgs::msg::Bme280Data>::SharedPtr publisher_pressure_data_;
   
   
     void init_parameters();
   
     void init_interfaces();
   
     void pressure_callback(const seabot2_msgs::msg::Bme280Data & msg);
   
     double compute_filter(deque<double> queue) const;   
   
   private:
   
   };
   #endif //BUILD_INTERNAL_SENSOR_FILTER_NODE_HPP
