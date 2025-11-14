
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_temperature_filter_include_seabot2_temperature_filter_filter_temperature_node.hpp:

Program Listing for File filter_temperature_node.hpp
====================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_temperature_filter_include_seabot2_temperature_filter_filter_temperature_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_temperature_filter/include/seabot2_temperature_filter/filter_temperature_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_TEMPERATURE_FILTER_NODE_HPP
   #define BUILD_TEMPERATURE_FILTER_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include <memory>
   #include "seabot2_msgs/msg/temperature_sensor_data.hpp"
   #include <deque>
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class TemperatureFilterNode final : public rclcpp::Node {
   public:
     TemperatureFilterNode();
   
   private:
     size_t filter_window_size_ = 5;
     size_t filter_median_remove_side_samples_ = 1;
   
     deque<double> temperature_memory_;
   
     rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr
       subscriber_temperature_data_;
     rclcpp::Publisher<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr publisher_temperature_data_;
   
   
     void init_parameters();
   
     void init_interfaces();
   
     void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData & msg);
   
     double compute_filter(deque<double> queue) const;   
   
   private:
   
   };
   #endif //BUILD_TEMPERATURE_FILTER_NODE_HPP
