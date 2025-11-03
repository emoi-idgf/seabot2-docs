
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_power_filter_include_seabot2_power_filter_filter_power_node.hpp:

Program Listing for File filter_power_node.hpp
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_power_filter_include_seabot2_power_filter_filter_power_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_power_filter/include/seabot2_power_filter/filter_power_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_FILTER_POWER_NODE_HPP
   #define BUILD_FILTER_POWER_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_msgs/msg/power_state.hpp"
   #include <deque>
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class FilterPowerNode final : public rclcpp::Node {
   public:
       FilterPowerNode();
   
   private:
   
       size_t filter_window_size_ = 5;
       size_t filter_median_remove_side_samples_ = 1;
   
       deque<double> battery_volt_memory_;
       deque<double> motor_current_memory_;
       array<deque<double>,2> esc_current_memory_;
       array<deque<double>, 4> cell_volt_memory_;
   
       rclcpp::Subscription<seabot2_msgs::msg::PowerState>::SharedPtr subscriber_power_data_;
       rclcpp::Publisher<seabot2_msgs::msg::PowerState>::SharedPtr publisher_power_data_;
   
   
       void init_parameters();
   
       void init_interfaces();
   
       void power_callback(const seabot2_msgs::msg::PowerState &msg);
   
       double compute_filter(deque<double> queue) const; 
   
   private:
   
   };
   #endif //BUILD_FILTER_POWER_NODE_HPP
