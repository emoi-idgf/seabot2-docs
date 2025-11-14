
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_lambert_include_seabot2_lambert_lambert_node.hpp:

Program Listing for File lambert_node.hpp
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_lambert_include_seabot2_lambert_lambert_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_lambert/include/seabot2_lambert/lambert_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_LAMBERT_NODE_HPP
   #define BUILD_LAMBERT_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_msgs/msg/gps_fix.hpp"
   #include "seabot2_msgs/msg/gnss_pose.hpp"
   #include <deque>
   
   #include <proj.h>
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class LambertNode final : public rclcpp::Node {
   public:
     LambertNode();
   
   private:
     PJ_CONTEXT *C_;
     PJ *P_;
     PJ * P_for_GIS_;
   
     string epsg_source_ = "EPSG:4326";
     string epsg_target_ = "EPSG:2154";
   
     deque<double> east_memory_;
     deque<double> north_memory_;
     deque<rclcpp::Time> time_memory_;
   
     std::chrono::seconds filter_position_mean_ = 5s;
     std::chrono::seconds filter_dt_heading_computation_ = 15s;
   
     rclcpp::Subscription<seabot2_msgs::msg::GpsFix>::SharedPtr subscriber_gnss_data_;
     rclcpp::Publisher<seabot2_msgs::msg::GnssPose>::SharedPtr publisher_lambert_data_;
     rclcpp::Publisher<seabot2_msgs::msg::GnssPose>::SharedPtr publisher_lambert_mean_data_;
   
   
     void init_parameters();
   
     void init_interfaces();
   
     void gnss_callback(const seabot2_msgs::msg::GpsFix & msg);
   
     void compute_mean();
   
   private:
   
   };
   #endif //BUILD_LAMBERT_NODE_HPP
