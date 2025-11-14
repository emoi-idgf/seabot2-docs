
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_gpsd_client_include_gpsd_client_gpsd_node.h:

Program Listing for File gpsd_node.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_gpsd_client_include_gpsd_client_gpsd_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/gpsd_client/include/gpsd_client/gpsd_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_GPSD_NODE_H
   #define BUILD_GPSD_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <memory>
   #include "seabot2_msgs/msg/gps_fix.hpp"
   #include "seabot2_msgs/msg/gps_pps.hpp"
   #include <libgpsmm.h>
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class GpsdNode: public rclcpp::Node {
   public:
     GpsdNode();
   
     ~GpsdNode();
   
   private:
     gpsmm * gps_ = nullptr;
     string frame_id_ = "gps";
     bool last_msg_no_fix_ = false;
     bool publish_when_no_fix_ = true;
   
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 100ms;   // loop dt
   
     rclcpp::Publisher < seabot2_msgs::msg::GpsFix > ::SharedPtr publisher_fix_;
     rclcpp::Publisher < seabot2_msgs::msg::GpsPps > ::SharedPtr publisher_pps_;
   
     void init_parameters();
   
     void init_interfaces();
   
     void process_data(const struct gps_data_t * p);
   
     void timer_callback();
   };
   
   #endif //BUILD_GPSD_NODE_H
