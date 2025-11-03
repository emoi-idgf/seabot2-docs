
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_bluerobotics_ping_driver_include_bluerobotics_ping_driver_ping_node.h:

Program Listing for File ping_node.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_bluerobotics_ping_driver_include_bluerobotics_ping_driver_ping_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/bluerobotics_ping_driver/include/bluerobotics_ping_driver/ping_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_PING_NODE_H
   #define BUILD_PING_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <memory>
   #include <ping-device-ping1d.h>
   #include <abstract-link.h>
   #include <seabot2_msgs/msg/profile.hpp>
   #include "std_srvs/srv/set_bool.hpp"
   #include <seabot2_msgs/msg/density.hpp>
   
   using namespace std::chrono_literals;
   
   class PingNode : public rclcpp::Node {
   public:
       PingNode();
       ~PingNode();
   
   
       void wait_message() const;
   
   private:
   
       rclcpp::Publisher<seabot2_msgs::msg::Profile>::SharedPtr publisher_profile_;
       rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_ping_enable_;
       rclcpp::Subscription<seabot2_msgs::msg::Density>::SharedPtr subscriber_density_;
   
       std::string uart_port_ = "/dev/ping1D";
       unsigned int uart_baudrate_ = 115200;
       std::unique_ptr<Ping1d> device_;
       std::shared_ptr<AbstractLink> port_;
   
       bool enable_ping_ = false;
       bool mode_auto_ = true; 
       double speed_of_sound_ = 1550.0; 
       int ping_interval_ = 200; 
       int gain_setting_ = 1; 
   
       void init_parameters();
   
       void init_interfaces();
   
       void init_driver();
   
       void ping_enable_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);
   
       void sound_speed_callback(const seabot2_msgs::msg::Density &msg);
   };
   
   #endif //BUILD_PING_NODE_H
