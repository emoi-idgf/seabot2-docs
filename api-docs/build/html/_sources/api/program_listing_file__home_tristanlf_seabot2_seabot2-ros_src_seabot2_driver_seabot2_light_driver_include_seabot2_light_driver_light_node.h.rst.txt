
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_light_driver_include_seabot2_light_driver_light_node.h:

Program Listing for File light_node.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_light_driver_include_seabot2_light_driver_light_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_light_driver/include/seabot2_light_driver/light_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_LIGHT_NODE_H
   #define BUILD_LIGHT_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <memory>
   #include "seabot2_light_driver/light.h"
   #include "seabot2_srvs/srv/light.hpp"
   #include "std_srvs/srv/set_bool.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class LightNode final: public rclcpp::Node {
   public:
     LightNode();
     ~LightNode() override;
   
   private:
   
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 100ms;   
   
     Light light_;
   
     rclcpp::Service < seabot2_srvs::srv::Light > ::SharedPtr service_light_;
     rclcpp::Service < std_srvs::srv::SetBool > ::SharedPtr service_flash_surface_;
   
     rclcpp::Time time_turn_off_light_ = this->now();
     bool special_flash_ = false;
     bool light_is_on_ = false;
     bool is_surface_ = false;
     const int nb_surface_flash_ = 1;
   
     void timer_callback();
   
     void init_parameters();
   
     void init_interfaces();
   
     void service_flash_surface_callback(
       const std::shared_ptr < rmw_request_id_t > request_header,
       const std::shared_ptr < std_srvs::srv::SetBool::Request > request,
       std::shared_ptr < std_srvs::srv::SetBool::Response > response);
   
     void service_light_callback(
       const std::shared_ptr < rmw_request_id_t > request_header,
       const std::shared_ptr < seabot2_srvs::srv::Light::Request > request,
       std::shared_ptr < seabot2_srvs::srv::Light::Response > response);
   
   };
   
   #endif //BUILD_LIGHT_NODE_H
