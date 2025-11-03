
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_include_icm20948_driver_icm20948_node.h:

Program Listing for File icm20948_node.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_include_icm20948_driver_icm20948_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/include/icm20948_driver/icm20948_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 06/06/23.
   //
   
   #ifndef BUILD_ICM20948_NODE_H
   #define BUILD_ICM20948_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <memory>
   #include "icm20948_driver/icm20948.h"
   #include "Fusion.h"
   
   #include "seabot2_msgs/msg/raw_data.hpp"
   #include "seabot2_msgs/msg/rpy.hpp"
   #include "seabot2_msgs/msg/debug_fusion.hpp"
   
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class ICM20948Node : public rclcpp::Node {
   public:
       ICM20948Node();
   
   private:
   
       rclcpp::TimerBase::SharedPtr timer_;
       std::chrono::milliseconds loop_dt_ = 20ms; 
   
       ICM20948 icm20948_;
   
       rclcpp::Publisher<seabot2_msgs::msg::RawData>::SharedPtr publisher_raw_data_;
       rclcpp::Publisher<seabot2_msgs::msg::RawData>::SharedPtr publisher_calibrated_data_;
       rclcpp::Publisher<seabot2_msgs::msg::RPY>::SharedPtr publisher_rpy_;
       rclcpp::Publisher<seabot2_msgs::msg::DebugFusion>::SharedPtr publisher_debug_fusion_;
   
       bool publish_raw_data_ = true;
       bool publish_calibrated_data_ = false;
       bool publish_rpy_ = true;
       bool publish_debug_fusion_ = false;
   
   
       // Fusion
       FusionMatrix gyroscopeMisalignment_ = {1.0f, 0.0f, 0.0f,
                                              0.0f, 1.0f, 0.0f,
                                              0.0f, 0.0f, 1.0f};
       FusionVector gyroscopeSensitivity_ = {1.0f, 1.0f, 1.0f};
       FusionVector gyroscopeOffset_ = {0.0f, 0.0f, 0.0f};
       FusionMatrix accelerometerMisalignment_ = gyroscopeMisalignment_;
       FusionVector accelerometerSensitivity_ = {1.0f, 1.0f, 1.0f};
       FusionVector accelerometerOffset_ = {0.0f, 0.0f, 0.0f};
       FusionMatrix magnetometerMisalignment_ = {1.0f, 0.0f, 0.0f,
                                                 0.0f, 1.0f, 0.0f,
                                                 0.0f, 0.0f, 1.0f};
       FusionVector magnetometerSensitivity_ = {1.0f, 1.0f, 1.0f};
       FusionVector magnetometerOffset_ = {0.0f, 0.0f, 0.0f};
       FusionMatrix softIronMatrix_ = {1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0 };
   
       FusionVector hardIronOffset_ = {0., 0., 0.};
       FusionOffset offset_{};
       FusionAhrs ahrs_{};
       FusionConvention convention_ = FusionConventionNwu;
       float fusion_gain_ = 0.5f;
       unsigned int sample_rate_ = 50;
       rclcpp::Time last_time_fusion_ = rclcpp::Time(0., RCL_STEADY_TIME);
       rclcpp::Clock steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);
       float acceleration_rejection_ = 3.0f;
       float magnetic_rejection_ = 3.0f;
       unsigned int recovery_trigger_period_ = 1;
   
       void compute_ahrs();
   
   
       void init_parameters();
   
       void init_interfaces();
   
       void timer_callback();
   
   };
   
   #endif //BUILD_ICM20948_NODE_H
