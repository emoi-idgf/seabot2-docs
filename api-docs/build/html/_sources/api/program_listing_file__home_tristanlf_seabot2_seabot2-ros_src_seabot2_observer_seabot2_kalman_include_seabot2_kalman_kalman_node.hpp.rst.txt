
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_kalman_include_seabot2_kalman_kalman_node.hpp:

Program Listing for File kalman_node.hpp
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_kalman_include_seabot2_kalman_kalman_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_kalman/include/seabot2_kalman/kalman_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_KALMANN_NODE_HPP
   #define BUILD_KALMANN_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_kalman/kalman.h"
   
   #include "seabot2_msgs/msg/piston_state.hpp"
   #include "seabot2_msgs/msg/depth_pose.hpp"
   #include "seabot2_msgs/msg/kalman_state.hpp"
   #include "seabot2_msgs/msg/density.hpp"
   #include "seabot2_msgs/msg/pressure_sensor_data.hpp"
   #include "seabot2_msgs/msg/temperature_sensor_data.hpp"
   
   #include "std_msgs/msg/int32.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   using namespace Eigen;
   
   class Kalman;
   
   class KalmanNode final : public rclcpp::Node {
   public:
     KalmanNode();
   
   private:
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 200ms;   
   
     Kalman k_;
   
   
     rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
     rclcpp::Subscription<seabot2_msgs::msg::PistonState>::SharedPtr subscriber_state_data_;
     rclcpp::Subscription<seabot2_msgs::msg::Density>::SharedPtr subscriber_density_;
     rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr subscriber_temperature_;
   
     rclcpp::Publisher<seabot2_msgs::msg::KalmanState>::SharedPtr publisher_kalman_;
   
   
     void init_parameters();
   
     void init_interfaces();
   
     void state_callback(const seabot2_msgs::msg::PistonState & msg);
   
     void depth_callback(const seabot2_msgs::msg::DepthPose & msg);
   
     void density_callback(const seabot2_msgs::msg::Density & msg);
   
     void pressure_callback(const seabot2_msgs::msg::PressureSensorData & msg);
   
     void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData & msg);
   
   
     void publish_data();
   
   };
   #endif //BUILD_KALMANN_NODE_HPP
