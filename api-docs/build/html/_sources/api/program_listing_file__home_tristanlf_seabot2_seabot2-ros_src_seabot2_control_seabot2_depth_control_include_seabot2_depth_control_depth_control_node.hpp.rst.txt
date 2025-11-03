
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_depth_control_include_seabot2_depth_control_depth_control_node.hpp:

Program Listing for File depth_control_node.hpp
===============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_depth_control_include_seabot2_depth_control_depth_control_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_depth_control/include/seabot2_depth_control/depth_control_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 05/09/22.
   //
   
   #ifndef BUILD_DEPTH_CONTROL_NODE_HPP
   #define BUILD_DEPTH_CONTROL_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   
   #include "seabot2_depth_control/alpha_solver.h"
   #include "seabot2_depth_control/depth_control.h"
   
   #include "seabot2_msgs/msg/depth_control_set_point.hpp"
   #include "seabot2_msgs/msg/kalman_state.hpp"
   #include "seabot2_msgs/msg/piston_state.hpp"
   #include "seabot2_msgs/msg/depth_pose.hpp"
   #include "seabot2_msgs/msg/depth_control_debug.hpp"
   #include "seabot2_msgs/msg/piston_set_point.hpp"
   #include "seabot2_msgs/msg/safety_status2.hpp"
   #include "seabot2_msgs/msg/density.hpp"
   #include "seabot2_msgs/msg/temperature_sensor_data.hpp"
   #include "seabot2_msgs/msg/alpha_debug.hpp"
   
   #include "seabot2_srvs/srv/alpha_mission.hpp"
   #include "std_srvs/srv/trigger.hpp"
   
   
   #define NB_STATES 8
   #define PISTON_STATE_OK 2
   
   using namespace std::chrono_literals;
   using namespace std;
   using namespace Eigen;
   
   class DepthControlNode : public rclcpp::Node {
   public:
     DepthControlNode();
   
   private:
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 200ms;   
   
     rclcpp::CallbackGroup::SharedPtr callback_group_;
   
     DepthControl dc_;
   
     std::vector<float> velocity_limits_requests_;
     bool velocity_limits_computations_ = false;
   
     bool enable_control_ = true;   // Allow publish set point to piston
     enum WAYPOINT_TYPE:unsigned int {WP_IDLE=0,
       WP_DEPTH=1,
       WP_SEAFLOOR_LANDING=2,
       WP_TEMPERATURE_KEEPING=3,
       WP_TEMPERATURE_PROFILE=4,
       WP_GNSS_PROFILE=5,
       WP_STOP=6};
     WAYPOINT_TYPE wp_type_ = WP_IDLE;
   
     std::vector<double> solver_velocity_, solver_alpha_;
   
     rclcpp::Subscription<seabot2_msgs::msg::KalmanState>::SharedPtr subscriber_kalman_data_;
     rclcpp::Subscription<seabot2_msgs::msg::PistonState>::SharedPtr subscriber_state_data_;
     rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
     rclcpp::Subscription<seabot2_msgs::msg::DepthControlSetPoint>::SharedPtr subscriber_mission_data_;
     rclcpp::Subscription<seabot2_msgs::msg::SafetyStatus2>::SharedPtr subscriber_safety_data_;
     rclcpp::Subscription<seabot2_msgs::msg::Density>::SharedPtr subscriber_density_;
     rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr
       subscriber_temperature_data_;
   
     rclcpp::Publisher<seabot2_msgs::msg::PistonSetPoint>::SharedPtr publisher_piston_;
     rclcpp::Publisher<seabot2_msgs::msg::DepthControlDebug>::SharedPtr publisher_debug_;
     rclcpp::Publisher<seabot2_msgs::msg::AlphaDebug>::SharedPtr publisher_alpha_debug_;
   
     rclcpp::Service<seabot2_srvs::srv::AlphaMission>::SharedPtr service_alpha_computation_;
     rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_alpha_generation_;
   
     void init_parameters();
   
     void init_interfaces();
   
     void timer_callback();
   
     void kalman_callback(const seabot2_msgs::msg::KalmanState & msg);
   
     void density_callback(const seabot2_msgs::msg::Density & msg);
   
     void piston_callback(const seabot2_msgs::msg::PistonState & msg);
   
     void depth_callback(const seabot2_msgs::msg::DepthPose & msg);
   
     void safety_callback(const seabot2_msgs::msg::SafetyStatus2 & msg);
   
     void depth_set_point_callback(const seabot2_msgs::msg::DepthControlSetPoint & msg);
   
     void temperature_callback(const seabot2_msgs::msg::TemperatureSensorData & msg);
   
     void alpha_mission_pre_computation(
       const std::shared_ptr<rmw_request_id_t> request_header,
       const std::shared_ptr<seabot2_srvs::srv::AlphaMission::Request> request,
       std::shared_ptr<seabot2_srvs::srv::AlphaMission::Response> response);
   
     void alpha_generation(
       const std::shared_ptr<rmw_request_id_t> request_header,
       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
     void publish_message();
   
     void generate_velocity_pairs();
   };
   #endif //BUILD_DEPTH_CONTROL_NODE_HPP
