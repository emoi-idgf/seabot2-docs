
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_depth_control_include_seabot2_depth_control_depth_control.h:

Program Listing for File depth_control.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_depth_control_include_seabot2_depth_control_depth_control.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_depth_control/include/seabot2_depth_control/depth_control.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 18/05/23.
   //
   
   #ifndef SEABOT2_DEPTH_CONTROL__DEPTH_CONTROL_H_
   #define SEABOT2_DEPTH_CONTROL__DEPTH_CONTROL_H_
   
   #include <eigen3/Eigen/Dense>
   #include <cmath>
   #include <cstdint>
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_depth_control/alpha_solver.h"
   
   using namespace std::chrono_literals;
   
   class DepthControl {
   public:
     DepthControl(const rclcpp::Time & start_time);
   
     static const int PISTON_STATE_OK = 2;
   
     bool debug_ = false;
   
     AlphaSolver alpha_solver_;
   
     bool emergency_ = true;
     float limit_depth_ = 100.0;
   
     double physics_rho_ = 1000.0;
     double physics_g_ = 9.81;
     double robot_mass_ = 12.0;
     double robot_added_mass_ = 2.51;
     double robot_diameter_ = 0.125;
     double screw_thread_ = 1.e-3;
     double tick_per_turn_ = 2048 * 4;
     double piston_diameter_ = 0.045;
     double piston_max_tick_value_ = 1146880;
     const double degree_to_kelvin_ = 273.15;
   
     double tick_to_volume_ = (screw_thread_ / tick_per_turn_) * pow(piston_diameter_ / 2.0, 2) * M_PI;
     double coeff_A_ = physics_g_ * physics_rho_ / (robot_added_mass_ + robot_mass_);
       // double coeff_B_ = 0.5 * physics_rho_ * S_ / (2.0 * robot_mass_);
     double coeff_B_ = 0.5 * physics_rho_ / (robot_added_mass_ + robot_mass_);
   
     const double fz_model_boundary_velocity_positive_ = 0.267911611144239;
     const double fz_model_boundary_velocity_negative_ = -0.272937623109179;
   
     const double fz_derivative_at_model_boundary_positive_ =
       fz_derivative_computation(fz_model_boundary_velocity_positive_);
     const double fz_offset_at_model_boundary_positive_ =
       fz_computation(fz_model_boundary_velocity_positive_) -
       fz_derivative_at_model_boundary_positive_ * fz_model_boundary_velocity_positive_;
     const double fz_derivative_at_model_boundary_negative_ =
       fz_derivative_computation(fz_model_boundary_velocity_negative_);
     const double fz_offset_at_model_boundary_negative_ =
       fz_computation(fz_model_boundary_velocity_negative_) -
       fz_derivative_at_model_boundary_negative_ * fz_model_boundary_velocity_negative_;
   
     double root_regulation_ = -0.2;
     double limit_depth_control_ = 0.5;   
     double flow_piston_sink_ = -5e-7;   
   
     double piston_reach_position_dead_zone_ = 50.;
     double piston_hysteresis_ = 0.6;
     double motor_max_rpm_ = 38.0;
     double flow_max_ = (motor_max_rpm_ / 60.) * tick_per_turn_ * tick_to_volume_;   
   
     double piston_flow_security_percentage_ = 0.5;
   
     bool hold_depth_enable_ = false;
     rclcpp::Duration hold_depth_validation_duration_ = 30s;
     rclcpp::Time hold_depth_validation_time_ {};
     bool hold_depth_validation_ = false;
     double hold_depth_value_enter_ = 0.0;   
     double hold_depth_value_exit_ = 0.0;   
     double hold_velocity_enter_ = 0.0;   
     double hold_velocity_exit_ = 0.0;
   
     bool control_filter_ = true;
     double delta_velocity_lb_ = -1e-3;
     double delta_velocity_ub_ = 1e-3;
     double delta_position_lb_ = -10e-3;
     double delta_position_ub_ = 10e-3;
   
     rclcpp::Duration safety_time_no_data_ = 5s;
   
     int64_t piston_position_ = 0;
     bool piston_switch_top_ = false;
     bool piston_switch_bottom_ = false;
     int32_t piston_state_ = 0;
     double piston_set_point_ = 0.;
     bool is_exit_ = true;
   
     rclcpp::Time time_last_kalman_callback_ {};
     rclcpp::Time time_last_piston_callback_ {};
   
       // [Velocity; Depth; Volume; Offset, chi, chi2, Cz]
     double kalman_velocity_ = 0.;
     double kalman_depth_ = 0.;
     double kalman_total_offset_ = 100.e-6;
     double kalman_chi_ = 0.;
     double kalman_chi2_ = 0.;
     double kalman_cz_ = 3.;
     double piston_volume_ = 0.;
   
     double depth_fusion_ = 0.0;
     double depth_set_point_ = 0.0;
     double limit_velocity_ = 0.0;
     double limit_velocity_last_ = 0.0;
     double approach_velocity_ = 1.0;
   
     double temperature_ = 288.15;
     double pressure_ = 101325;
   
     std::chrono::milliseconds last_waypoint_max_delay_ = 5s;
     rclcpp::Time last_waypoint_time_ {};
   
     enum STATE_MACHINE {STATE_SURFACE, STATE_SINK, STATE_CONTROL, STATE_PISTON_ISSUE,
       STATE_HOLD_DEPTH, STATE_COMPUTE_ALPHA, STATE_STOP};
     STATE_MACHINE regulation_state_ = STATE_SURFACE;
   
     double y_debug_ = 0.0;
     double dy_debug_ = 0.0;
     double u_debug_ = 0.0;
   
   public:
     void update_state(
       const double & velocity,
       const double & depth,
       const double & chi,
       const double & chi2,
       const double & cz,
       const double & offset_total,
       const rclcpp::Time & time_update);
   
     void update_piston(
       const int & position,
       const bool & switch_top,
       const bool & switch_bottom,
       const int & state,
       const rclcpp::Time & time_update);
   
     void update_depth(
       const double & depth,
       const double & pressure);
   
     void update_safety(
       const bool & emergency,
       const float & limit_depth);
   
     void update_waypoint(
       const float & depth,
       const double & limit_velocity,
       const rclcpp::Time & time_update,
       const bool & enable_control);
   
     void update_density(const float & density);
   
     void update_temperature(const float & temperature);
   
     double compute_u(double set_point, double limit_velocity);
   
     static double optimize_u(std::array < double, 4 > &u_tab);
   
     void update_coeff();
   
     void state_machine_step(const rclcpp::Duration & dt, const rclcpp::Time & current_time);
   
     void set_start_time(const rclcpp::Time & start_time);
   
     void update_AB();
   
     double fz_computation(double velocity) const;
   
     double fz_derivative_computation(double velocity) const;
   };
   
   
   #endif  // SEABOT2_DEPTH_CONTROL__DEPTH_CONTROL_H_
