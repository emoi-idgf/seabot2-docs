
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_kalman_include_seabot2_kalman_kalman.h:

Program Listing for File kalman.h
=================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_kalman_include_seabot2_kalman_kalman.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_kalman/include/seabot2_kalman/kalman.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 12/11/22.
   //
   
   #ifndef BUILD_KALMAN_H
   #define BUILD_KALMAN_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <eigen3/Eigen/Dense>
   #include <cmath>
   
   using namespace std;
   using namespace Eigen;
   
   class Kalman {
   public:
     static const int NB_MESURES = 1;
     static const int NB_STATES = 7;
     static const int NB_COMMAND = 1;
   
   public:
     Kalman();
   
     void init_parameters(const rclcpp::Time & init_time);
   
     void update_density(double physics_rho);
   
     double fz_computation(double velocity) const;
   
     double fz_derivative_computation(double velocity) const;
   
   public:
   
     double physics_rho_ = 1000.0;
     double physics_g_ = 9.81;
     double robot_mass_ = 12.0;
     double robot_added_mass_ = 2.51;
     double robot_diameter_ = 0.125;
     double screw_thread_ = 1.e-3;
     double tick_per_turn_ = 2048 * 4;
     double piston_diameter_ = 0.045;
     double piston_max_tick_ = 1146880;
     const double degree_to_kelvin_ = 273.15;
   
       // double S_ = M_PI*pow(robot_diameter_/2.0, 2);
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
   
     double piston_max_volume_ = piston_max_tick_ * tick_to_volume_;
   
     double enable_kalman_depth_ = 0.5;   
     double piston_volume_eq_init_ = 90e-6;    
     double init_chi_ = 0.0;   
     double init_chi2_ = 0.0;   
     double init_cz_ = 2.0;
     double init_volume_air_ = 5e-3;   
   
     double gamma_alpha_velocity_ = 1e-4;
     double gamma_alpha_depth_ = 1e-5;
     double gamma_alpha_offset_ = 1e-9;
     double gamma_alpha_chi_ = 1e-15;    // 2e-8
     double gamma_alpha_chi2_ = 1e-15;    // 2e-8
     double gamma_alpha_cz_ = 1e-5;
     double gamma_alpha_volume_air_ = 1e-6;
   
     double gamma_init_velocity_ = 1e-3;
     double gamma_init_depth_ = 1.0e-3;
     double gamma_init_offset_ = 1e-6;
     double gamma_init_chi_ = 1e-10;    // 20e-
     double gamma_init_chi2_ = 1e-10;    // 1e-1
     double gamma_init_cz_ = 0.01;
     double gamma_init_volume_air_ = 1e-6;
   
     double gamma_beta_depth_ = 1e-2;    // 5e-4 (m)
   
   public:
     double fusion_depth_ = 0.;
     double fusion_velocity_ = 0.;
     rclcpp::Time fusion_stamp_;
   
     double piston_position_ = 0.;
     rclcpp::Time piston_stamp_;
   
     double temperature_ = 288.15;   
     double pressure_ = 101325.0;    
   
   public:
   /*
    *  xhat_ definition
    *  xhat_(0) velocity
    *  xhat_(1) depth
    *  xhat_(2) Piston volume to equilibrium
    *  xhat_(3) chi (chi*z)
    *  xhat_(4) chi2 (chi2*z²)
    *  xhat_(5) Cz
    *  xhat_(6) Bubble (Vb/z)
    */
     Matrix < double, NB_STATES, 1 > xhat_ = Matrix < double, NB_STATES, 1 > ::Zero();
     Matrix < double, NB_STATES, 1 > x_forcast_ = Matrix < double, NB_STATES, 1 > ::Zero();
     Matrix < double, NB_STATES, NB_STATES > gamma_ = Matrix < double, NB_STATES, NB_STATES > ::Zero();
     Matrix < double, NB_STATES, NB_STATES > gamma_forcast_ = Matrix < double, NB_STATES,
     NB_STATES > ::Zero();
     double offset_total_ = 0.0;
   
     Matrix < double, NB_STATES, NB_STATES > gamma_alpha_ = Matrix < double, NB_STATES,
     NB_STATES > ::Zero();
     Matrix < double, NB_MESURES, NB_MESURES > gamma_beta_ = Matrix < double, NB_MESURES,
     NB_MESURES > ::Zero();
     Matrix < double, NB_MESURES, NB_STATES > Ck_ = Matrix < double, NB_MESURES, NB_STATES > ::Zero();
   
     bool enable_kalman_ = true;
     bool is_valid_ = true;
     rclcpp::Time time_last_predict_;
     std::chrono::milliseconds forecast_dt_ = 0ms;
   
   private:
   
     [[nodiscard]] Matrix < double, NB_STATES, 1 > f_dyn(const Matrix < double, NB_STATES, 1 > &x,
                                         const Matrix < double, NB_COMMAND, 1 > &u) const;
   
     void kalman_predict(
       Matrix < double, NB_STATES, 1 > &x,
       Matrix < double, NB_STATES, NB_STATES > &gamma,
       const Matrix < double, NB_COMMAND, 1 > &u,
       const Matrix < double, NB_STATES, NB_STATES > &gamma_alpha,
       const double & dt) const;
   
     static void kalman_correc(
       Matrix < double, NB_STATES, 1 > &x,
       Matrix < double, NB_STATES, NB_STATES > &gamma,
       const Matrix < double, NB_MESURES, 1 > &y,
       const Matrix < double, NB_MESURES, NB_MESURES > &gamma_beta,
       const Matrix < double, NB_MESURES, NB_STATES > &Ck);
   
     void init_kalman();
   
     bool is_out_of_range(const Matrix < double, NB_STATES, 1 > &xhat) const;
   
   public:
     void compute_kalman(bool new_depth_data = false, bool new_piston_data = false);
   
     void set_new_piston_data(double position, double set_point, const rclcpp::Time & stamp);
   
     void set_new_depth_data(double depth, double velocity, const rclcpp::Time & stamp);
   
     void update_temperature(double temperature);
   
     void update_pressure(double pressure);
   
     void update_coeffAB();
   
   };
   
   
   #endif //BUILD_KALMAN_H
