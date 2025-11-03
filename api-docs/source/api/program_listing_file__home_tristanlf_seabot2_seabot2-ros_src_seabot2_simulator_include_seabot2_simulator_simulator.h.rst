
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_simulator_include_seabot2_simulator_simulator.h:

Program Listing for File simulator.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_simulator_include_seabot2_simulator_simulator.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_simulator/include/seabot2_simulator/simulator.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 06/11/22.
   //
   
   #ifndef BUILD_SIMULATOR_H
   #define BUILD_SIMULATOR_H
   
   #include <eigen3/Eigen/Dense>
   
   #include "seabot2_kalman/kalman.h"
   #include "seabot2_depth_control/depth_control.h"
   
   #include "seabot2_mission/mission.hpp"
   #include <random>
   #include "rosbag2_cpp/writer.hpp"
   #include "seabot2_temperature_profile/temperature_profile.h"
   
   #include "TeosCpp/TeosSea.h"
   
   //using namespace std::chrono_literals;
   using namespace std;
   using namespace Eigen;
   
   class WaveGenerator{
   public:
       WaveGenerator(const double amplitude,
                     const double period,
                     const double phase,
                     const double offset,
                     const bool water_velocity,
                     const bool is_contraction = false,
                     const double starting_time = 0.0,
                     const double duration = 0.0){
           amplitude_ = amplitude;
           period_ = period;
           phase_ = phase;
           offset_ = offset;
           water_velocity_ = water_velocity;
           is_contraction_ = is_contraction;
           starting_time_ = starting_time;
           duration_ = duration;
       }
   
       double amplitude_{}, period_{}, phase_{}, offset_{}, starting_time_{}, duration_{};
       bool is_contraction_{}, water_velocity_{};
   };
   
   #define SIMU_NB_STATES 5
   
   #define MOTOR_STOP 2000
   #define MOTOR_DEAD_ZONE 50
   #define MOTOR_DOWN 500
   #define MOTOR_UP 3500
   
   class Simulator{
   
   public:
       Simulator();
   
       Matrix<double, SIMU_NB_STATES, 1> f(const Matrix<double, SIMU_NB_STATES, 1> &x, int pwm=MOTOR_STOP);
   
       void run_simulation();
   
       double salinity_from_depth(double z);
   
       double fz_computation(double velocity) const;
   
       double fz_derivative_computation(double velocity) const;
   
       double temperature_from_depth(double z);
   
       double get_density_from_depth(double z, double sea_pressure);
   
       int control_pwm(int position_set_point);
   
       void simulate_sensors();
   
       void simulate_piston_position();
   
       void save_data(const rclcpp::Time &time);
   
       void write_to_file_fz() const;
   
       void init_bag_writer();
   
       double find_index_center_thermocline();
   
       int init_wave_file();
   
       std::array<double, 3> compute_wave(double t, bool water_velocity=true);
   
       void compute_std_generators();
   
       double depth_from_temperature(double temperature);
   
   private:
       std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;
   
   public:
   
       rclcpp::Time start_time_= rclcpp::Time(0., RCL_ROS_TIME);
       rclcpp::Time end_time_= rclcpp::Time(0., RCL_ROS_TIME);
       rclcpp::Duration dt_ = 1000us;
       rclcpp::Time t_ = rclcpp::Time(0., RCL_ROS_TIME);
       unsigned long int nb_steps = 0;
       /* state variable x_
        * x[0] : theta (motor)
        * x[1] : dtheta (motor)
        * x[2] : i (current)
        * x[3] : dz (velocity)
        * x[4] : z (depth)
        */
       Matrix<double, SIMU_NB_STATES, 1> x_ = Matrix<double, SIMU_NB_STATES, 1>::Zero();
       int motor_cmd_ = MOTOR_STOP;
       double salinity_{}, rho_{}, g_{};
       double sea_pressure_{}; // Sea pressure at depth z in dbar (0 dbar at depth 0.0m)
       double abs_pressure_{}; // Absolute pressure at depth z in Pa (101325 Pa at depth 0.0m)
       double piston_volume_ = 0.0;
       double volume_total_ = 0.0;
       double volume_air_ = 0.0;
       double volume_antenna_ = 0.0;
       double temperature_degree_ = 10.0;
   
       double latitude_ = 48.368894;
       double seafloor_depth_ = 40.0;
       double seafloor_hardness_ = 0.8; // Absorption of velocity when impact with seafloor
       double salinity_cst_ = 0.0;
       double robot_mass_ =  12.0;
       double robot_added_mass_ =  2.51;
       const double robot_diameter_ =  0.125;
       const double screw_thread_ =  1.e-3;
       double tick_per_turn_ =  2048*4;
       const double piston_diameter_ =  0.045;
       double piston_max_tick_ =  1146880;
       double screw_Radius_ = 6e-3; 
       double screw_FrictionCoefficient_ = 0.2; 
       // double S_ = M_PI*pow(robot_diameter_/2.0, 2);
   
       double battery_tension_ = 16.0; 
       double volume_equilibrium_ = 90e-6; 
       double chi_ = 0.0;
       double chi2_ = 0.0;
       double volume_air_V0_ = 15e-6; //15e-6; /// m3
       double volume_air_P0_ = 101325.0; 
       double volume_air_T0_ = (273.15+15.0);
   
       double volume_air_nR_ = 0.0;
   
       const double fz_model_boundary_velocity_positive_ = 0.267911611144239;
       const double fz_model_boundary_velocity_negative_ = -0.272937623109179;
   
       const double fz_derivative_at_model_boundary_positive_ = fz_derivative_computation(fz_model_boundary_velocity_positive_);
       const double fz_offset_at_model_boundary_positive_ = fz_computation(fz_model_boundary_velocity_positive_)
                                           - fz_derivative_at_model_boundary_positive_ * fz_model_boundary_velocity_positive_;
       const double fz_derivative_at_model_boundary_negative_ = fz_derivative_computation(fz_model_boundary_velocity_negative_);
       const double fz_offset_at_model_boundary_negative_ = fz_computation(fz_model_boundary_velocity_negative_)
                                           - fz_derivative_at_model_boundary_negative_ * fz_model_boundary_velocity_negative_;
   
       const double maxon_RotorInertia_ = 9.0; 
       const double maxon_SpeedConstant_ = 416; 
       const double maxon_TorqueConstant_ = 22.9; 
       const double maxon_TerminalResistance_ = 1.84; 
       const double maxon_TerminalInductance_ = 0.198; 
       const double maxon_Reduction_ = 103; 
       const double seabot_AddedInductance_ = 1.619e-3; 
       const double seabot_AddedInductanceResistance_ = 480e-3; 
       const double rpm_to_rad_s_ = M_PI/30.;
       const double J_ = maxon_RotorInertia_ * 1e-7; 
       const double Ke_ = 1./(maxon_SpeedConstant_*rpm_to_rad_s_); 
       const double Kt_ = maxon_TorqueConstant_*1e-3; 
       const double R_ = maxon_TerminalResistance_ + seabot_AddedInductanceResistance_; 
       const double L_ = maxon_TerminalInductance_*1e-3 + seabot_AddedInductance_; 
       const double rad_to_tick_ = tick_per_turn_/(2*M_PI*maxon_Reduction_);
       // const double Tfstatic_ = Kt_*20e-3; /// 20mA
       const double pistonSurface_ = M_PI*pow(piston_diameter_/2.,2);
       const double i_screw_=screw_thread_/(2*screw_Radius_); 
       const double force_to_torque_coeff_ = screw_Radius_*tan(screw_FrictionCoefficient_+i_screw_);
       //const double Tz_coeff = pistonSurface_ * force_to_torque_coeff_ / maxon_Reduction_;
   
   #define REGULATION_LOOP_FREQ 50
   #define MOTOR_PWM_MAX 4000
   #define MOTOR_V_TO_CMD MOTOR_PWM_MAX/(2*16)
       int motor_delta_speed = (100/REGULATION_LOOP_FREQ)*MOTOR_V_TO_CMD; // Limit to 100V/s, delta_speed in PWM quantum/0.02s = 250
       rclcpp::Time control_pwm_last_time = rclcpp::Time(0., RCL_ROS_TIME);
       rclcpp::Duration control_pwm_dt = 20ms; 
       bool switch_top_ = false, switch_bottom_ = true;
       int motor_set_point_ = 0;
       const int motor_regulation_dead_zone_ = 50;
       const double motor_regulation_K_ = 0.3;
   
       TeosSea ts;
   
       rclcpp::Duration memory_dt = 100ms; 
       rclcpp::Time memory_last_time = rclcpp::Time(0., RCL_ROS_TIME);
   
       Kalman k_;
   
       TemperatureProfile tp_;
   
       rclcpp::Time pressure_sensor_last_time = rclcpp::Time(0., RCL_ROS_TIME);
       rclcpp::Duration pressure_sensor_dt = 200ms; 
       double pressure_sensor_ = 0.; 
       const double pressure_sensor_mean_ = 0.0;
       double pressure_sensor_stddev_ = 0.002; // in bar (2mm)
       std::default_random_engine generator_;
       std::normal_distribution<double> pressure_sensor_dist_; //{pressure_sensor_mean_, pressure_sensor_stddev_};
       double temperature_sensor_ = 0.;
       double temperature_sensor_stddev_ = 0.02; // in °C
       std::default_random_engine generator_temperature_;
       std::normal_distribution<double> temperature_sensor_dist_; //{0.0, temperature_sensor_stddev_};
       double temperature_sensor_coeff_ = 0.04;
   
       rclcpp::Time piston_last_time_ = rclcpp::Time(0., RCL_ROS_TIME);
       rclcpp::Duration piston_dt_ = 200ms; 
       double piston_position_ = 0.;
   
       rclcpp::Time temperature_last_time_ = rclcpp::Time(0., RCL_ROS_TIME);
       rclcpp::Duration temperature_dt_ = 200ms; 
   
       double fusion_depth_{}, fusion_velocity_{};
   
       DepthControl dc_;
       rclcpp::Time dc_last_time_ = rclcpp::Time(0., RCL_ROS_TIME);
       rclcpp::Duration dc_dt_ = 200ms; 
   
       std::vector<double> solver_velocity_, solver_alpha_;
   
       Mission mission_;
       rclcpp::Time mission_last_time_ = rclcpp::Time(0., RCL_ROS_TIME);
       rclcpp::Duration mission_dt_ = 1s; 
       string mission_file_name_ = "";
       string mission_path_ = "./";
       rclcpp::Duration mission_delay_before_start_ = 10s;
       rclcpp::Duration mission_delay_after_end_ = 120s;
   
       std::string bag_path_ = "./mission";
   
       std::vector<double> temperature_profile_depth_;
       std::vector<double> temperature_profile_temperature_;
   
       size_t index_center_thermocline_ = 0;
       double thermocline_depth_ = 0.0;
       std::vector<WaveGenerator> wave_generators_;
       string wave_file_name_ = "";
   
   
   };
   
   #endif //BUILD_SIMULATOR_H
