
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_temperature_profile_include_seabot2_temperature_profile_temperature_profile.h:

Program Listing for File temperature_profile.h
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_temperature_profile_include_seabot2_temperature_profile_temperature_profile.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_temperature_profile/include/seabot2_temperature_profile/temperature_profile.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 12/11/22.
   //
   
   #ifndef BUILD_TEMPERATURE_PROFILE_H
   #define BUILD_TEMPERATURE_PROFILE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <eigen3/Eigen/Dense>
   #include <cmath>
   #include <deque>
   
   using namespace std;
   using namespace Eigen;
   
   class TemperatureProfile {
   public:
     TemperatureProfile();
   
   private:
   
     std::deque < std::pair < double, double >> temperature_depth_data_;
   
   public:
   
     bool enable_kalman_ = true;
     bool is_valid_ = true;
     rclcpp::Time time_last_predict_;
     std::chrono::milliseconds forecast_dt_ = 0ms;
     size_t max_number_data_ = 100;
   
     double profile_slope_ = -0.3;   // m/°C
     double profile_intercept_ = -5.0;   // m
   
   private:
   
   public:
     void update_temperature(double temperature, double depth);
   
     void compute_profile();
   };
   
   
   #endif //BUILD_TEMPERATURE_PROFILE_H
