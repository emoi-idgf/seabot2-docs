
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_latlon_control_src_latlon_control.cpp:

Program Listing for File latlon_control.cpp
===========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_latlon_control_src_latlon_control.cpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_latlon_control/src/latlon_control.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "seabot2_latlon_control/latlon_control.h"
   
   #include <cmath>
   #include <GeographicLib/Geodesic.hpp>
   
   using namespace GeographicLib;
   
   
   double sawtooth(const double & x)
   {
       // Normalize x to the range [-pi, pi]
     return std::fmod(x + M_PI, 2 * M_PI) - M_PI;
   }
   
   LatLonControl::LatLonControl()
   {
   }
   
   LatLonControl::~LatLonControl()
   {
   }
   
   void LatLonControl::update_lat_lon(const double & lat, const double & lon)
   {
     lat_ = lat;
     lon_ = lon;
   }
   
   void LatLonControl::update_heading(const double & heading)
   {
     heading_ = heading;
   }
   
   void LatLonControl::update_safety(const bool & global_safety_valid)
   {
     global_safety_valid_ = global_safety_valid;
   }
   
   void LatLonControl::compute_control_commands()
   {
   
     if (!global_safety_valid_) {
       linear_velocity_ = 0.0;
       angular_velocity_ = 0.0;
       return;     // Safety is engaged, stop all movement
     }
   
     double distance, heading;
     inverse_geodesic(lat_, lon_, lat_set_point_, lon_set_point_, distance, heading);
   
     linear_velocity_ = linear_velocity_max_ *
       std::tanh(distance * std::atanh(0.99) * 1. / distance_threshold_slowing_down_);
   
     angular_velocity_ = angular_velocity_max_ * 1. / M_PI * sawtooth(heading - heading_);
   
   }
   
   float LatLonControl::get_linear_velocity() const
   {
     return linear_velocity_;
   }
   
   float LatLonControl::get_angular_velocity() const
   {
     return angular_velocity_;
   }
   
   void LatLonControl::inverse_geodesic(
     const double & lat1, const double & lon1,
     const double & lat2, const double & lon2,
     double & distance, double & heading)
   {
   
     double s12, azi1, azi2;
     Geodesic::WGS84().Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
   
     distance = s12;   // Distance in meters
     heading = azi1;   // Initial heading in degrees
   }
