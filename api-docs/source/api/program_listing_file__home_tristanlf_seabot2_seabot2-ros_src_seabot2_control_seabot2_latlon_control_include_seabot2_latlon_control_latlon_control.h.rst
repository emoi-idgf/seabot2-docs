
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_latlon_control_include_seabot2_latlon_control_latlon_control.h:

Program Listing for File latlon_control.h
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_latlon_control_include_seabot2_latlon_control_latlon_control.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_latlon_control/include/seabot2_latlon_control/latlon_control.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_LATLON_CONTROL_H
   #define BUILD_LATLON_CONTROL_H
   
   
   class LatLonControl
   {
   public:
   
       // Setpoint variables
     double lat_set_point_;
     double lon_set_point_;
   
       // State variables
     double lat_;
     double lon_;
     float heading_;
   
       // Command variables
     float linear_velocity_;
     float angular_velocity_;
   
       // Control parameters
     float linear_velocity_max_ = 0.5;   // Maximum linear velocity
     float angular_velocity_max_ = 1.0;   // Maximum angular velocity
     float distance_threshold_slowing_down_ = 2;   // Threshold distance for stopping in m
   
     bool global_safety_valid_ = false;
   
   
     LatLonControl();
     ~LatLonControl();
   
     void update_lat_lon(const double & lat, const double & lon);
   
     void update_heading(const double & heading);
   
     void update_safety(const bool & global_safety_valid);
   
     void compute_control_commands();
   
     float get_linear_velocity() const;
   
     float get_angular_velocity() const;
   
     void inverse_geodesic(
       const double & lat1, const double & lon1,
       const double & lat2, const double & lon2,
       double & distance, double & heading);
   
   };
   
   
   #endif // BUILD_LATLON_CONTROL_H
