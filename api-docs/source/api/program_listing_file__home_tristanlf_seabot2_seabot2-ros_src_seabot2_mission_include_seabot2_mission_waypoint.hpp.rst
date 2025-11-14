
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_mission_include_seabot2_mission_waypoint.hpp:

Program Listing for File waypoint.hpp
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_mission_include_seabot2_mission_waypoint.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_mission/include/seabot2_mission/waypoint.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef SEABOT2_MISSION__WAYPOINT_HPP_
   #define SEABOT2_MISSION__WAYPOINT_HPP_
   
   #include <rclcpp/rclcpp.hpp>
   
   #include "seabot2_mission/mission.hpp"
   #include "seabot2_msgs/msg/depth_control_set_point.hpp"
   #include "seabot2_msgs/msg/mission_state.hpp"  // Used in the case of the shared library
   
   
   class Mission;
   
   class Waypoint{
   public:
     explicit Waypoint(Mission *mission)
     :time_end(0., RCL_STEADY_TIME)
     {
       mission_ = mission;
     }
   
     virtual ~Waypoint() = default;
   
     virtual void process(const rclcpp::Time & time) = 0;
   
     double velocity = 0.0;
     unsigned int mode = seabot2_msgs::msg::MissionState::MODE_IDLE;
     rclcpp::Time time_end;
     Mission *mission_ = nullptr;
   };
   
   class WaypointDepth final : public Waypoint {
   public:
     explicit WaypointDepth(Mission *mission)
     :Waypoint(mission) {}
     ~WaypointDepth() override = default;
   
     void process(const rclcpp::Time & time) override;
   
     double depth = 0.0;
   };
   
   
   class WaypointSeafloorLanding final : public Waypoint {
   public:
     explicit WaypointSeafloorLanding(Mission *mission)
     :Waypoint(mission) {}
     ~WaypointSeafloorLanding() override = default;
   
     void process(const rclcpp::Time & time) override;
   };
   
   class WaypointTemperatureKeeping final : public Waypoint {
   public:
     explicit WaypointTemperatureKeeping(Mission *mission)
     :Waypoint(mission) {}
     ~WaypointTemperatureKeeping() override = default;
   
     void process(const rclcpp::Time & time) override;
   
     double temperature_ = 20.0;
     double error_temperature_ = 0.0;
   };
   
   class WaypointTemperatureProfile final : public Waypoint {
   public:
     explicit WaypointTemperatureProfile(Mission *mission)
     :Waypoint(mission) {}
     ~WaypointTemperatureProfile() override = default;
   
     void process(const rclcpp::Time & time) override;
   
     double temperature_high_ = 20.0;
     double temperature_low_ = 20.0;
     double depth_min_ = 0.0;
     double depth_max_ = 0.0;
     rclcpp::Duration max_delay_ = rclcpp::Duration::from_seconds(100.0);
   
   private:
     rclcpp::Time time_last_transition_;
     enum PROFILE_STATE{PROFILE_GO_DOWN, PROFILE_GO_UP};
     PROFILE_STATE state_ = PROFILE_GO_DOWN;
     bool first_init_ = true;
   };
   
   class WaypointGNSSProfile final : public Waypoint {
   public:
     explicit WaypointGNSSProfile(Mission *mission)
     :Waypoint(mission) {}
   
     ~WaypointGNSSProfile() override = default;
   
     void process(const rclcpp::Time & time) override;
   
     // Northing setpoint in meter
     double north = 0.0;
     // Easting setpoint in meter
     double east = 0.0;
   };
   
   
   #endif  // SEABOT2_MISSION__WAYPOINT_HPP_
