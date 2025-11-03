
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_mission_include_seabot2_mission_mission.hpp:

Program Listing for File mission.hpp
====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_mission_include_seabot2_mission_mission.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_mission/include/seabot2_mission/mission.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef MISSION_H
   #define MISSION_H
   
   #include <rclcpp/rclcpp.hpp>
   #include "seabot2_mission/waypoint.hpp"
   #include "seabot2_msgs/msg/depth_control_set_point.hpp"
   #include "seabot2_msgs/msg/mission_state.hpp"
   
   #include <boost/property_tree/ptree.hpp>
   #include <string>
   #include <vector>
   #include <filesystem>
   
   namespace fs = std::filesystem;
   
   class Waypoint;
   class WaypointDepth;
   class WaypointSeafloorLanding;
   class WaypointTemperatureKeeping;
   class WaypointTemperatureProfile;
   class WaypointGnssProfile;
   class WaypointStop;
   
   
   class Mission
   {
   public:
       Mission()= default;
   
       bool update_state(const rclcpp::Time &t_now);
   
       bool is_new_mission_file(const std::string &file_xml, const std::string &folder_path);
   
       int load_mission(const std::string &file_xml, const std::string &folder_path="",
                        const rclcpp::Time &start_time_if_not_given=rclcpp::Time(0., RCL_STEADY_TIME));
   
       [[nodiscard]] bool is_mission_enable() const;
   
       std::vector<float> get_velocity_list() const;
   
       [[nodiscard]] rclcpp::Time get_start_time() const{
           return time_start_;
       }
   
       [[nodiscard]] rclcpp::Time get_end_time() const{
           return time_end_;
       }
   
       [[nodiscard]] size_t get_number_waypoints() const{
           return waypoints_.size();
       }
   
       [[nodiscard]]  size_t get_current_waypoint_id() const{
           return current_waypoint_id_;
       }
   
       [[nodiscard]] double get_time_to_next_waypoint() const{
           return duration_next_waypoint_.seconds();
       }
   
       void set_limit_velocity_default(const double &vel){
           limit_velocity_default_ = vel;
       }
   
       [[nodiscard]] unsigned int get_mission_mode() const{
           return mission_mode_;
       }
   
       [[nodiscard]] unsigned int get_mission_state() const{
           return mission_state_;
       }
   
       void update_depth(const double &depth){
           depth_ = depth;
       }
   
       void update_temperature(const double &temperature){
           temperature_ = temperature;
       }
   
       void update_temperature_profile(const double &temp_slope, const double &temp_intercept){
           temperature_slope_ = temp_slope;
           temperature_intercept_ = temp_intercept;
       }
   
       seabot2_msgs::msg::DepthControlSetPoint& get_depth_control_set_point(){
           return dc_msg_;
       }
   
       void idle_state_configuration(const rclcpp::Time &t_now);
   
       double get_temperature() const{
           return temperature_;
       }
   
       [[nodiscard]] double get_temp_slope() const{
           return temperature_slope_;
       }
   
       [[nodiscard]] double get_temp_intercept() const{
           return temperature_intercept_;
       }
   
   private:
       void decode_waypoint(const std::shared_ptr<Waypoint> &w, const boost::property_tree::ptree::value_type &v, rclcpp::Time &last_time) const;
   
       void decode_waypoint_depth(const std::shared_ptr<WaypointDepth> &w, const boost::property_tree::ptree::value_type &v, const double &depth_offset) const;
   
       void decode_waypoint_temperature_keeping(const std::shared_ptr<WaypointTemperatureKeeping>& w, const boost::property_tree::ptree::value_type &v) const;
   
       void decode_waypoint_temperature_profile(const std::shared_ptr<WaypointTemperatureProfile>& w, const boost::property_tree::ptree::value_type &v) const;
   
       int decode_paths(boost::property_tree::ptree::value_type &v, rclcpp::Time &last_time, const double &depth_offset);
   
   public:
       enum WAYPOINT_TYPE:unsigned int {WP_IDLE=0,
           WP_DEPTH=1,
           WP_SEAFLOOR_LANDING=2,
           WP_TEMPERATURE_KEEPING=3,
           WP_TEMPERATURE_PROFILE=4,
           WP_GNSS_PROFILE=5
       };
   private:
       std::string file_name_ = "mission_empty.xml";
   
       std::vector<std::pair<std::shared_ptr<Waypoint>, WAYPOINT_TYPE>> waypoints_;
   
       size_t current_waypoint_id_ = 0;
       bool is_first_waypoint_ = true;
       bool mission_enable_ = false;
       rclcpp::Duration duration_next_waypoint_ = rclcpp::Duration::from_seconds(0.);
   
       std::filesystem::file_time_type file_time_;
   
       rclcpp::Time time_start_ = rclcpp::Time(0., RCL_STEADY_TIME);
       rclcpp::Time time_end_ = rclcpp::Time(0., RCL_STEADY_TIME);
       double offset_north_ = 0.0;
       double offset_east_ = 0.0;
       double limit_velocity_default_ = 0.02;
   
       double default_time_to_start_ = 60.0;
   
       // Temperature profile
       double temperature_slope_ = -0.2;
       double temperature_intercept_ = 5.0;
   
       // Mission mode
       unsigned int mission_mode_ = seabot2_msgs::msg::MissionState::MODE_IDLE;
   
       // Mission state
       enum MISSION_STATE:unsigned int {NOT_STARTED=0,
           RUNNING=1,
           ENDING=2,
           NO_WP=3};
       MISSION_STATE mission_state_ = NOT_STARTED;
   
       // Control messages
       seabot2_msgs::msg::DepthControlSetPoint dc_msg_;
   
       // Waypoint type
       const std::string XML_DEPTH = "waypoint_depth";
       const std::string XML_DEPTH_LEGACY = "waypoint";
       const std::string XML_SEAFLOOR_LANDING =   "seafloor_landing";
       const std::string XML_TEMPERATURE_KEEPING = "temperature_keeping";
       const std::string XML_TEMPERATURE_PROFILE = "temperature_profile";
       const std::string XML_GNSS_PROFILE =   "gnss_profile";
       const std::vector<std::string> XML_TYPE = {XML_DEPTH,
                                                  XML_DEPTH_LEGACY,
                                                  XML_SEAFLOOR_LANDING,
                                                  XML_TEMPERATURE_KEEPING,
                                                  XML_TEMPERATURE_PROFILE,
                                                  XML_GNSS_PROFILE
                                               };
   
       // State
       double depth_ = 0.0;
       double temperature_ = 15.0; // in degree
   
   public:
       double temperature_keeping_k_ = 0.03;
   
   public:
       bool is_current_waypoint_of_type(const WAYPOINT_TYPE &type) const;
   
       std::shared_ptr<WaypointTemperatureKeeping> get_current_waypoint_temperature_keeping() const;
   
       double get_depth() const {return depth_;}
   
   };
   
   
   #endif // MISSION_H
