
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_wtf_include_seabot2_wtf_wtf_node.hpp:

Program Listing for File wtf_node.hpp
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_wtf_include_seabot2_wtf_wtf_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_wtf/include/seabot2_wtf/wtf_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_WTF_NODE_HPP
   #define BUILD_WTF_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include <ncurses.h>
   
   #include "seabot2_msgs/msg/depth_pose.hpp"
   #include "seabot2_msgs/msg/bme280_data.hpp"
   #include "seabot2_msgs/msg/power_state.hpp"
   #include "seabot2_msgs/msg/safety_status2.hpp"
   #include "seabot2_msgs/msg/piston_state.hpp"
   #include "seabot2_msgs/msg/mission_state.hpp"
   #include "seabot2_msgs/msg/depth_control_set_point.hpp"
   #include "seabot2_msgs/msg/depth_control_debug.hpp"
   #include "seabot2_msgs/msg/gps_fix.hpp"
   #include "seabot2_msgs/msg/profile.hpp"
   #include "seabot2_msgs/msg/density.hpp"
   #include "seabot2_msgs/msg/temperature_sensor_data.hpp"
   #include "seabot2_msgs/msg/sync_dspic.hpp"
   
   #include "rcl_interfaces/msg/log.hpp"
   #include <deque>
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class WtfNode final : public rclcpp::Node {
   public:
     WtfNode();
     ~WtfNode() override;
   
   private:
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 100ms;   
   
     string hostname_ = "Seabot";
   
     const array<std::string, 4> gpsd_mode_string_ = {"NOT_SEEN",
       "NO_FIX",
       "2D",
       "3D"};
     const array<std::string, 6> piston_state_string_ = {"SEARCH_SWITCH_BOTTOM",
       "RELEASE_SWITCH_BOTTOM",
       "BACK_SWITCH_BOTTOM",
       "REGULATION",
       "EXIT",
       "BATT_LOW"};
     const array<std::string, 6> power_state_string_ = {"IDLE",
       "MEASURE_VOLTAGE",
       "WIRE_DETECTION",
       "POWER_ON",
       "WAIT_TO_SLEEP",
       "SLEEP"};
   
     const array<std::string, 8> depth_control_string_ = {"SURFACE",
       "SINK",
       "CONTROL",
       "EMERGENCY",
       "PISTON_ISSUE",
       "HOLD_DEPTH"};
   
     const array<std::string, 8> mission_mode_string_ = {"IDLE",
       "DEPTH_CONTROL",
       "SEAFLOOR_LANDING",
       "TEMPERATURE_KEEPING",
       "TEMPERATURE_PROFILE",
       "GNSS_PROFILE"};
     const array<std::string, 8> mission_state_string_ = {"NOT_STARTED",
       "RUNNING",
       "ENDING",
       "NO_WP"};
   
     WINDOW *windows_robot_;
     WINDOW *windows_safety_;
     WINDOW *windows_internal_pressure_;
     WINDOW *windows_depth_;
     WINDOW *windows_power_;
     WINDOW *windows_depth_control_;
     WINDOW *windows_piston_;
     WINDOW *windows_mission_;
     WINDOW *windows_gnss_;
     WINDOW *windows_sensors_;
     WINDOW *windows_log_;
     WINDOW *windows_audio_;
     WINDOW *windows_com_;
   
     int windows_max_y_{}, windows_max_x_{};
     int windows_default_y_ = 4;
     int windows_current_y_ = windows_default_y_, windows_current_x_{};
     int windows_width_max_{};
   
   
     seabot2_msgs::msg::SafetyStatus2 msg_safety_;
     seabot2_msgs::msg::DepthPose msg_depth_data_;
     seabot2_msgs::msg::Bme280Data msg_internal_sensor_filter_;
     seabot2_msgs::msg::PowerState msg_power_data_;
     seabot2_msgs::msg::PistonState msg_piston_data_;
     seabot2_msgs::msg::DepthControlSetPoint msg_depth_control_set_point_;
     seabot2_msgs::msg::MissionState msg_mission_state_;
     seabot2_msgs::msg::DepthControlDebug msg_depth_control_;
     seabot2_msgs::msg::GpsFix msg_gnss_;
     seabot2_msgs::msg::Profile msg_profile_;
     seabot2_msgs::msg::Density msg_density_;
     seabot2_msgs::msg::TemperatureSensorData msg_temperature_sensor_data_;
     seabot2_msgs::msg::SyncDspic msg_audio_dspic_;
     deque<rcl_interfaces::msg::Log> msg_queue_log_;
   
     rclcpp::Time time_last_safety_ = this->now();
     rclcpp::Time time_last_depth_data_ = this->now();
     rclcpp::Time time_last_internal_sensor_filter_ = this->now();
     rclcpp::Time time_last_power_data_ = this->now();
     rclcpp::Time time_last_piston_data_ = this->now();
     rclcpp::Time time_last_mission_state_ = this->now();
     rclcpp::Time time_last_depth_control_set_point_ = this->now();
     rclcpp::Time time_last_depth_control_ = this->now();
     rclcpp::Time time_last_gnss_ = this->now();
     rclcpp::Time time_last_profile_ = this->now();
     rclcpp::Time time_last_density_ = this->now();
     rclcpp::Time time_last_temperature_sensor_data_ = this->now();
     rclcpp::Time time_last_audio_dspic_ = this->now();
   
     bool msg_first_received_safety_ = false;
     bool msg_first_received_depth_data_ = false;
     bool msg_first_received_internal_sensor_filter_ = false;
     bool msg_first_received_power_data_ = false;
     bool msg_first_received_piston_data_ = false;
     bool msg_first_received_mission_state_ = false;
     bool msg_first_received_depth_control_set_point = false;
     bool msg_first_received_depth_control_ = false;
     bool msg_first_received_gnss_ = false;
     bool msg_first_received_profile_ = false;
     bool msg_first_received_density_ = false;
     bool msg_first_received_temperature_sensor_data_ = false;
     bool msg_first_received_audio_dspic_ = false;
     size_t msg_queue_log_size_ = 5;
   
     rclcpp::Subscription<seabot2_msgs::msg::SafetyStatus2>::SharedPtr subscriber_safety_;
     rclcpp::Subscription<seabot2_msgs::msg::DepthPose>::SharedPtr subscriber_depth_data_;
     rclcpp::Subscription<seabot2_msgs::msg::Bme280Data>::SharedPtr subscriber_internal_sensor_filter_;
     rclcpp::Subscription<seabot2_msgs::msg::PowerState>::SharedPtr subscriber_power_data_;
     rclcpp::Subscription<seabot2_msgs::msg::PistonState>::SharedPtr subscriber_piston_data_;
     rclcpp::Subscription<seabot2_msgs::msg::MissionState>::SharedPtr subscriber_mission_state_;
     rclcpp::Subscription<seabot2_msgs::msg::DepthControlSetPoint>::SharedPtr
       subscriber_depth_control_set_point_;
     rclcpp::Subscription<seabot2_msgs::msg::DepthControlDebug>::SharedPtr subscriber_control_debug_;
     rclcpp::Subscription<seabot2_msgs::msg::GpsFix>::SharedPtr subscriber_gnss_;
     rclcpp::Subscription<seabot2_msgs::msg::Profile>::SharedPtr subscriber_profile_;
     rclcpp::Subscription<seabot2_msgs::msg::Density>::SharedPtr subscriber_density_;
     rclcpp::Subscription<seabot2_msgs::msg::TemperatureSensorData>::SharedPtr
       subscriber_temperature_sensor_data_;
     rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscriber_log_;
     rclcpp::Subscription<seabot2_msgs::msg::SyncDspic>::SharedPtr subscriber_audio_dspic_;
   
     void init_parameters();
   
     void init_interfaces();
   
     void timer_callback();
   
     void depth_callback(const seabot2_msgs::msg::DepthPose & msg);
   
     void internal_sensor_callback(const seabot2_msgs::msg::Bme280Data & msg);
   
     void power_callback(const seabot2_msgs::msg::PowerState & msg);
   
     void piston_callback(const seabot2_msgs::msg::PistonState & msg);
   
     void safety_callback(const seabot2_msgs::msg::SafetyStatus2 & msg);
   
     void mission_state_callback(const seabot2_msgs::msg::MissionState & msg);
   
     void depth_control_set_point_callback(const seabot2_msgs::msg::DepthControlSetPoint & msg);
   
     void depth_control_callback(const seabot2_msgs::msg::DepthControlDebug & msg);
   
     void gnss_callback(const seabot2_msgs::msg::GpsFix & msg);
   
     void profile_callback(const seabot2_msgs::msg::Profile & msg);
   
     void density_callback(const seabot2_msgs::msg::Density & msg);
   
     void temperature_sensor_data_callback(const seabot2_msgs::msg::TemperatureSensorData & msg);
   
     void log_callback(const rcl_interfaces::msg::Log & msg);
   
     void audio_dspic_callback(const seabot2_msgs::msg::SyncDspic & msg);
   
     void update_safety_windows();
   
     void update_mission_windows();
   
     void update_internal_pressure_windows();
   
     void update_power();
   
     void update_depth();
   
     void update_piston();
   
     void update_robot();
   
     void update_depth_control();
   
     void update_gnss();
   
     void update_sensors();
   
     void update_audio() const;
   
     void update_log() const;
   
     static std::string set_color_valid(WINDOW *w, bool valid, const std::string & text = "");
   
     static std::string get_bool_text(bool valid);
   
     WINDOW * create_new_sub_window(int height, int width, const string & name);
   
     void create_windows();
   
   };
   #endif //BUILD_WTF_NODE_HPP
