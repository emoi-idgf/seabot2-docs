
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_iridium_driver_include_seabot2_iridium_driver_sbd_log_data.h:

Program Listing for File log_data.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_iridium_driver_include_seabot2_iridium_driver_sbd_log_data.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_iridium_driver/include/seabot2_iridium_driver/sbd/log_data.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef LOGDATA_H
   #define LOGDATA_H
   
   #include <string>
   #include <vector>
   #include <boost/multiprecision/cpp_int.hpp>
   
   using boost::multiprecision::cpp_int;
   
   #define WGS84_LAT_MIN (-90.0)
   #define WGS84_LAT_MAX 90.0
   #define WGS84_LON_MIN (-180.0)
   #define WGS84_LON_MAX 180.0
   
   //#define REF_POSIX_TIME 1604874973 //To be update every 5 years !
   #define BATT_MIN 12.0
   #define BATT_MAX 16.8
   
   //enum MSG_TYPE:unsigned int {LOG_STATE=0, CMD_SLEEP=1, CMD_PARAMETERS=2, CMD_MISSION_NEW=3, CMD_MISSION_KEEP=4};
   // First octet (4 bits) : message_id
   
   #define NB_BITS_LOG1 128
   // 16 bytes
   typedef boost::multiprecision::number < boost::multiprecision::cpp_int_backend < NB_BITS_LOG1,
     NB_BITS_LOG1, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked,
     void >> uint_log1_t;
   
   #define NB_BITS_CMD_SLEEP 16
   typedef boost::multiprecision::number < boost::multiprecision::cpp_int_backend < NB_BITS_CMD_SLEEP,
     NB_BITS_CMD_SLEEP, boost::multiprecision::unsigned_magnitude, boost::multiprecision::unchecked,
     void >> uint_cmd_sleep_t;
   
   #define NB_BITS_CMD_PARAMETERS 16
   typedef boost::multiprecision::number < boost::multiprecision::cpp_int_backend <
     NB_BITS_CMD_PARAMETERS, NB_BITS_CMD_PARAMETERS, boost::multiprecision::unsigned_magnitude,
     boost::multiprecision::unchecked, void >> uint_cmd_parameters_t;
   
   #define NB_BITS_CMD_WAYPOINT_DEPTH 24
   typedef boost::multiprecision::number < boost::multiprecision::cpp_int_backend <
     NB_BITS_CMD_WAYPOINT_DEPTH, NB_BITS_CMD_WAYPOINT_DEPTH, boost::multiprecision::unsigned_magnitude,
     boost::multiprecision::unchecked, void >> uint_cmd_waypoint_depth_t;
   
   #define NB_BITS_CMD_WAYPOINT_TRAJ 40
   typedef boost::multiprecision::number < boost::multiprecision::cpp_int_backend <
     NB_BITS_CMD_WAYPOINT_TRAJ, NB_BITS_CMD_WAYPOINT_TRAJ, boost::multiprecision::unsigned_magnitude,
     boost::multiprecision::unchecked, void >> uint_cmd_waypoint_traj_t;
   
   #define NB_BITS_CMD_WAYPOINT_TYPE 8
   typedef boost::multiprecision::number < boost::multiprecision::cpp_int_backend <
     NB_BITS_CMD_WAYPOINT_TYPE, NB_BITS_CMD_WAYPOINT_TYPE, boost::multiprecision::unsigned_magnitude,
     boost::multiprecision::unchecked, void >> uint_cmd_waypoint_type_t;
   
   #define NB_BITS_CMD_MISSION_HEADER 64
   typedef boost::multiprecision::number < boost::multiprecision::cpp_int_backend <
     NB_BITS_CMD_MISSION_HEADER, NB_BITS_CMD_MISSION_HEADER, boost::multiprecision::unsigned_magnitude,
     boost::multiprecision::unchecked, void >> uint_cmd_mission_header_t;
   
   class Waypoint {
   
   public:
     Waypoint() = default;
   
     Waypoint(const double & duration_param, const double & depth_param, const double & east_param,
       const double & north_param, const bool & enable_thrusters_param = false,
       const bool & seafloor_landing_param = false) {
           duration = duration_param;
           depth = depth_param;
           east = east_param;
           north = north_param;
           enable_thrusters = enable_thrusters_param;
           seafloor_landing = seafloor_landing_param;
     }
   public:
     double north = 0.0;
     double east = 0.0;
     double depth = 0.0;
     double duration = 0;
     bool enable_thrusters = false;
     bool seafloor_landing = false;
   };
   
   class LogData
   {
   public:
     LogData() = default;
   
     enum MSG_TYPE:unsigned int {LOG_STATE=0, CMD_SLEEP=1, CMD_PARAMETERS=2, CMD_MISSION_NEW=3,
       CMD_MISSION_KEEP=4};
   
   private:
     template < typename T >
     unsigned int serialize_data(
       T & bits, const unsigned int & nb_bit, const unsigned int & start_bit,
       const double & value, const double & value_min, const double & value_max);
   
     template < typename T >
     unsigned int deserialize_data(
       T & bits, const unsigned int & nb_bit,
       const unsigned int & start_bit, double & value, const double & value_min,
       const double & value_max);
   
     template < typename T >
     unsigned int serialize_data(
       T & bits, const unsigned int & nb_bit, const unsigned int & start_bit,
       const unsigned int & value);
   
     template < typename T >
     unsigned int deserialize_data(
       const T & bits, const unsigned int & nb_bit,
       const unsigned int & start_bit, unsigned int & value);
   
     template < typename T >
     unsigned int deserialize_data(
       const T & bits, const unsigned int & nb_bit,
       const unsigned int & start_bit, int & value);
   
     template < typename T >
     unsigned int deserialize_data(
       const T & bits, const unsigned int & nb_bit,
       const unsigned int & start_bit, bool & value);
   
   public:
   
     bool deserialize_log_CMD(const std::string & raw_data);
   
     std::string serialize_log_state(const long long & time);
   
     bool deserialize_log_CMD_sleep(const std::string & data);
   
     bool deserialize_log_CMD_mission(const std::string & data);
   
     bool deserialize_log_CMD_parameters(const std::string & message);
   
     static bool write_file(
       const std::string & file_name, const std::string & data,
       const unsigned int nb_bits = NB_BITS_LOG1);
   
     static std::string read_file(const std::string & file_name);
   
   private:
   
   //  /**
   //   * @brief serialize_log_CMD_waypoint
   //   * @param save_file
   //   * @param w
   //   * @return
   //   */
   //  std::string serialize_log_CMD_waypoint(const Waypoint &w);
   
     unsigned int deserialize_log_CMD_waypoint(
       const std::string & message,
       const unsigned int & bit_position);
   
   public:
     double time_now_ = 0.0;
   
     double gnss_speed_ = 4.2;
     double gnss_heading_ = 242.0;
     double battery_ = 10.0;
     double gnss_lat_ = 0.0;
     double gnss_long_ = 0.0;
   
     double gnss_mean_east_ = 42.0;
     double gnss_mean_north_ = 6000042.0;
     double gnss_mean_heading_ = 242.0;
   //    unsigned long start_time_ = REF_POSIX_TIME;
   
     double internal_pressure_ = 742.0;
     double internal_temperature_ = 42.0;
     double internal_humidity_ = 0.0;
   
     unsigned int  current_waypoint_ = 42;   // 0 to 255 max
   
     unsigned int sleep_time_ = 0;   // sleep time in min
   
     std::vector < Waypoint > waypoint_list_;
     double offset_east_ = 0.;
     double offset_north_ = 0.;
   //  long long offset_time_ = TIME_POSIX_START; // in sec
   
     bool enable_mission_ = true;
     bool enable_flash_ = true;
     bool enable_depth_ = true;
     bool enable_engine_ = true;
   
     unsigned int last_cmd_received_ = 0;
     unsigned int period_message_ = 15;   // in 10*min
   
     bool safety_global_safety_valid_ = false;
     bool safety_published_frequency_ = false;
     bool safety_depth_limit_ = false;
     bool safety_batteries_limit_ = false;
     bool safety_depressurization_ = false;
     bool safety_seafloor_ = false;
     bool safety_piston_ = false;
     bool safety_zero_depth_ = false;
   
     MSG_TYPE msg_type_;
   };
   
   template < typename T >
   unsigned int LogData::serialize_data(
     T & bits, const unsigned int & nb_bit,
     const unsigned int & start_bit, const double & value, const double & value_min,
     const double & value_max)
   {
     double scale = ((1 << nb_bit) - 1) / (value_max - value_min);
     double bit_max = (1 << nb_bit) - 1;
     long unsigned int v = static_cast < long unsigned int >
       (std::min(std::max(round((value - value_min) * scale), 0.0), bit_max));
     auto v_max = static_cast < long unsigned int > (bit_max);
     v = std::min(v, v_max);
   
     T mask = ((T(1) << nb_bit) - 1);
     bits &= ~(mask << start_bit);
     bits |= (T(v) & mask) << start_bit;
     return nb_bit;
   }
   
   template < typename T >
   unsigned int LogData::deserialize_data(
     T & bits, const unsigned int & nb_bit,
     const unsigned int & start_bit, double & value, const double & value_min,
     const double & value_max)
   {
     double scale = ((1 << nb_bit) - 1.0) / (value_max - value_min);
   
     T mask = ((T(1) << nb_bit) - 1) << start_bit;
     T v = (bits & mask) >> start_bit;
     value = static_cast < double > (v) / scale + value_min;
   
   //  std::cout << "---" << std::endl;
   //  std::cout << "scale = " << scale << std::endl;
   //  std::cout << "value min = " << value_min << std::endl;
   //  std::cout << "binary value = " << v << std::endl;
   //  std::cout << "scaled value = " << static_cast<double>(v)/scale << std::endl;
   //  std::cout << "value = " << value << std::endl << std::endl;
     return nb_bit;
   }
   
   template < typename T >
   unsigned int LogData::serialize_data(
     T & bits, const unsigned int & nb_bit,
     const unsigned int & start_bit, const unsigned int & value)
   {
     T mask = ((T(1) << nb_bit) - 1);
     bits &= ~(mask << start_bit);
     bits |= (T(value) & mask) << start_bit;
     return nb_bit;
   }
   
   template < typename T >
   unsigned int LogData::deserialize_data(
     const T & bits, const unsigned int & nb_bit,
     const unsigned int & start_bit, unsigned int & value)
   {
     T mask = ((T(1) << nb_bit) - 1) << start_bit;
     T v = (bits & mask) >> start_bit;
   
     value = static_cast < unsigned int > (v);
     return nb_bit;
   }
   
   template < typename T >
   unsigned int LogData::deserialize_data(
     const T & bits, const unsigned int & nb_bit,
     const unsigned int & start_bit, int & value)
   {
     T mask = ((T(1) << nb_bit) - 1) << start_bit;
     T v = (bits & mask) >> start_bit;
   
     value = static_cast < int > (v);// Two bit complement is automaticaly done
     return nb_bit;
   }
   
   template < typename T >
   unsigned int LogData::deserialize_data(
     const T & bits, const unsigned int & nb_bit,
     const unsigned int & start_bit, bool & value)
   {
     T mask = ((T(1) << nb_bit) - 1) << start_bit;
     T v = (bits & mask) >> start_bit;
   
     value = static_cast < bool > (v);
     return nb_bit;
   }
   
   #endif // LOGDATA_H
