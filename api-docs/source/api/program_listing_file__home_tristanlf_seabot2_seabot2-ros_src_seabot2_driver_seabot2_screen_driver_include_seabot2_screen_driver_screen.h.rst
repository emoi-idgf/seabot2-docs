
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_screen_driver_include_seabot2_screen_driver_screen.h:

Program Listing for File screen.h
=================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_screen_driver_include_seabot2_screen_driver_screen.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_screen_driver/include/seabot2_screen_driver/screen.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef SCREEN_H
   #define SCREEN_H
   
   #include <rclcpp/rclcpp.hpp>
   
   #include <sys/types.h>
   #include <iostream>
   #include <fstream>
   #include <unistd.h>
   #include <fcntl.h>
   
   extern "C" {
   #include <linux/i2c-dev.h>
   #include <i2c/smbus.h>
   }
   
   #define REGISTER_WRITE_SCREEN 0
   #define REGISTER_ROBOT_NAME 1
   #define REGISTER_IP 2
   #define REGISTER_PRESSURE 3
   #define REGISTER_TEMPERATURE 4
   #define REGISTER_VOLTAGE 5
   #define REGISTER_HYGRO 6
   #define REGISTER_MISSION_NAME 7
   #define REGISTER_WAYPOINT_ID 9
   #define REGISTER_NB_WAYPOINT 10
   #define REGISTER_TIME 11
   #define REGISTER_TIME_REMAINING 12
   #define REGISTER_STATUS 13
   #define REGISTER_CODE_VERSION 14
   
   #define SCREEN_MISSION_NAME_SIZE 14
   #define SCREEN_ROBOT_NAME_SIZE 14
   
   #define DELAY_SLEEP_US 1000
   
   class Screen
   {
   public:
     Screen(rclcpp::Node * n) {
           n_ = n;
     }
   
     ~Screen();
   
     int i2c_open();
   
     void write_ip(const std::array < unsigned char, 4 > &data);
   
     void write_pressure(const short & pressure);
   
     void write_temperature(const short & temperature);
   
     void write_hygro(const short & hygro);
   
     void write_voltage(const char & volt);
   
     void write_robot_name(const std::string & name);
   
     void write_mission_name(const std::string & mission_name);
   
     void write_current_waypoint(const unsigned char & wp_id);
   
     void write_number_waypoints(const unsigned char & id_max);
   
     void write_time(const char & hour, const char & minute);
   
     void write_remaining_time(const char & minute, const char & second);
   
     enum Robot_Status { ERROR=0, WARNING=1, OK=3 };
   
     void write_robot_status(const Robot_Status & status);
   
     void write_screen();
   
     [[nodiscard]] int getI2CAddr() const;
   
     void setI2CAddr(int i2CAddr);
   
     [[nodiscard]] const std::string & getI2CPeriph() const;
   
     void setI2CPeriph(const std::string & i2CPeriph);
   
   private:
     rclcpp::Node * n_ = nullptr; 
   
     int file_ = 0;   
     int i2c_addr_ = 0x3C;
   
     std::string i2c_periph_ = "/dev/i2c-0";
   
   };
   
   #endif // SCREEN_H
