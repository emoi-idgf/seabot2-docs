
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_thruster_driver_include_seabot2_thruster_driver_thruster.h:

Program Listing for File thruster.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_thruster_driver_include_seabot2_thruster_driver_thruster.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_thruster_driver/include/seabot2_thruster_driver/thruster.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef THRUSTER_H
   #define THRUSTER_H
   
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
   
   class Thruster
   {
   public:
     static const uint8_t MOTOR_PWM_STOP = 150;
     static const uint8_t MAX_PWM = 190;
     static const uint8_t MIN_PWM = 110;
     static const uint8_t ENABLE_MOTOR = 0x10;
   
   public:
     Thruster(rclcpp::Node * n) {
           n_ = n;
           i2c_open();
   
     }
   
     ~Thruster();
   
     int i2c_open();
   
     int write_cmd(const uint8_t & left, const uint8_t & right) const;
   
     void write_enable_motors(bool enable);
   
     uint8_t & get_version();
   
     int getI2CAddr() const;
   
     void setI2CAddr(int i2CAddr);
   
     const std::string & getI2CPeriph() const;
   
     void setI2CPeriph(const std::string & i2CPeriph);
   
   private:
     rclcpp::Node * n_ = nullptr; 
   
     int file_ = 0;   
     int i2c_addr_ = 0x20;
     const int code_version_ = 0x01;   
     uint8_t pic_code_version_ = 0; 
   
   private:
     std::string i2c_periph_ = "/dev/i2c-1";
   
     bool reverse_thruster_order_ = false;   // not used ?
   
   
   };
   
   #endif // THRUSTER_H
