
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_light_driver_include_seabot2_light_driver_light.h:

Program Listing for File light.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_light_driver_include_seabot2_light_driver_light.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_light_driver/include/seabot2_light_driver/light.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef LIGHT_H
   #define LIGHT_H
   
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
   
   #define REGISTER_LIGHT_ENABLE 0x00
   #define REGISTER_LIGHT_POWER 0x01
   #define REGISTER_LIGHT_PATTERN 0x02
   #define CODE_VERSION 0xC0
   #define NB_PATTERN 10
   
   class Light
   {
   public:
       Light(rclcpp::Node *n){
           n_ = n;
       }
   
       ~Light();
   
       int i2c_open();
   
       uint8_t& get_version();
   
       int getI2CAddr() const;
   
       void setI2CAddr(int i2CAddr);
   
       const std::string &getI2CPeriph() const;
   
       void setI2CPeriph(const std::string &i2CPeriph);
   
   private:
       rclcpp::Node* n_= nullptr; 
   
       int file_ = 0; 
       std::string i2c_periph_ = "/dev/i2c-0";
       int i2c_addr_ = 0x28;
       const int code_version_ = 0x02; 
       uint8_t pic_code_version_=0; 
   
   public:
       int flash_duration_ = 1;
       int flash_pause_end_ = 40;
       int flash_pause_between_flash_ = 5;
   
   public:
   
       int set_light_enable(const bool &enable) const;
   
       bool get_light_enable() const;
   
       void set_power(const __u8 &val) const;
   
       void set_pattern(const std::array<__u8, NB_PATTERN> &pattern) const;
   
       void set_flash_number(const unsigned int &nb_flash) const;
   };
   
   #endif // LIGHT_H
