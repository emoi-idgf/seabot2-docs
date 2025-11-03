
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_temperature_tsys01_driver_include_temperature_tsys01_driver_temperature_tsys01.hpp:

Program Listing for File temperature_tsys01.hpp
===============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_temperature_tsys01_driver_include_temperature_tsys01_driver_temperature_tsys01.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/temperature_tsys01_driver/include/temperature_tsys01_driver/temperature_tsys01.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef TEMPERATURE_H
   #define TEMPERATURE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <sys/types.h>
   #include <fstream>
   
   extern "C" {
   #include <linux/i2c-dev.h>
   #include <i2c/smbus.h>
   }
   
   #define CMD_RESET 0x1E // reset command
   #define CMD_ADC_READ 0x00 // ADC read command
   #define CMD_ADC_CONV 0x48 // ADC conversion command
   #define CMD_PROM 0xA0 // Coefficient location
   
   #define CONVERSION_TIME 10000 // 10ms
   
   class Temperature_TSYS01
   {
   public:
       Temperature_TSYS01(rclcpp::Node *n): k_{} {
           n_ = n;
       }
   
       ~Temperature_TSYS01();
   
       int i2c_open();
       int init_sensor();
       int reset() const;
   
       bool measure();
   
       double get_temperature() const;
   
       int getI2CAddr() const;
   
       void setI2CAddr(int i2CAddr);
   
       const std::string &getI2CPeriph() const;
   
       void setI2CPeriph(const std::string &i2CPeriph);
   
   private:
   
       rclcpp::Node* n_= nullptr; 
   
       int file_ = 0;
       int i2c_addr_ = 0x77;
       std::string i2c_periph_ = "/dev/i2c-1";
   
       u_int16_t k_[5];
       bool valid_data_ = false;
   
       double temperature_ = 0.;
   
   };
   
   inline double Temperature_TSYS01::get_temperature() const {
       return temperature_;
   }
   
   #endif // TEMPERATURE_H
