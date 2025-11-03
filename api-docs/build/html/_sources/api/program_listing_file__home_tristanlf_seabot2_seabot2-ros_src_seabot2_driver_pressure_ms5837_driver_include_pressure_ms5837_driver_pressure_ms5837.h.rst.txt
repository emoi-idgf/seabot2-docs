
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_pressure_ms5837_driver_include_pressure_ms5837_driver_pressure_ms5837.h:

Program Listing for File pressure_ms5837.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_pressure_ms5837_driver_include_pressure_ms5837_driver_pressure_ms5837.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/pressure_ms5837_driver/include/pressure_ms5837_driver/pressure_ms5837.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef PRESSURE_H
   #define PRESSURE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <sys/types.h>
   #include <iostream>
   #include <fstream>
   #include <unistd.h>
   #include <fcntl.h>
   #include <deque>
   
   extern "C" {
   #include <linux/i2c-dev.h>
   #include <i2c/smbus.h>
   }
   
   #define CMD_RESET 0x1E 
   #define CMD_ADC_READ 0x00 
   
   #define CMD_ADC_CONV_D1_8192 0x4A 
   #define CMD_ADC_CONV_D2_8192 0x5A 
   
   #define CMD_PROM 0xA0 // Coefficient location
   
   class Pressure_ms5837
   {
   public:
       explicit Pressure_ms5837(rclcpp::Node *n):
           C_(),
           D1_(0),
           D2_(0) {
           n_ = n;
       }
   
       ~Pressure_ms5837();
   
       int init_sensor();
   
       bool measure();
   
       int reset() const;
   
       float get_pression() const;
   
       float get_temperature() const;
   
       bool compute();
   
       int getI2CAddr() const;
   
       void setI2CAddr(int i2CAddr);
   
       const std::string &getI2CPeriph() const;
   
       void setI2CPeriph(const std::string &i2CPeriph);
   
       void setCoefficient(const u_int16_t val, const long unsigned int index){
           if(index<C_.size())
               C_[index] = val;
       }
   
       void setD1(u_int32_t d1);
   
       void setD2(u_int32_t d2);
   
   private:
       int i2c_open();
       bool measure_D1();
       bool measure_D2();
   
       rclcpp::Node* n_= nullptr; 
   
       int file_ = 0; 
   
       int i2c_addr_ = 0x76; 
       std::string i2c_periph_ = "/dev/i2c-0";
   
   
       const float p_min_out_range_ = 0.7; 
       const float p_max_out_range_ = 30.0; 
       const float t_min_out_range_ = 0.0; 
       const float t_max_out_range_ = 80.0; 
   
       std::array<u_int16_t, 7>  C_; 
   
       u_int32_t D1_, D2_; 
   
       float pressure_ = 1.0; 
       float temperature_ = 10.0; // in degree
   
   };
   
   inline bool Pressure_ms5837::measure_D1(){
       i2c_smbus_write_byte(file_, CMD_ADC_CONV_D1_8192);
       usleep(20000);
       unsigned char buff[3] = {0, 0, 0};
       if (i2c_smbus_read_i2c_block_data(file_, CMD_ADC_READ, 3, buff) != 3){
           RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5837] Error Reading D1");
           return false;
       }
       D1_ = (buff[0] << 16) | (buff[1] << 8) | buff[2];
       return true;
   }
   
   inline bool Pressure_ms5837::measure_D2(){
       i2c_smbus_write_byte(file_, CMD_ADC_CONV_D2_8192);
       usleep(20000);
       unsigned char buff[3] = {0, 0, 0};
       if (i2c_smbus_read_i2c_block_data(file_, CMD_ADC_READ, 3, buff) != 3){
           RCLCPP_WARN(n_->get_logger(), "[Pressure_ms5837] Error Reading D2");
           return false;
       }
       D2_ = (buff[0] << 16) | (buff[1] << 8) | buff[2];
       return true;
   }
   
   inline float Pressure_ms5837::get_pression() const {
       return pressure_;
   }
   
   inline float Pressure_ms5837::get_temperature() const {
       return temperature_;
   }
   
   #endif // PRESSURE_H
