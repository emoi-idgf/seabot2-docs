
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_power_driver_include_seabot2_power_driver_power.h:

Program Listing for File power.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_power_driver_include_seabot2_power_driver_power.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_power_driver/include/seabot2_power_driver/power.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef POWER_H
   #define POWER_H
   
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
   
   #define REGISTER_DATA_READ 0x00
   #define REGISTER_DATA_SIZE 11
   #define CONVERT_BRIDGE_BATTERY (3.3/1024.)
   #define CONVERT_BRIDGE_CURRENT (3.3/1024.)
   #define CONVERT_CURRENT_V_to_A (1./66000.)
   
   class Power
   {
   public:
       explicit Power(rclcpp::Node *n) {
           n_ = n;
       }
   
       ~Power();
   
       int i2c_open();
   
       uint8_t& get_version();
   
       int getI2CAddr() const;
   
       void setI2CAddr(int i2CAddr);
   
       const std::string &getI2CPeriph() const;
   
       void setI2CPeriph(const std::string &i2CPeriph);
   
   private:
       rclcpp::Node* n_= nullptr; 
   
       int file_ = 0; 
       std::string i2c_periph_ = "/dev/i2c-1";
       int i2c_addr_ = 0x39;
       const int code_version_ = 0x06; 
       uint8_t pic_code_version_ = 0; 
   
       std::array<double, 2> R1_= {180, 180};
       std::array<double, 2> R2_= {820, 820};
   
   public:
       std::array<float, 2> cell_volt_{};
       double battery_volt_ = 0.;
       std::array<float, 2> esc_current_{};
       float motor_current_ = 0.0;
       int power_state_=0;
   
   public:
   
       int get_all_data();
   
       int set_sleep();
   
   };
   
   #endif // POWER_H
