
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_audio_recorder_include_seabot2_audio_recorder_tlv320adc.h:

Program Listing for File tlv320adc.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_audio_recorder_include_seabot2_audio_recorder_tlv320adc.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_audio_recorder/include/seabot2_audio_recorder/tlv320adc.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef TLV320ADC_H
   #define TLV320ADC_H
   
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
   
   class TLV320ADC
   {
   public:
     TLV320ADC(rclcpp::Node * n) {
           n_ = n;
     }
   
     ~TLV320ADC();
   
     int i2c_open();
   
     int configure(uint8_t gain) const;
   
     int set_channel_parameters() const;
   
     int set_power_up() const;
   
     int getI2CAddr() const;
   
     void setI2CAddr(int i2CAddr);
   
     const std::string & getI2CPeriph() const;
   
     void setI2CPeriph(const std::string & i2CPeriph);
   
       /*
        * gain values: 0dB=0x00, 10dB=0x28, 20dB=0x50, 30dB=78, 40dB=A0
        */
     enum class AdcGain
     {
       GAIN_0dB = 0,
       GAIN_10dB = 0x28,
       GAIN_20dB = 0x50,
       GAIN_30dB = 0x78,
       GAIN_40dB = 0xA0,
     };
   
     int set_adc_gain(uint8_t gain_ch1, uint8_t gain_ch2) const;
   
     int set_wake_up() const;
   
     int set_reset() const;
   
     int set_page(uint8_t page) const;
   
     int set_i2s_32bit() const;
   
     int set_channel_slot() const;
   
     int set_channel_enable() const;
   
     int set_enable_channel() const;
   
     int set_enable_input_ch1(uint8_t ch1, uint8_t ch2) const;
   
     int set_enable_input(uint8_t ch1, uint8_t ch2) const;
   
   private:
     rclcpp::Node * n_ = nullptr; 
   
     int file_ = 0;   
     std::string i2c_periph_ = "/dev/i2c-0";
     int i2c_addr_ = 0x4E;
   
   public:
   
   };
   
   #endif // TLV320ADC_H
