
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_audio_recorder_include_seabot2_audio_recorder_dspic_acoustic.h:

Program Listing for File dspic_acoustic.h
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_audio_recorder_include_seabot2_audio_recorder_dspic_acoustic.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_audio_recorder/include/seabot2_audio_recorder/dspic_acoustic.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef DSPIC_ACOUSTIC_H
   #define DSPIC_ACOUSTIC_H
   
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
   
   class DspicAcoustic {
   public:
     explicit DspicAcoustic(rclcpp::Node *n)
     {
       n_ = n;
     }
   
     ~DspicAcoustic();
   
     int i2c_open();
   
     void wait_recompute_signal() const;
   
     [[nodiscard]] int getI2CAddr() const;
   
     void setI2CAddr(int i2CAddr);
   
     [[nodiscard]] const std::string & getI2CPeriph() const;
   
     void setI2CPeriph(const std::string & i2CPeriph);
   
     void sync_pps() const;
   
     [[nodiscard]] int enable_chirp(bool enable = true) const;
   
     [[nodiscard]] int set_duration_between_shoot(uint16_t duration_seconds) const;
   
     [[nodiscard]] int set_shoot_offset_from_posix_zero(uint16_t offset_seconds) const;
   
     [[nodiscard]] int set_robot_data(const int & data_size, const uint64_t & data) const;
   
     [[nodiscard]] int recompute_chirp(
       const uint16_t & frequency_middle,
       const uint16_t & frequency_range,
       const uint8_t & signal_function) const;
   
     int get_frequency_middle() const;
   
     int get_frequency_range() const;
   
     int get_signal_function() const;
   
     [[nodiscard]] int set_robot_code(const uint8_t & robot_code) const;
   
     [[nodiscard]] int get_robot_code() const;
   
     [[nodiscard]] uint32_t get_posix_time() const;
   
     [[nodiscard]] uint16_t get_signal_number() const;
   
   private:
     rclcpp::Node * n_ = nullptr;  
   
     int file_ = 0;   
     std::string i2c_periph_ = "/dev/i2c-0";
     int i2c_addr_ = 0x1A;
     int code_version = 0x05;
   };
   
   #endif // DSPIC_ACOUSTIC_H
