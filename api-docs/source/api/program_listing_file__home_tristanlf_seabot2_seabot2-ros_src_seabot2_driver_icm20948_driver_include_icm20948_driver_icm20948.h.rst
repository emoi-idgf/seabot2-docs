
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_include_icm20948_driver_icm20948.h:

Program Listing for File icm20948.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_include_icm20948_driver_icm20948.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/include/icm20948_driver/icm20948.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth
   //
   
   #ifndef BUILD_ICM20948_H
   #define BUILD_ICM20948_H
   
   #include <string>
   #include <array>
   #include <cstdint>
   
   #include "icm20948_register.h"
   
   using namespace std;
   
   class ICM20948 {
   public:
     ICM20948() = default;
   
     ~ICM20948();
   
     int init();
   
     int get_measure();
   
     double acc_x_ {}, acc_y_ {}, acc_z_ {};
     double gyro_x_ {}, gyro_y_ {}, gyro_z_ {};
     double mag_x_ {}, mag_y_ {}, mag_z_ {};
   
     float temp_ {};
   
     double accel_x_bias_ {}, accel_y_bias_ {}, accel_z_bias_ {};
   
     double mag_x_old_ {}, mag_y_old_ {}, mag_z_old_ {};
     int nb_not_update = 0;
   
     bool mag_data_has_been_skipped_ = false;
     bool mag_data_is_ready_ = false;
     bool mag_sensor_magnetic_overflow_ = false;
   
     uint8_t spi_bus_ = 0;
     uint8_t spi_device_ = 0;
   
     uint8_t gyro_dlpfcfg_ = GYRO_DLPFCFG_BW23HZ;
     uint8_t gyro_fs_sel_ = GYRO_FS_SEL_250DPS;
     uint8_t gyro_fchoice_ = 1;
     uint8_t gyro_avgcfg_ = GYRO_AVGCFG_128X;  // For low power mode only
     uint8_t gyro_smplrt_div_ = 21;  // => 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]) | 21 => 50Hz (ODR) | MAX 255
   
     uint8_t accel_smplrt_div_ = 21;  // 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]) => 1.125 kHz/(1+21) = 52.5Hz (ODR)
     uint8_t accel_dlpfcfg_ = ACCEL_DLPFCFG_BW50HZ;
     uint8_t accel_fs_sel_ = ACCEL_FS_SEL_4G;
     uint8_t accel_fchoice_ = 1;
     uint8_t acc_dec3Cfg_ = ACCEL_DEC3CFG_32X;  // For low power mode only
   
      // Interruption Wakeup On Move (WOM)
     uint8_t accel_intel_en_ = 0;  // Disable wake on motion
     uint8_t accel_intel_mode_int_ = 0;
     uint8_t accel_intel_thr_ = 0;
   
     const std::array < double,
     4 > accel_sensitivity_ = {2. / 32767., 4. / 32767., 8. / 32767., 16. / 32767.};
     const std::array < double,
     4 > gyro_sensitivity_ = {250. / 32767., 500. / 32767., 1000. / 32767., 2000. / 32767.};
     const std::array < double, 4 > gyro_full_scale_ = {250., 500., 1000., 2000.};
     const std::array < double, 4 > accel_full_scale_ = {2., 4., 8., 16.};
   
     const double mag_sensitivity_ = 4912.0 / 32752.0;
     const double mag_normalization_ = 1.0 / 50.0;  // mean value of the magnetic field
     AK09916_mode_e mag_mode_ = AK09916_mode_cont_50hz;
     uint8_t i2c_smplrt_div_ = 4;  // 1.1 kHz/(2^((odr_config[3:0])), 4=>68.75 // 1 ? || MAX = 15
   
     uint8_t slv4_dly_ = 0x00;  // 0x04
     uint8_t i2c_slv0_delay_en_ = 0x00;  // 0x01
   
     uint8_t i2c_frequency_clk_ = 0x07;  // 7 recommended
     uint8_t i2c_lp_mode_ = 0x00;  // 0x01
     uint8_t i2c_if_dis_ = 0x01;
   
   private:
     int spi_fd_ = 0;
     uint32_t speed_ = 6000000;
     int current_bank_ = -1;
   
     uint8_t cs_change_ = 0;
     uint16_t delay_usecs_ = 0;
   
   private:
     int open_spi(const unsigned int & spi_bus, const unsigned int & spi_device);
   
     std::vector < uint8_t > readRegister(const uint8_t & reg, const uint32_t & len = 1) const;
   
     int writeRegister(const uint8_t & reg, const std::vector < uint8_t > &data) const;
   
     int writeRegister(const uint8_t & reg, const uint8_t & data, bool check_write = true);
   
     static uint8_t imuJoinUserCtrl(
       const uint8_t dmpEn,
       const uint8_t fifoEn,
       const uint8_t i2cMstEn,
       const uint8_t i2cIfDis,
       const uint8_t dmpRst,
       const uint8_t sramRst,
       const uint8_t i2cMstRst);
   
     static uint8_t imuJoinLpConfig(
       const uint8_t i2cMstCycle,
       const uint8_t accelCycle,
       const uint8_t gyroCycle);
   
     static uint8_t imuJoinPwrMgmt1(
       const uint8_t deviceRst,
       const uint8_t sleep,
       const uint8_t lpEn,
       const uint8_t tempDis,
       const uint8_t clkSel);
   
     static uint8_t imuJoinPwrMgmt2(
       const uint8_t disableAccel,
       const uint8_t disableGyro);
   
     static uint8_t joinGyroSmplrtDiv(const uint8_t gyroSmplrtDiv);
   
     static uint8_t imuJoinGyroConfig1(
       const uint8_t gyroDlpfCfg,
       const uint8_t gyroFsSel,
       const uint8_t gyroFchoice);
   
     static uint8_t imuJoinGyroConfig2(
       const uint8_t xGyroCten,
       const uint8_t yGyroCten,
       const uint8_t zGyroCten,
       const uint8_t gyroAvgcfg);
   
     static uint8_t imuJoinAccelSmplrtDiv1(const uint16_t accelSmplrtDiv);
   
     static uint8_t imuJoinAccelSmplrtDiv2(const uint16_t accelSmplrtDiv);
   
     static uint8_t imuJoinAccelIntelCtrl(
       const uint8_t accelIntelEn,
       const uint8_t accelIntelModeInt);
   
   
     static uint8_t imuJoinAccelConfig(
       const uint8_t accelDlpfcfg,
       const uint8_t accelFsSel,
       const uint8_t accelFchoice);
   
     static uint8_t imuJoinAccelConfig2(
       const uint8_t axStEnReg,
       const uint8_t ayStEnReg,
       const uint8_t azStEnReg,
       const uint8_t dec3Cfg);
   
     static uint8_t imuJoinBankId(const uint8_t bank_id);
   
     static uint8_t imuJoinMstCtrl(
       const uint8_t multMstEn,
       const uint8_t i2cMstPNsr,
       const uint8_t i2cMstClk);
   
     static uint8_t imuJoinIntEnable(
       const uint8_t regWofEn,
       const uint8_t womIntEn,
       const uint8_t pllRdyEn,
       const uint8_t dmpInt1En,
       const uint8_t i2cMstIntEn);
   
     static uint8_t imuJoinI2cMstDelayCtrl(
       const uint8_t delayEsShadow,
       const uint8_t i2cSlv4DelayEn,
       const uint8_t i2cSlv3DelayEn,
       const uint8_t i2cSlv2DelayEn,
       const uint8_t i2cSlv1DelayEn,
       const uint8_t i2cSlv0DelayEn);
   
     static uint8_t imuJoinI2cSlv4Ctrl(
       const uint8_t i2cSlv4En,
       const uint8_t i2cSlv4IntEn,
       const uint8_t i2cSlv4RegDis,
       const uint8_t i2cSlv4Dly);
   
     static uint8_t imuJoinIntPinCfg(
       const uint8_t int1Actl,
       const uint8_t int1Open,
       const uint8_t int1LatchEn,
       const uint8_t int1Anyrd2Clear,
       const uint8_t int1FsyncLvl,
       const uint8_t int1FsyncEn,
       const uint8_t int1BypassEn);
   
     void dump_bank(const int & bank_id);
   
     void dump_ak09916();
   
     void dump_icm20948();
   
     int reset_i2c_slv4();
   
     int reset_i2c();
   
     int select_bank(const uint8_t & bank_id);
   
     int i2c_master_reset();
   
     bool test_who_i_am_mag();
   
     int set_mag_mode(const AK09916_mode_e & mode);
   
     int init_magnetometer();
   
   private:
     int write_i2c_AK09916_byte(const uint8_t & reg, const uint8_t & data);
   
     int read_i2c_AK09916_byte(const uint8_t & reg, uint8_t & data);
   };
   
   #endif //BUILD_ICM20948_H
