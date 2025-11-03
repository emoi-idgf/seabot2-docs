
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionAhrs.h:

Program Listing for File FusionAhrs.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionAhrs.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/Fusion/FusionAhrs.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef FUSION_AHRS_H
   #define FUSION_AHRS_H
   
   //------------------------------------------------------------------------------
   // Includes
   
   #include "FusionConvention.h"
   #include "FusionMath.h"
   #include <stdbool.h>
   
   //------------------------------------------------------------------------------
   // Definitions
   
   typedef struct {
       FusionConvention convention;
       float gain;
       float gyroscopeRange;
       float accelerationRejection;
       float magneticRejection;
       unsigned int recoveryTriggerPeriod;
   } FusionAhrsSettings;
   
   typedef struct {
       FusionAhrsSettings settings;
       FusionQuaternion quaternion;
       FusionVector accelerometer;
       bool initialising;
       float rampedGain;
       float rampedGainStep;
       bool angularRateRecovery;
       FusionVector halfAccelerometerFeedback;
       FusionVector halfMagnetometerFeedback;
       bool accelerometerIgnored;
       int accelerationRecoveryTrigger;
       int accelerationRecoveryTimeout;
       bool magnetometerIgnored;
       int magneticRecoveryTrigger;
       int magneticRecoveryTimeout;
   } FusionAhrs;
   
   typedef struct {
       float accelerationError;
       bool accelerometerIgnored;
       float accelerationRecoveryTrigger;
       float magneticError;
       bool magnetometerIgnored;
       float magneticRecoveryTrigger;
   } FusionAhrsInternalStates;
   
   typedef struct {
       bool initialising;
       bool angularRateRecovery;
       bool accelerationRecovery;
       bool magneticRecovery;
   } FusionAhrsFlags;
   
   //------------------------------------------------------------------------------
   // Function declarations
   
   void FusionAhrsInitialise(FusionAhrs *const ahrs);
   
   void FusionAhrsReset(FusionAhrs *const ahrs);
   
   void FusionAhrsSetSettings(FusionAhrs *const ahrs, const FusionAhrsSettings *const settings);
   
   void FusionAhrsUpdate(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const FusionVector magnetometer, const float deltaTime);
   
   void FusionAhrsUpdateNoMagnetometer(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float deltaTime);
   
   void FusionAhrsUpdateExternalHeading(FusionAhrs *const ahrs, const FusionVector gyroscope, const FusionVector accelerometer, const float heading, const float deltaTime);
   
   FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs *const ahrs);
   
   void FusionAhrsSetQuaternion(FusionAhrs *const ahrs, const FusionQuaternion quaternion);
   
   FusionVector FusionAhrsGetLinearAcceleration(const FusionAhrs *const ahrs);
   
   FusionVector FusionAhrsGetEarthAcceleration(const FusionAhrs *const ahrs);
   
   FusionAhrsInternalStates FusionAhrsGetInternalStates(const FusionAhrs *const ahrs);
   
   FusionAhrsFlags FusionAhrsGetFlags(const FusionAhrs *const ahrs);
   
   void FusionAhrsSetHeading(FusionAhrs *const ahrs, const float heading);
   
   #endif
   
   //------------------------------------------------------------------------------
   // End of file
