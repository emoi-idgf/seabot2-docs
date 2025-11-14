
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionCalibration.h:

Program Listing for File FusionCalibration.h
============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionCalibration.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/Fusion/FusionCalibration.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef FUSION_CALIBRATION_H
   #define FUSION_CALIBRATION_H
   
   //------------------------------------------------------------------------------
   // Includes
   
   #include "FusionMath.h"
   
   //------------------------------------------------------------------------------
   // Inline functions
   
   static inline FusionVector FusionCalibrationInertial(
     const FusionVector uncalibrated,
     const FusionMatrix misalignment, const FusionVector sensitivity, const FusionVector offset)
   {
     return FusionMatrixMultiplyVector(misalignment,
       FusionVectorHadamardProduct(FusionVectorSubtract(uncalibrated, offset), sensitivity));
   }
   
   static inline FusionVector FusionCalibrationMagnetic(
     const FusionVector uncalibrated,
     const FusionMatrix softIronMatrix, const FusionVector hardIronOffset)
   {
     return FusionMatrixMultiplyVector(softIronMatrix,
       FusionVectorSubtract(uncalibrated, hardIronOffset));
   }
   
   #endif
   
   //------------------------------------------------------------------------------
   // End of file
