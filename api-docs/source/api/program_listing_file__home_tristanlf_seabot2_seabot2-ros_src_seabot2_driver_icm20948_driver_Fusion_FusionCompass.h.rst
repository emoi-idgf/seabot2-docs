
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionCompass.h:

Program Listing for File FusionCompass.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionCompass.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/Fusion/FusionCompass.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef FUSION_COMPASS_H
   #define FUSION_COMPASS_H
   
   //------------------------------------------------------------------------------
   // Includes
   
   #include "FusionConvention.h"
   #include "FusionMath.h"
   
   //------------------------------------------------------------------------------
   // Function declarations
   
   float FusionCompassCalculateHeading(
     const FusionConvention convention,
     const FusionVector accelerometer, const FusionVector magnetometer);
   
   #endif
   
   //------------------------------------------------------------------------------
   // End of file
