
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionCompass.c:

Program Listing for File FusionCompass.c
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionCompass.c>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/Fusion/FusionCompass.c``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   //------------------------------------------------------------------------------
   // Includes
   
   #include "FusionAxes.h"
   #include "FusionCompass.h"
   #include <math.h> // atan2f
   
   //------------------------------------------------------------------------------
   // Functions
   
   float FusionCompassCalculateHeading(const FusionConvention convention, const FusionVector accelerometer, const FusionVector magnetometer) {
       switch (convention) {
           case FusionConventionNwu: {
               const FusionVector west = FusionVectorNormalise(FusionVectorCrossProduct(accelerometer, magnetometer));
               const FusionVector north = FusionVectorNormalise(FusionVectorCrossProduct(west, accelerometer));
               return FusionRadiansToDegrees(atan2f(west.axis.x, north.axis.x));
           }
           case FusionConventionEnu: {
               const FusionVector west = FusionVectorNormalise(FusionVectorCrossProduct(accelerometer, magnetometer));
               const FusionVector north = FusionVectorNormalise(FusionVectorCrossProduct(west, accelerometer));
               const FusionVector east = FusionVectorMultiplyScalar(west, -1.0f);
               return FusionRadiansToDegrees(atan2f(north.axis.x, east.axis.x));
           }
           case FusionConventionNed: {
               const FusionVector up = FusionVectorMultiplyScalar(accelerometer, -1.0f);
               const FusionVector west = FusionVectorNormalise(FusionVectorCrossProduct(up, magnetometer));
               const FusionVector north = FusionVectorNormalise(FusionVectorCrossProduct(west, up));
               return FusionRadiansToDegrees(atan2f(west.axis.x, north.axis.x));
           }
       }
       return 0; // avoid compiler warning
   }
   
   //------------------------------------------------------------------------------
   // End of file
