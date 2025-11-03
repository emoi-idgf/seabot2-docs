
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionOffset.h:

Program Listing for File FusionOffset.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionOffset.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/Fusion/FusionOffset.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef FUSION_OFFSET_H
   #define FUSION_OFFSET_H
   
   //------------------------------------------------------------------------------
   // Includes
   
   #include "FusionMath.h"
   
   //------------------------------------------------------------------------------
   // Definitions
   
   typedef struct {
       float filterCoefficient;
       unsigned int timeout;
       unsigned int timer;
       FusionVector gyroscopeOffset;
   } FusionOffset;
   
   //------------------------------------------------------------------------------
   // Function declarations
   
   void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate);
   
   FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope);
   
   #endif
   
   //------------------------------------------------------------------------------
   // End of file
