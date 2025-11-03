
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionOffset.c:

Program Listing for File FusionOffset.c
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionOffset.c>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/Fusion/FusionOffset.c``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   //------------------------------------------------------------------------------
   // Includes
   
   #include "FusionOffset.h"
   #include <math.h> // fabs
   
   //------------------------------------------------------------------------------
   // Definitions
   
   #define CUTOFF_FREQUENCY (0.02f)
   
   #define TIMEOUT (5)
   
   #define THRESHOLD (3.0f)
   
   //------------------------------------------------------------------------------
   // Functions
   
   void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate) {
       offset->filterCoefficient = 2.0f * (float) M_PI * CUTOFF_FREQUENCY * (1.0f / (float) sampleRate);
       offset->timeout = TIMEOUT * sampleRate;
       offset->timer = 0;
       offset->gyroscopeOffset = FUSION_VECTOR_ZERO;
   }
   
   FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope) {
   
       // Subtract offset from gyroscope measurement
       gyroscope = FusionVectorSubtract(gyroscope, offset->gyroscopeOffset);
   
       // Reset timer if gyroscope not stationary
       if ((fabs(gyroscope.axis.x) > THRESHOLD) || (fabs(gyroscope.axis.y) > THRESHOLD) || (fabs(gyroscope.axis.z) > THRESHOLD)) {
           offset->timer = 0;
           return gyroscope;
       }
   
       // Increment timer while gyroscope stationary
       if (offset->timer < offset->timeout) {
           offset->timer++;
           return gyroscope;
       }
   
       // Adjust offset if timer has elapsed
       offset->gyroscopeOffset = FusionVectorAdd(offset->gyroscopeOffset, FusionVectorMultiplyScalar(gyroscope, offset->filterCoefficient));
       return gyroscope;
   }
   
   //------------------------------------------------------------------------------
   // End of file
