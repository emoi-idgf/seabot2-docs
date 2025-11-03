
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionConvention.h:

Program Listing for File FusionConvention.h
===========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_Fusion_FusionConvention.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/Fusion/FusionConvention.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef FUSION_CONVENTION_H
   #define FUSION_CONVENTION_H
   
   //------------------------------------------------------------------------------
   // Definitions
   
   typedef enum {
       FusionConventionNwu, /* North-West-Up */
       FusionConventionEnu, /* East-North-Up */
       FusionConventionNed, /* North-East-Down */
   } FusionConvention;
   
   #endif
   
   //------------------------------------------------------------------------------
   // End of file
