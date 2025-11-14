
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_density_TeosCpp_include_TeosCpp_map_struct.h:

Program Listing for File map_struct.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_density_TeosCpp_include_TeosCpp_map_struct.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_density/TeosCpp/include/TeosCpp/map_struct.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef MAP_STRUCT_H_INCLUDED
   #define MAP_STRUCT_H_INCLUDED
   
   /*************************
      Version 1.06
      by Randall Kent Whited
      rkwhited@gmail.com
   **************************/
   
   using namespace std;
   
   /*************************************
   (the value repeated in original)
   "replaced" in VLA (very large arrays)
   **************************************/
   const double repeatSectionValue = 8.9999999999999998e+90;
   
   struct ARRAYINFO
   {
     unsigned begins;
     unsigned ends;
     unsigned elements;
     bool wasRepeatValue;
   
     void init()
     {
       begins = ends = elements = 0;
       wasRepeatValue = true;
     }
   };
   
   #endif // MAP_STRUCT_H_INCLUDED
