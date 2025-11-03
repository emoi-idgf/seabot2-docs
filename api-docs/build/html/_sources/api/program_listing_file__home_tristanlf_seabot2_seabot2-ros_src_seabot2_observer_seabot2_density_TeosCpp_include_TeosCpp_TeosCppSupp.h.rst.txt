
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_density_TeosCpp_include_TeosCpp_TeosCppSupp.h:

Program Listing for File TeosCppSupp.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_density_TeosCpp_include_TeosCpp_TeosCppSupp.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_density/TeosCpp/include/TeosCpp/TeosCppSupp.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   /****************************************
      order the array by largest value 1st
      [first array element]
      proceeding down to smallest value
      [last array element]
   ****************************************/
   bool orderByLargeToSmall(const DI &A, const DI& B)
   {
      if (A.d < B.d)
         return (false);
   
      if (A.d > B.d)
         return (true);
   
      if (A.i < B.i)
         return (true);
   
      return (false);
   }
   /****************************************
      order the array by smallest value 1st
      [first array element]
      proceeding down to largest value
      [last array element]
   ****************************************/
   bool orderBySmallToLarge(const DI &A, const DI& B)
   {
      if (A.d < B.d)
         return (true);
   
      if (A.d > B.d)
         return (false);
   
      if (A.i < B.i)
         return (false);
   
      return (true);
   }
