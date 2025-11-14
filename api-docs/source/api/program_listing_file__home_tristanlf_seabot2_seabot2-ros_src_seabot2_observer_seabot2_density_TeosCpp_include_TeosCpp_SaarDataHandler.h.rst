
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_density_TeosCpp_include_TeosCpp_SaarDataHandler.h:

Program Listing for File SaarDataHandler.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_density_TeosCpp_include_TeosCpp_SaarDataHandler.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_density/TeosCpp/include/TeosCpp/SaarDataHandler.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef SAARDATAHANDLER_H
   #define SAARDATAHANDLER_H
   
   /***************************************
     Version 1.06
     by Randall Kent Whited
     rkwhited@gmail.com
     ------------------
     This is a modification of the
     TEOS-10 file "gsw_saar_data.c"
     as adapted from the TEOS-10
     C version 3.05
     http://www.teos-10.org
     -------------------------------------
     About a 40% reduction in file size
     (disk space & memory use) resulted,
     in the debug compilation.
     -------------------------------------
     All copyrights and all license issues
     are the same as for the C version
   ****************************************/
   
   using namespace std;
   
   const double SAAR_ERROR_LIMIT = 1e10;
   
   const unsigned sdata_lat_max = 45;
   const unsigned sdata_long_max = 91;
   const unsigned ndepth_ref_max = 4095;
   /*******************************************
      These next two VLA (very large arrays)
      were re-designed by removing a meg or so
      of repeated-value sections and then
      'mapping' into the new locations.
   ********************************************/
   const unsigned delta_sa_max = 111628;
   const unsigned saar_max = 111921;
   
   const int gsw_nx = 91;
   const int gsw_ny = 45;
   const int gsw_nz = 45;
   
   /****************************************
      SAAR data-array access by C++
      "getter" methods/member functions
      -------------------------------------
      The parameter "pos" is the zero-based
      original array position of the data
      value being sought.
      -------------------------------------
      Objects of this class handle array
      data that was originally placed in
      the "gsw_saar_data.c" file.
   ****************************************/
   class SaarDataHandler
   {
   public:
     SaarDataHandler();
     virtual ~SaarDataHandler();
   
     const double get_p_ref(unsigned pos);
     const double get_lats_ref(unsigned pos);
     const double get_longs_ref(unsigned pos);
     const double get_saar_ref(unsigned pos);
     const double get_ndepth_ref(unsigned pos);
     const double get_delta_sa_ref(unsigned pos);
   
   private:
     double getSaarValue(unsigned tpos);
     double getDeltaSaValue(unsigned tpos);
   
     unsigned getSaarArrayPos(unsigned valPos);
     unsigned getDeltaSaArrayPos(unsigned valPos);
   
     unsigned getOffset(unsigned arrayNumber, bool isSaar);
   };
   
   #endif // SAARDATAHANDLER_H
