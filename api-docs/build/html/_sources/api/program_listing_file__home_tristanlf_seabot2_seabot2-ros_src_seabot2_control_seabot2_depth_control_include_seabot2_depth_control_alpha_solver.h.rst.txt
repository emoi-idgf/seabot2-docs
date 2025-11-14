
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_depth_control_include_seabot2_depth_control_alpha_solver.h:

Program Listing for File alpha_solver.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_depth_control_include_seabot2_depth_control_alpha_solver.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_depth_control/include/seabot2_depth_control/alpha_solver.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 11/04/23.
   //
   
   #ifndef SEABOT2_DEPTH_CONTROL__ALPHA_SOLVER_H_
   #define SEABOT2_DEPTH_CONTROL__ALPHA_SOLVER_H_
   
   // #include "ibex.h"
   #include <array>
   #include <vector>
   #include <utility>
   
   using std::array;
   using std::vector;
   using std::pair;
   
   class AlphaSolver {
   public:
     AlphaSolver() = default;  // Default constructor
   
       /*
        * Compute the alpha value
        */
     double compute_alpha(const double beta);
   
   //    void update_coeff(const double Cf, const double A, const double B, const double dVp_max);
   
     pair < bool, double > exist_in_memory(const double beta);
   
     void add_to_memory(const double beta, const double alpha);
   
     vector < array < double, 2 >> &get_computed_memory() {
       return computed_memory_;
     }
   
     void set_test_in_memory(bool enable)
     {
       enable_test_in_memory_ = enable;
     }
   
   private:
       // B = rho*S/(2*mv)
       // A = rho*g/mv
       // S= math.pi*(0.125/2.)**2
   //    ibex::Interval Cf_ = ibex::Interval(1.0);
   //    ibex::Interval A_ = ibex::Interval(418.96875);
   //    ibex::Interval B_ = ibex::Interval(0.262055051263797);
   //    ibex::Interval dVp_max_ = ibex::Interval(1.0072731445572273e-06);
   
     double alpha_search_max_ = 20.0;
   //    double z_search_max_ = 10.0;
   //    double epsilon_ = 0.01; // Drive the time of the algorithm
   
     vector < array < double, 2 >> computed_memory_;
   
     bool enable_test_in_memory_ = true;
   };
   
   
   #endif  // SEABOT2_DEPTH_CONTROL__ALPHA_SOLVER_H_
