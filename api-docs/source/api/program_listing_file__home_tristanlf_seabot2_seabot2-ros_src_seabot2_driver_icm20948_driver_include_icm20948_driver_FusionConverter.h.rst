
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_include_icm20948_driver_FusionConverter.h:

Program Listing for File FusionConverter.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_icm20948_driver_include_icm20948_driver_FusionConverter.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/icm20948_driver/include/icm20948_driver/FusionConverter.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by lemezoth on 01/01/24.
   //
   
   #include "Fusion.h"
   #include <vector>
   
   
   //------------------------------------------------------------------------------
   // std::vector conversion
   
   static inline FusionVector FusionVectorFromStdVector(const std::vector<double> &vector) {
       const FusionVector result = {.axis = {
               .x = static_cast<float>(vector[0]),
               .y = static_cast<float>(vector[1]),
               .z = static_cast<float>(vector[2]),
       }};
       return result;
   }
   
   static inline FusionMatrix FusionMatrixFromStdVector(const std::vector<double> &matrix) {
       const FusionMatrix result = {.element = {
               .xx = static_cast<float>(matrix[0]),
               .xy = static_cast<float>(matrix[1]),
               .xz = static_cast<float>(matrix[2]),
               .yx = static_cast<float>(matrix[3]),
               .yy = static_cast<float>(matrix[4]),
               .yz = static_cast<float>(matrix[5]),
               .zx = static_cast<float>(matrix[6]),
               .zy = static_cast<float>(matrix[7]),
               .zz = static_cast<float>(matrix[8]),
       }};
       return result;
   }
   
   static inline FusionQuaternion FusionQuaternionFromStdVector(const std::vector<double> &quaternion) {
       const FusionQuaternion result = {.element = {
               .w = static_cast<float>(quaternion[0]),
               .x = static_cast<float>(quaternion[1]),
               .y = static_cast<float>(quaternion[2]),
               .z = static_cast<float>(quaternion[3]),
       }};
       return result;
   }
   
   static inline FusionEuler FusionEulerFromStdVector(const std::vector<double> &euler) {
       const FusionEuler result = {.angle = {
               .roll = static_cast<float>(euler[0]),
               .pitch = static_cast<float>(euler[1]),
               .yaw = static_cast<float>(euler[2]),
       }};
       return result;
   }
   
   static inline std::vector<double> FusionVectorToStdVector(const FusionVector &vector) {
       return {vector.axis.x, vector.axis.y, vector.axis.z};
   }
   
   static inline std::vector<double> FusionMatrixToStdVector(const FusionMatrix &matrix) {
       return {matrix.element.xx, matrix.element.xy, matrix.element.xz,
               matrix.element.yx, matrix.element.yy, matrix.element.yz,
               matrix.element.zx, matrix.element.zy, matrix.element.zz};
   }
   
   static inline std::vector<double> FusionQuaternionToStdVector(const FusionQuaternion &quaternion) {
       return {quaternion.element.w, quaternion.element.x, quaternion.element.y, quaternion.element.z};
   }
   
   static inline std::vector<double> FusionEulerToStdVector(const FusionEuler &euler) {
       return {euler.angle.roll, euler.angle.pitch, euler.angle.yaw};
   }
