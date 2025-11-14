
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_iridium_driver_include_seabot2_iridium_driver_sbd_mission_xml.h:

Program Listing for File mission_xml.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_seabot2_iridium_driver_include_seabot2_iridium_driver_sbd_mission_xml.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/seabot2_iridium_driver/include/seabot2_iridium_driver/sbd/mission_xml.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef MISSIONXML_H
   #define MISSIONXML_H
   
   #include "seabot2_iridium_driver/sbd/log_data.h"
   #include <boost/property_tree/ptree.hpp>
   
   class MissionXML
   {
   public:
   
     MissionXML(LogData & log);
   
     void write(const std::string & filename) const;
   
   private:
     LogData m_log;
   //  boost::property_tree::ptree m_tree;
   };
   
   #endif // MISSIONXML_H
