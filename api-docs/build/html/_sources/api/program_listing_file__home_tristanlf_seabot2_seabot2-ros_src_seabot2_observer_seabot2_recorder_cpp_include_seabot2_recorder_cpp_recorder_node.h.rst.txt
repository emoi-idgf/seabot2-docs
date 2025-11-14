
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_recorder_cpp_include_seabot2_recorder_cpp_recorder_node.h:

Program Listing for File recorder_node.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_recorder_cpp_include_seabot2_recorder_cpp_recorder_node.h>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_recorder_cpp/include/seabot2_recorder_cpp/recorder_node.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_RECORDER_NODE_H
   #define BUILD_RECORDER_NODE_H
   
   #include "rclcpp/rclcpp.hpp"
   #include <filesystem>
   #include "std_srvs/srv/set_bool.hpp"
   
   using namespace std::chrono_literals;
   
   class RecorderNode final: public rclcpp::Node {
   public:
     RecorderNode();
   
     ~RecorderNode() override;
   
   public:
     const std::string command_ = "ros2 bag record --all --storage mcap";
   
   private:
   
     rclcpp::TimerBase::SharedPtr timer_;
     std::chrono::milliseconds loop_dt_ = 5s;   
     rclcpp::CallbackGroup::SharedPtr callback_group_;
     std::string workingDirectory_ = "";
     bool thread_currently_running_ = false;
     std::future < int > subprocessFuture_;
   
     rclcpp::Service < std_srvs::srv::SetBool > ::SharedPtr service_rosbag_;
   
   
   
     void manage_subprocess_rosbag(bool start_new_bag);
   
     void wait_kill();
   
     void init_interfaces();
   
     void callback_trigger(
       const std::shared_ptr < rmw_request_id_t > request_header,
       const std::shared_ptr < std_srvs::srv::SetBool::Request > request,
       std::shared_ptr < std_srvs::srv::SetBool::Response > response);
   
     void test_state();
   };
   
   #endif //BUILD_RECORDER_NODE_H
