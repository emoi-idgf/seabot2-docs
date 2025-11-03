
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_log_parameters_include_seabot2_log_parameters_log_parameter_node.hpp:

Program Listing for File log_parameter_node.hpp
===============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_observer_seabot2_log_parameters_include_seabot2_log_parameters_log_parameter_node.hpp>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_observer/seabot2_log_parameters/include/seabot2_log_parameters/log_parameter_node.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef BUILD_LOG_PARAMETER_NODE_HPP
   #define BUILD_LOG_PARAMETER_NODE_HPP
   
   #include "rclcpp/rclcpp.hpp"
   #include "seabot2_msgs/msg/log_parameter.hpp"
   #include "std_srvs/srv/trigger.hpp"
   
   using namespace std::chrono_literals;
   using namespace std;
   
   class LogParameterNode final : public rclcpp::Node {
   public:
       LogParameterNode();
   
   private:
   
       rclcpp::Publisher<seabot2_msgs::msg::LogParameter>::SharedPtr publisher_parameters_;
       rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_log_parameters_;
   
       rclcpp::CallbackGroup::SharedPtr callback_group_;
   
   
       void init_interfaces();
   
       void record_parameters();
   
       std::vector<std::string> get_param_list(const std::string &node_name);
   
       void get_param_values(const std::string &node_name,
                                               const std::vector<std::string> &param_name);
   
       void service_record(const std::shared_ptr<rmw_request_id_t> request_header,
                                             const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
   
   private:
   
   };
   #endif //BUILD_LOG_PARAMETER_NODE_HPP
