
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_launch_mission_launch.py:

Program Listing for File mission_launch.py
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_launch_mission_launch.py>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2/launch/mission_launch.py``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: py

   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch_ros.actions import Node
   
   def generate_launch_description():
   
       home_path = os.path.expanduser('~')
       parameters_file_list = [{"mission_path": home_path}]
   
       config_mission = os.path.join(
           home_path,
           'config/default/',  # Directory where yaml are
           'mission.yaml'  # Name of the file
       )
       if os.path.exists(config_mission):
           parameters_file_list.append(config_mission)
   
       config_safety = os.path.join(
           home_path,
           'config/default/',  # Directory where yaml are
           'safety.yaml'  # Name of the file
       )
       if os.path.exists(config_safety):
           parameters_file_list.append(config_safety)
   
       seabot2_mission = Node(
           package='seabot2_mission',
           executable='mission_node',
           namespace='mission',
           name='mission_node',
           parameters=parameters_file_list
       )
   
       seabot2_safety = Node(
           package='seabot2_safety',
           executable='safety_node',
           namespace='safety',
           name='safety_node',
           parameters=parameters_file_list
       )
   
       seabot2_record_parameters = Node(
           package='seabot2_log_parameters',
           executable='log_parameter_node',
           namespace='observer',
           name='log_parameter_node'
       )
   
       seabot2_iridium = Node(
           package='seabot2_iridium_driver',
           executable='iridium_node',
           namespace='iridium',
           name='iridium_node',
           parameters=parameters_file_list
       )
   
       return LaunchDescription([
           seabot2_mission,
           seabot2_safety,
           seabot2_record_parameters,
           #seabot2_iridium
       ])
