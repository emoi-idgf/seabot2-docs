
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_launch_seabot_launch.py:

Program Listing for File seabot_launch.py
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_launch_seabot_launch.py>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2/launch/seabot_launch.py``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: py

   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.actions import ExecuteProcess
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   
   def generate_launch_description():
   
       control = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('seabot2'), 'launch'),
               '/control_launch.py'])
       )
   
       observer = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('seabot2'), 'launch'),
               '/observer_launch.py'])
       )
   
       mission = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('seabot2'), 'launch'),
               '/mission_launch.py'])
       )
   
       return LaunchDescription([
          control,
          observer,
          mission
       ])
