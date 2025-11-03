Convert Rosbags to Numpy Files and Visualize Data
=================================================

Tools
-----

seabot2_bag
^^^^^^^^^^^

The ``seabot2_bag`` tool processes a rosbag folder and convert it to numpy files.

After using ``seabot2_bag`` on a rosbag folder, a ``data`` folder containing numpy files is created inside the rosbag folder.

The ``seabot2_bag`` tool is located at::

   ~/seabot2/seabot2-tools/seabot2-log-analyzer/seabot2_bag


seabot2_log
^^^^^^^^^^^

The ``seabot2_log`` tool does the same as ``seabot2_bag``, then open a window to visualize data.

The ``seabot2_log`` tool is located at::

   ~/seabot2/seabot2-tools/seabot2-log-analyzer/seabot2_log


.. note:: The following tutorial uses ``seabot2_log`` for demo.



Option 1 (Recommended): On Your Host System with Your Current Config
--------------------------------------------------------------------

If you are not a ROS 2 user, jump to step 3.

1. **Build the seabot2_msgs package:**

   .. code-block:: bash

      zsh ~/seabot2/seabot2-tools/seabot2-log-analyzer/build_seabot2_msgs

2. **Source your ROS 2 setup:**

   .. code-block:: bash

      source ~/seabot2/seabot2-ros/install/setup.bash

3. **Use the tool (adapt the path if necessary):**

   .. code-block:: bash

      zsh ~/seabot2/seabot2-tools/seabot2-log-analyzer/seabot2_log path_to_your_rosbag_folder

If it doesn’t work, check that the rosbag hasn’t been processed already with another configuration.  
If so, there must be a ``data`` folder inside the rosbag folder — remove it and try again.

.. warning::
    **Non-ROS 2 users:**  
    Be careful with the rosbags you process. The seabot’s program must have been stopped cleanly using ``stop.sh``.  
    A clean rosbag contains a ``metadata.yaml`` file in addition to the ``.mcap`` data file.  
    If the bag is not clean, it cannot be processed without ros2 tools.



Option 2 (For ROS 2 Users with Version Issues): Force Not Using ROS 2 Python Libraries
--------------------------------------------------------------------------------------

In **``seabot2_log``** or **``seabot2_bag``**, depending on which tool you want to use, set the environment variable ``DO_NOT_USE_ROS2`` to ``1`` if you want to avoid using ROS 2 Python libraries.

Then follow *Option 1* as a non-ROS 2 user.



Option 3 (Docker): In a Seabot2 AMD64 ROS 2 Humble Docker Container
-------------------------------------------------------------------

1. You will need the ``seabot2-full-amd64`` Docker image.  
   Make sure you followed the setup guide in **seabot2-docker**.  
   You also need the **seabot2-ros** repository.

2. **Prior ROS 2 build:**  
   If ``seabot2_msgs`` has not been built since the last update, run:

   .. code-block:: bash

      zsh ~/seabot2/seabot2-ros/docker/build_amd64.sh

   If you don't want to compile all packages but only ``seabot2_msgs``, edit
   ``~/seabot2/seabot2-ros/docker/run_docker_build.sh`` and change the line:

   .. code-block:: bash

      colcon build

   to:

   .. code-block:: bash

      colcon build --packages-select seabot2_msgs

3. **Run the Docker container:**

   .. code-block:: bash

      zsh ~/seabot2/seabot2-tools/seabot2-log-analyzer/docker/run_docker_seabot2_display.sh

4. **In the container, use the tool on your rosbag:**

   .. code-block:: bash

      seabot2-tools/seabot2-log-analyzer/seabot2_log path_to_your_rosbag

   If you only want to get numpy data from rosbags, use ``seabot2_bag`` instead.

   (*Ctrl + D* or *exit* to leave the container*)

If it doesn’t work, check that the rosbag hasn’t been processed already with another configuration.  
If so, there must be a ``data`` folder inside the rosbag folder — remove it and try again.
