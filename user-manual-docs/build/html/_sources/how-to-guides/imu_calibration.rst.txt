Calibrate the IMU
=================

First, collect raw data with a seabot2 following these steps:

1. Turn on the robot

2. Wait for the program to start completely, especially the imu driver

3. Move the robot randomly around all 3 axis, make sure to cover as much directions as possible

4. Stop the mission

5. Retrieve the rosbags on your computer

6. On your computer, process the rosbags with ``seabot2_log`` or ``seabot2_bag``

Now, in your rosbag, you must have a ``raw_data.npz`` file in ``data/driver``

Then, pull the magnetometer-calibration tool from github in your chosen location:

.. code-block:: bash

    git clone https://github.com/tlefloch/magnetometer-calibration.git


And follow the instructions in the ``README``

You should obtain:

* **M** the soft-iron calibration matrix

* **v** the hard-iron calibration matrix

Finally, update the configuration file ``config/seabot<id>/driver.yaml`` with these values