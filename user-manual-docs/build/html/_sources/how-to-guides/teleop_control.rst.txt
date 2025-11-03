Teleoperately control the thrusters with a joystick
===================================================

via wi-fi
---------

Both the robot and your computer must be connected to the same network

**On the robot:**

In seabot2-ros/install/seabot2/share/seabot2/linux/source-ros.sh, make sure ``ROS_AUTOMATIC_DISCOVERY_RANGE`` is set to ``SUBNET`` and not ``LOCALHOST``


**On your computer:**

Set ``ROS_AUTOMATIC_DISCOVERY_RANGE`` to ``SUBNET``

.. code-block:: bash

    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET


Then use:

.. code-block:: bash

    zsh ~/seabot2/seabot2-tools/seabot2-joystick/seabot2_joystick <seabot2_id>

Now you can teleoperate your seabot2's thrusters.


via Xbee Radio
--------------

Before proceeding, place your working shell in a specific python environment containing the digi-xbee module.

Make sure you have built the ``seabot2_msgs`` and ``xbee_driver`` packages on your computer. To do so, use:

.. code-block:: bash

    zsh ~/seabot2/seabot2-tools/seabot2-xbee/build_ros_xbee.sh


In order to make sure there is no conflict on the ``/cmd_vel`` topic if both your laptop and the seabot2 are connected to the same network, make sure ``ROS_AUTOMATIC_DISCOVERY_RANGE`` is set to ``LOCALHOST`` both on the seabot2 and on your computer.

.. code-block:: bash

    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST


Then, launch the xbee driver on your computer:

.. code-block:: bash

    source ~/seabot2/seabot2-tools/seabot2-xbee/seabot2_xbee.sh <seabot2_id>


In another terminal, launch the teleop_twist_joy program using:

.. code-block:: bash

    source ~/seabot2/seabot2-tools/seabot2-joystick/seabot2_joystick

.. note:: No need to specify the id here, the commands will be retrieved by the xbee node, which is already targeting a seabot2, and sent.