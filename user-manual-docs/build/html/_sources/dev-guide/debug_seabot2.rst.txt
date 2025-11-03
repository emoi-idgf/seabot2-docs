Debug Seabot2
=============

On the seabot2, ROS2 nodes are launched via two services: ``seabot2.driver`` and ``seabot2``.
So their terminal outputs appear with other services, in journalctl.

In a seabot2 terminal :

.. code-block:: bash

    journalctl -f
