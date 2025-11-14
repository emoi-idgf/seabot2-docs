Build ROS2 sources
==================

Before continuing, make sure you have followed the ``seabot2-docker`` setup guide.

.. code-block:: bash

    cd ~/seabot2/seabot2-ros

For **arm64** (seabot2 Raspberry Pi architecture)

.. code-block:: bash

    bash docker/docker_build_arm64.sh


For **amd64** (laptop architecture)

.. code-block:: bash

    bash docker/docker_build_amd64.sh
