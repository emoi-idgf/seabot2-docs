Upload a new install on a seabot
================================

After compiling the sources, to upload the program on a seabot2, use:


.. code-block:: bash

    zsh ~/seabot2/seabot2-tools/seabot2-scp/seabot2_sync_src <seabot2_id>


If you encounter permission issues running wtf on the seabot2, run:

.. code-block:: bash

    chmod +x ~/seabot2-ros/install/seabot2/share/seabot2/linux/wtf.sh
