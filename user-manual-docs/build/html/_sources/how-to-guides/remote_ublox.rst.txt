Connect the Remote u-blox Module to Local U-Center via Network
==============================================================

1. Set Up: Use U-Center on Ubuntu
-------------------------------------

Based on the following tutorial:  
`How to use Ardusimple kit and U-center in Ubuntu (French) <https://fr.ardusimple.com/how-to-use-ardusimple-kit-and-u-center-in-ubuntu/>`_

Install **Wine** (Windows emulator):

.. code-block:: bash

   sudo apt install wine64

Download the **U-Center** installer from the official u-blox website, either manually or via command line:

.. code-block:: bash

   cd ~/Downloads
   wget "https://content.u-blox.com/sites/default/files/2025-06/u-center_v25.06_installer.zip"

Extract the installer:

.. code-block:: bash

   unzip u-center_v25.06_installer.zip

Launch the installer with Wine:

.. code-block:: bash

   wine ./u-center_v25.06_installer.exe

Install **U-Center** as if you were on Windows and choose your installation path, for example ``~/APPLIS/WIN``.

Then, every time you want to launch U-Center, simply run:

.. code-block:: bash

   wine your-installation-path/u-center.exe

---

2. Emulate the Remote Serial Port on Your Local Machine Using *socat*
--------------------------------------------------------------------------

Use the dedicated script with the appropriate ``seabot2_id`` value:

.. code-block:: bash

   sudo zsh ~/seabot2/seabot2-tools/seabot2-sensors/remote_gnss_serial.sh <seabot2_id>

The remote GNSS serial port will be emulated on your local machine at ``/dev/ttyS10``,  
which corresponds to **COM11** in Windows terminology.

Now, simply launch **U-Center** with Wine and select the port **COM11**.
