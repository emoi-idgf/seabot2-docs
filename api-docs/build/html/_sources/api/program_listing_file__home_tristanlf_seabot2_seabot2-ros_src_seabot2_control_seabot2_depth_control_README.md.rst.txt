
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_depth_control_README.md:

Program Listing for File README.md
==================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_control_seabot2_depth_control_README.md>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_control/seabot2_depth_control/README.md``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: markdown

   Note to install ibex-lib
   
   ```bash 
   git clone -b develop https://github.com/ibex-team/ibex-lib.git
   cd ibex-lib
   mkdir -p build && cd build
   # or -DCMAKE_INSTALL_PREFIX=${HOME}/ibex
   cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Relase -DCMAKE_CXX_FLAGS="-fPIC" -DCMAKE_C_FLAGS="-fPIC" ..
   make && make check && make install
   ```
