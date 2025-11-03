
.. _program_listing_file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_xbee_driver_setup.py:

Program Listing for File setup.py
=================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_tristanlf_seabot2_seabot2-ros_src_seabot2_driver_xbee_driver_setup.py>` (``/home/tristanlf/seabot2/seabot2-ros/src/seabot2_driver/xbee_driver/setup.py``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: py

   from setuptools import find_packages, setup
   
   package_name = 'xbee_driver'
   
   setup(
       name=package_name,
       version='0.0.1',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools', 'digi-xbee'],
       zip_safe=True,
       maintainer='lemezoth',
       maintainer_email='thomas.le_mezo@ensta-bretagne.org',
       description='Xbee driver for seabot2 robot',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'xbee = xbee_driver.xbee:main'
           ],
       },
   )
