Mission planning
================

Mission file
------------

Global structure
^^^^^^^^^^^^^^^^

On your chosen location on your local computer create an xml file to define the mission.

.. code-block:: xml

    <?xml version="1.0"?>

    <mission>
        <offset>
                <start_time_utc>
                <year>YYYY</year>
                <month>MM</month>
                <day>DD</day>
                <hour>hh</hour>
                <min>mm</min>
            </start_time_utc>
        </offset>

        <description>Optional description</description>

        <paths>

            <!--Where to set waypoints-->
            ...

        </paths>

    </mission>

Waypoints
^^^^^^^^^

Waypoint
""""""""

All waypoint types are defined under a dedicated XML tag. The waypoint parameters are defined under the waypoint tag.

Waypoints of all types take the following optional parameters :

* ``<duration_since_start>`` : Duration between the start of the mission and the end of the waypoint in seconds.
* ``<duration>`` : Waypoint duration in seconds

.. warning:: At least one of the above duration type must be given, else an error will be thrown when reading the mission file.
    If both are given, ``<duration_since_start>`` has priority over ``<duration>``.

* ``<limit_velocity>`` : Seabot limit velocity in m/s

Waypoint depth
""""""""""""""

Waypoint tag : ``<waypoint_depth>``

Required parameters :

* ``<depth>`` : Depth setpoint in meters

Example of depth waypoint for 10 min at 10 m :

.. code-block:: xml

    <waypoint_depth>
        <duration>600</duration>
        <limit_velocity>0.1</limit_velocity>
        <depth>10</depth>
    </waypoint_depth>

.. note:: Using the xml tag ``<waypoint>`` is also possible instead of ``<waypoint_depth>`` but prefer the above for readability

Waypoint seafloor landing
"""""""""""""""""""""""""

Waypoint tag : ``<seafloor_landing>``

Required parameters :


Waypoint temperature keeping
""""""""""""""""""""""""""""

Waypoint tag : ``<temperature_keeping>``

Required parameters :

* ``<temperature>`` : Temperature setpoint in degrees Celsius

Example of temperature keeping for 10 min at 15 °C :

.. code-block:: xml

    <waypoint_depth>
        <duration>600</duration>
        <limit_velocity>0.1</limit_velocity>
        <temperature>15</temperature>
    </waypoint_depth>

Waypoint temperature profile
""""""""""""""""""""""""""""

Waypoint tag : ``<temperature_profile>``

Required parameters :

* ``<temperature_high>`` : Temperature high boundary in °C
* ``<temperature_low>`` : Temperature low boundary in °C
* ``<depth_max>`` : Max depth boundary in m
* ``<depth_min>`` : Min depth boundary in m

Optional parameters :

* ``<max_delay>`` : Maximum duration in s for going up or down, defaults to 100 s

Example of temperature profile between 5 and 30 m, 10 to 18 °C, during 10 min :

.. code-block:: xml

    <waypoint_depth>
        <duration>600</duration>
        <limit_velocity>0.1</limit_velocity>
        <temperature_high>18</temperature_high>
        <temperature_low>10</temperature_low>
        <depth_min>5</depth_min>
        <depth_max>30</depth_max>
        <max_delay>300</max_delay>
    </waypoint_depth>

Waypoint GNSS profile
"""""""""""""""""""""

Waypoint tag : ``<sgnss_profile>``

Parameters :

Loop
^^^^

Upload the mission
------------------

Once defined, upload your mission file on the seabot using:

.. code-block:: bash

    zsh ~/seabot2/seabot2-tools/seabot2-scp/seabot2_mission_upload <path_to_your_mission_file> <seabot2_id>