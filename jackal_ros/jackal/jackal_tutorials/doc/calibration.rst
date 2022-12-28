Calibrating Jackal's IMU
========================

This tutorial explains how Jackal's compass works and how to calibrate it for top performance.

Calibrating Jackal's magnetometer is a straightforward process. For best performance, it's recommended to
perform this operation on a seasonal basis.


Understanding Compass Calibration
---------------------------------

Jackal includes integrated `inertial measurement`_, which allows it to estimate its own orientation in space.

This is achieved using a combination of three sensors: a gyroscope, an accelerometer, and a magnetometer.
These measure, respectively: angular motion, acceleration, and magnetic field. The angular motion is integrated
to achieve a relative estimate, which is then anchored to two absolute reference points: the gravity vector
as supplied by the accelometer, and the magnetic field vector as supplied by the magnetometer.

Unfortunately, although gravity is pretty consistent about pointing down, the magnetic reference vector
has an offset due to declination_, and is also highly subject to disturbance. There's little that can be done
by a disturbance outside of Jackal (for example, driving by a strong magnetic field), but the effect of magnetic
fields onboard Jackal from the battery, motors, sheet metal parts, etc, can be calibrated away.

 .. _inertial measurement: http://en.wikipedia.org/wiki/Inertial_measurement_unit
 .. _declination: http://en.wikipedia.org/wiki/Magnetic_declination


Running Jackal's Calibration
----------------------------

To run the calibration routine, connect to Jackal by SSH, and execute:

.. code-block:: bash

    rosrun jackal_base calibrate_compass

Jackal should rotate slowly in place for 60 seconds, recording magnetometer data.

The output will be:

.. code-block:: bash

    Started rosbag record, duration 60 seconds, pid [6793]
    Started motion commands, pid [6794]
    [ INFO] [1417463864.452760841]: Subscribing to /tf
    [ INFO] [1417463864.457840117]: Subscribing to /imu/rpy/raw
    [ INFO] [1417463864.462399042]: Subscribing to /imu/data_raw
    [ INFO] [1417463864.466624735]: Subscribing to /imu/mag
    [ INFO] [1417463864.474944762]: Recording to /tmp/calibrate_compass.xxxx/imu_record.bag.
    Test underway.
    Time remaining: 0
    Shutting down motion command publisher.
    Waiting for rosbag to shut down.
    Computing magnetic calibration.
    Calibration generated in /tmp/calibrate_compass.g8oM/mag_config.yaml.
    Restart ROS service to begin using saved calibration.


Once complete, you will be prompted to enter the user password in order to save the new calibration
into ``$ROS_ETC_DIR``. Once this is done, restart the ROS service to begin using the new calibration:

.. code-block:: bash

    sudo service ros restart

Calibration should be performed when jackal is first received, and any time the platform is modified by adding
new peripherals. It is also recommended to recalibrate annually as part of seasonal maintenance.
