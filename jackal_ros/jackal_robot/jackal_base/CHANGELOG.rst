^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2021-03-08)
------------------
* Use eval to find the mag config. This should fix a bug when jackal is installed to ros.d
* Set default for optenv JACKAL_MAG_CONFIG
* Removed env-hooks
* Contributors: Chris Iverach-Brereton, Dave Niewinski

0.6.0 (2020-04-20)
------------------
* [jackal_base] Stopped using Vector3 messages for on-board IMU.
* Folding Wifi into single function
* removed extra code.
* specify jackal serial port as arg in launch file (`#7 <https://github.com/jackal/jackal_robot/issues/7>`_)
* allow configuring jackal's port in env (`#6 <https://github.com/jackal/jackal_robot/issues/6>`_)
* fix calibration script (`#5 <https://github.com/jackal/jackal_robot/issues/5>`_)
  * fix batch file
  * fix batch script for mix use of forward slash
  * remove todo comment
* check wireless connection on Windows (`#4 <https://github.com/jackal/jackal_robot/issues/4>`_)
  * check wireless connection on Windows
  * add comment about unused member variable
* add calibrate_compass.bat (`#3 <https://github.com/jackal/jackal_robot/issues/3>`_)
* add env-hook (`#2 <https://github.com/jackal/jackal_robot/issues/2>`_)
* quickly get around the build break.
* Contributors: James Xu, Lou Amadio, Sean Yen, Tony Baltovski, seanyen

0.3.9 (2019-06-19)
------------------
* [jackal_base] Minor launch file nit.
* Merge pull request `#14 <https://github.com/jackal/jackal_robot/issues/14>`_ from hawesie/kinetic-devel
  Add parameter to change IMU message type.
* Add paramter to change IMU message type.
* Contributors: Nick Hawes, Tony Baltovski

0.3.8 (2018-11-07)
------------------
* [jackal_base] Added environment variable for changing the wireless interface which controlls the light on the HMI.
* [jackal_base] Fixed linting.
* Contributors: Tony Baltovski

0.3.7 (2018-08-02)
------------------
* Updated the wireless interface for kinetic
* Contributors: Dave Niewinski

0.3.6 (2016-09-30)
------------------
* Minor linter fixes to jackal_diagnostic_updater.
* Contributors: Tony Baltovski

0.3.5 (2016-02-22)
------------------

0.3.4 (2016-02-10)
------------------

0.3.3 (2015-02-20)
------------------
* Remove duration cast, was using incorrect method to get time out of clock
* Return from function early when getifaddrs fails to avoid double free.
* Contributors: Mike Purvis, Paul Bovbel

0.3.2 (2015-02-19)
------------------
* Add simple connection detect to enable Jackal's wifi status LED.
* Get elapsed time from monotonic time source
* Contributors: Mike Purvis, Paul Bovbel

0.3.1 (2015-02-03)
------------------

0.3.0 (2015-01-20)
------------------

0.2.2 (2015-01-14)
------------------
* Simplify mag computation.
* Don't output stderr from env hook.
* A new approach to fallback configuration.
* Add more missing dependencies to jackal_base.
* Add default compass configuration and install it.
* Remove sixpair, use system one instead.
* Contributors: Mike Purvis

0.2.1 (2015-01-12)
------------------
* Resolve catkin_lint.
* Contributors: Mike Purvis

0.2.0 (2015-01-12)
------------------
* read mag msg in radian.
* added magnetometer calibration computation scripts.
* Contributors: Shokoofeh Pourmehr

0.1.0 (2014-11-11)
------------------
* Initial release of basic functionality.
* Contributors: Mike Purvis
