^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2021-03-08)
------------------
* Add VLP16 support, refactor main/secondary laser envar support (`#27 <https://github.com/jackal/jackal_robot/issues/27>`_)
  * Add groups for the front/rear fender lasers' nodes
  * Revert the front/rear laser frames to "front_laser" and "rear_laser" respectively, to match with the frames in jackal_description (proposed PR to rename them was rejected)
  * Enable launching the secondary 2D laser, and the primary 3D laser
  * Rename LASER2 to LASER_SECONDARY
  * Change the default mount for the 3d laser to the middle
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
* Remove the PS4 symlink; we've consolidated the udev rules and are relying on ds4drv to provide that rule now
* Contributors: Chris I-B, Chris Iverach-Brereton

0.6.0 (2020-04-20)
------------------
* Fix the IP address for the urg_node used by the hokuyo lidar
* Add the additional udev rule for the PS4 controller
* [jackal_bringup] Re-added pointgrey_camera_driver as run depend.
* Add udev rules for the PS4, logitech, and USB to serial adapter (copied from Husky)
* Add the urg_node to the dependencies
* Create the urg_node needed for the hokuyo sensor
* Contributors: Chris I-B, Chris Iverach-Brereton, Tony Baltovski

0.3.9 (2019-06-19)
------------------
* Temporarily removed point grey driver dependency
* Contributors: Dave Niewinski

0.3.8 (2018-11-07)
------------------

0.3.7 (2018-08-02)
------------------
* Updated default IPs for Kinetic
* Added stereo cameras accessory.
* Contributors: Dave Niewinski, Tony Baltovski

0.3.6 (2016-09-30)
------------------
* Added parameter for flea3 camera frame rate.
* Added flea3 to accessories.
* Contributors: Tony Baltovski

0.3.5 (2016-02-22)
------------------
* Fixed scan topic name and optenv for lms1xx bringup.
* Contributors: Tony Baltovski

0.3.4 (2016-02-10)
------------------
* Added bumblebee2 to accessories.
* Contributors: Tony Baltovski

0.3.3 (2015-02-20)
------------------

0.3.2 (2015-02-19)
------------------
* Retry on startup when network device does not exist.
* Set args types to int.
* Contributors: Mike Purvis

0.3.1 (2015-02-03)
------------------
* Add multicast lib, add navsat rtk relay.
* Add launch functionality of the Novatel GPS to accessories.launch
* Contributors: BryceVoort, Mike Purvis

0.3.0 (2015-01-20)
------------------
* Support changing topic name for accessory laser.
* Support turning on the LMS1xx accessory via optenv.
* Contributors: Mike Purvis

0.2.2 (2015-01-14)
------------------
* Add default compass configuration and install it.
* Contributors: Mike Purvis

0.2.1 (2015-01-12)
------------------
* Resolve catkin_lint.
* Contributors: Mike Purvis

0.2.0 (2015-01-12)
------------------
* Add install script.
* Contributors: Mike Purvis

0.1.0 (2014-11-11)
------------------
* Make jackal_bringup package just a stub for now.
* Contributors: Mike Purvis
