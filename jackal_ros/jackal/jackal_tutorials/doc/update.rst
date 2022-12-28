Keeping Jackal Updated
======================

Jackal is always being improved, both its own software and the many community ROS packages upon which it
depends! You can use the apt package management system to receive new versions all software running on the
platform.


Getting New Packages
--------------------

Each Jackal leaves the factory already configured to pull packages from http://packages.ros.org as well as
http://packages.clearpathrobotics.com. To update your package and download new package versions, simply run:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get dist-upgrade

If you see any errors, please `get in touch`_ and we'll see if we can get you sorted out.

.. _get in touch: https://support.clearpathrobotics.com/hc/en-us/requests/new


MCU Firmware Update
-------------------

When you update packages, there is periodically a new version of Jackal's firmware available. You will know this
is the case in one of two ways:

1. The firmware and PC are unable to connect to each other, which will be apparent if the two-arrow comms indicator
   fails to come on after system bootup.
2. If the firmware version number in the ``/status`` message does not match the package version output by
   ``dpkg -s ros-indigo-jackal-firmware``. In the future there will be an automated check for this which outputs
   a diagnostics warning when a firmware update is available to be applied.

If new firmware is available, follow the below procedure to flash it to Jackal's MCU:

1. Place Jackal up on blocks. Firmware loading does not usually result in unintended motion, but it's safest when
   off the ground.
2. Ensure that Jackal is on and open.
3. In the top-middle of the main circuit board, look for a small left-right switch in between two buttons.
4. By default it is in the left position, but move it now to the right position.
5. Press the reset button to the left, labeled ``M_RST``. Jackal is now in its bootloader, ready to receive new
   firmware.

Now, from Jackal's PC (connected via SSH or screen/keyboard), execute:

.. code-block:: bash

    rosrun jackal_firmware upload

You should see about 20 seconds worth of lines output beginning with "Download from image ...". When this is
complete, move the switch back to the leftmost position and quickly push the reset button again. You're now
running the updated firmware!

If you're too slow on pushing the reset button, Jackal will power off, including the internal PC. It's okay
if that happens; just press the external power button again, and you should be back in business.


.. _scratch:

Starting From Scratch
---------------------

If Jackal's computer has become inoperable, or for any reason you want to restore it to the factory state, begin
by opening Jackal, lowering the computer tray, and connecting a screen and keyboard, as well as a wired internet
connection. You can then download the most recent version of the Jackal boot ISO from the following location:

http://packages.clearpathrobotics.com/stable/images/latest/indigo-jackal/amd64/

Use unetbootin or a similar tool to flash the ISO image to a USB memory stick. Boot Jackal's computer with the USB
memory connected, and you should be in the purple Debian/Ubuntu installer. The installer runs by itself and shuts
down the computer when finished.

Once done, turn Jackal on once more, and run the following:

.. code-block:: bash

    rosrun jackal_bringup install

This installs Jackal's `robot_upstart`_ job, so that ROS starts each time the robot starts.

.. _robot_upstart: http://wiki.ros.org/robot_upstart

Note that if you may need to re-pair your gamepad to the robot, and you'll have some extra work to do if you have
integrated accessories which require additional launchers or URDF.


Bluetooth Controller Pairing
----------------------------

If your Sixaxis controller runs out of batteries, or you purchase a new one, you might want to re-pair your platform
and controller. To do this, lower the computer tray and plug the controller into an available USB port using a
standard Mini-B USB cable. Then, from the prompt, run:

.. code-block:: bash

    sudo sixpair

You should see a notice that the MAC address of Jackal's bluetooth adapter has been written into the controller. Now
disconnect the USB cable and you should be able to press the pair button and achieve a pairing. Note that this first
pairing *may* cause the joystick to come up as ``/dev/input/js1`` rather than ``/dev/input/js0``. If Jackal does not
respond to your commands, power-cycle the full system and you should be set.

