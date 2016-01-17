^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roomba_serial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Catkin will now know to install roomba.py
  I added it to the list of scripts to copy to the install directory.
* Initial commit of class and ROS stuff for roomba.
  I think the controller system has all that I want it to.
* Added the sensor service to the list of services.
  So now catkin will process it.
* Fixed some type mistakes for Sensors service.
* Added a service file for sensor query & reply.
* Set it up so catkin will process the message files
  Informed it about message generation depenencies and existing messages.
* Began adding message files.
  These will be sent by other nodes commanding the roomba controller.
* Changed package name so catkin won't complain.
  It didn't like the hyphen, so I used an underscore instead.
* Removed the RoombaCommand message
  Because it wasn't really a good way of structuring things.
* Initialized Catkin package.
* Initial commit
* Contributors: Daniel Smith
