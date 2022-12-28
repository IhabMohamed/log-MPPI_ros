# mission_control

The mission control package loads a mission from a .txt file, parses the given file and 
executes the mission. The node processes the mission step-by-step and interacts with other ROS
nodes to actually execute the command on the robot itself. For example, waypoint commands are 
forwarded to the navigation stack.

## Usage
```
roslaunch mission_control forest0_mission.launch
```

## Parameter
### ~mission_file (default: None)
Defines the path to the mission file that will be parsed and executed. See [Mission Definition](#commands) for more details on its content.

### ~command_timeout (default: 360)
Specify the maximum time in seconds to finish the execution of a single command. If the timeout is
reached, we reset the simulation, abort the data capture and retry/skip the command.

## Mission Definition <a name="commands"></a>
Missions are defined in a plain text file that is parsed line by line. Each line represents one
command. Lines that start with # are seen as comment and not parsed as command.

The following commands are currently supported in the parser and mission execution:

### Waypoint
```
wp: 2.0 4.0 90.0
```
Waypoint command start with <wp> followed by three floats (x y yaw) specifying the position in
meters and the orientation in degrees. The reference frame depends on the navigation stack, but 
should be the '/map' frame. If the execution of the waypoint is not successful, we skip it and
continue with the next command in the mission file.

### Random Waypoints
```
rd: 100 -5.0 5.0 -5.0 5.0
```
The random waypoints command samples N waypoints from a uniform distribution. The five floats after
<rd:> define the number of waypoints N and the range in x and y direction, the waypoints are sampled
from (N min_x max_x min_y max_y). The orientation is uniformly sampled from 0 to 360 degree. If the
execution of the sampled waypoint is not successful, we resample it and try again.
