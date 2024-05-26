#!/bin/bash
rosrun topic_tools throttle messages /camera/odom/sample 30.0 /odom
rostopic echo /odom