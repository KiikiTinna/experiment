#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init


# launching app

#######Experiments 7 and 8  
#dt-exec python3 -m "seven.move"
#roslaunch sevenexp lilimove.launch
#roslaunch sevenexp turn.launch
roslaunch eightexp moveeight.launch


####Object_detection
#rosrun my_package camera_reader_node.py
#roslaunch object_detection see_ducks.launch

# wait for app to end
dt-launchfile-join



