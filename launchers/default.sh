#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app

#dt-exec python3 -m "seven.move"
roslaunch sevenexp move.launch
#roslaunch sevenexp turn.launch
#roslaunch eightexp moveeight.launch

# wait for app to end
dt-launchfile-join
