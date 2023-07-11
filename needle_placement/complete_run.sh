#!/bin/bash
'''
Author: Selvakumar Nachimuthu
'''

roslaunch needle_placement auto_pcd_recorder.launch &

function cleanup {
    kill $launch_pid
    exit
}

trap cleanup SIGINT

launch_pid=$!  #get all pids we started inside this script

#wait until all the topics are up and running else sleep
while ! rostopic list > /dev/null 2>&1; do      
    sleep 1
done

#once the topics are up and running, start the predefined_jointstates.py
rosrun needle_placement predefined_jointstates.py

#once the predefined_jointstates.py is done, check if the model_recording_done is true
    #if true, kill all the nodes as we are done with the recording then start the model registration process
if [ "$(rosparam get /model_recording_done)" = "true" ]; then
    rosnode kill -a
else
    while [ "$(rosparam get /model_recording_done)" != "true" ]; do
        sleep 1
    done

    rosrun needle_placement model_registration_node_for_integration.py
fi

kill $launch_pid  #kill all



