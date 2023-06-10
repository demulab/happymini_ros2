#!/bin/bash
tab="--tab-with-profile=Unnamed --command"
window="--window-with-profile=Unnamed --command"

cd $HOME/colcon_ws/src/happymini_ros2/bash_launcher/bashes

gnome-terminal \
    $tab 'bash -c "sleep 1.0 ; ./bringup.bash"'\
    $tab 'bash -c "sleep 1.2 ; ./navigation2.bash"'\
    $tab 'bash -c "sleep 1.4 ; ./detect_left_right.bash"'\
    $tab 'bash -c "sleep 1.4 ; ./chaser_python.bash"'\
    $tab 'bash -c "sleep 1.6 ; ./voice.bash"'\
    $tab 'bash -c "sleep 1.5 ; ./open_manipulator_x.bash"'\
    $tab 'bash -c "sleep 1.5 ; ./grasp_bag.bash"'\
