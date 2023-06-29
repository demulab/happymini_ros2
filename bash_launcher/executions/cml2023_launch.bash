#!/bin/bash
tab="--tab-with-profile=bash_launcher --command"
window="--window-with-profile=bash_launcher --command"

cd $HOME/main_ws/src/happymini_ros2/bash_launcher/bashes

gnome-terminal \
    $tab 'bash -c "sleep 1.0 ; ./bringup.bash;bash"'\
    $tab 'bash -c "sleep 1.2 ; ./navigation2.bash;bash"'\
    $tab 'bash -c "sleep 1.4 ; ./detect_left_right.bash;bash"'\
    $tab 'bash -c "sleep 1.4 ; ./chaser_python.bash;bash"'\
    $tab 'bash -c "sleep 1.6 ; ./voice.bash;bash"'\
    $tab 'bash -c "sleep 1.5 ; ./open_manipulator_x.bash;bash"'\
    $tab 'bash -c "sleep 3.0 ; ./grasp_bag.bash;bash"'\
    $tab 'bash -c "sleep 1.5 ; ./rs_launch.bash;bash"'\
    $tab 'bash -c "sleep 7.0 ; ./object_detection_tf_origin.bash;bash"'\
    $tab 'bash -c "sleep 3.0 ; ./stand_in_line.bash;bash"'\
