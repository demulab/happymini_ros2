#!/bin/bash
tab="--tab-with-profile=bash_launcher --command"
window="--window-with-profile=bash_launcher --command"

cd $HOME/main_ws/src/happymini_ros2/bash_launcher/bashes

gnome-terminal \
    $tab 'bash -c "sleep 1.0 ; ./bringup.bash;bash"'\
    $tab 'bash -c "sleep 1.2 ; ./navigation2.bash;bash"'\
    $tab 'bash -c "sleep 1.4 ; ./speech_to_text.bash;bash"'\
    $tab 'bash -c "sleep 1.4 ; ./name_detect.bash;bash"'\
    $tab 'bash -c "sleep 1.6 ; ./rs_launch.bash;bash"'\
    $tab 'bash -c "sleep 1.5 ; ./v4l2_camera_node.bash;bash"'\
    $tab 'bash -c "sleep 1.5 ; ./attribute_recog_node.bash;bash"'\
    $tab 'bash -c "sleep 1.5 ; ./object_detection_tf.bash;bash"'\
    $tab 'bash -c "sleep 1.5 ; ./person_detector.bash;bash"'\
    $tab 'bash -c "sleep 1.5 ; ./approach_person.bash;bash"'\
