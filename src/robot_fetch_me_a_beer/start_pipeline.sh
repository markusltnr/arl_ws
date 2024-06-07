#!/bin/bash

session="tiago"

tmux new-session -d -s $session
tmux set mouse on

window=0
tmux rename-window -t $session:$window 'tiago launch'

tmux split-window -h
tmux split-window -h
tmux split-window -h
tmux select-layout even-horizontal
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

tmux select-pane -t 1
tmux send-keys 'source /home/user/exchange/arl_ws/devel/setup.bash' C-m
tmux send-keys 'roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true end_effector_left:=pal-gripper end_effector_right:=pal-gripper world:=pick_coca gzpose:="-x 1.95 -y -2.3 -z 0.0 -R 0.0 -P 0.0 -Y -1.57" base_type:=omni_base' C-m

tmux select-pane -t 2
tmux send-keys  "rosservice call /controller_manager/switch_controller \"start_controllers: 
- ''
stop_controllers:
- 'head_controller'
- 'arm_left_controller'
- 'arm_right_controller'
- 'torso_controller'
- 'whole_body_kinematic_controller'
strictness: 0\""

tmux select-pane -t 3
tmux send-keys 'roslaunch tiago_dual_wbc tiago_dual_wbc.launch'

tmux select-pane -t 4 
tmux send-keys 'roslaunch tiago_dual_wbc push_reference_tasks.launch source_data_arm:=topic_reflexx_typeII source_data_gaze:=topic'

tmux select-pane -t 5
tmux send-keys 'rosrun pal_wbc_utils push_torso_height_ref _before_task_id:=position_arm_right_tool_link _link_name:=torso_lift_link  _reference_height:=1.1'

tmux select-pane -t 0
tmux send-keys 'cd /home/user/exchange/arl_ws && source devel/setup.bash' C-m
tmux send-keys 'rosrun robot_fetch_me_a_beer pipeline.py'

tmux attach
