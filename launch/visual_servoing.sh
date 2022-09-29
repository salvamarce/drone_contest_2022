#!/bin/bash

session="visual_servoing"


if tmux has-session -t "$session"; then
  "Session $SESSION already exists."
  exit 0
fi


source ~/.bashrc
# start ./startProgram1 here
tmux new-session -d -s "$session" 


# start all other applications
# use -v for vertical split and -h for horizontal

tmux new-window -t "$session" "tmux set-option remain-on-exit on; tmux bind-key r respawn-pane; source ~/.bashrc; roslaunch px4 visual_servoing.launch" #tmux set-option remain-on-exit on; tmux bind-key r respawn-pane;

tmux select-window -t "$session":1

# start ./startProgram3 here
tmux split-window -v "sleep 2; source ~/.bashrc; roslaunch drone_contest_2022 action_server.launch "

tmux select-window -t "$session":1

# start ./startProgram3 here
tmux split-window -h "sleep 5; rostopic pub /takeOff/goal drone_contest_2022/takeOffActionGoal \"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  altitude_setpoint: 2.0
  duration: 5.0\" "

tmux select-window -t "$session":1



# start ./startProgram3 here
tmux split-window -v "sleep 7; roslaunch drone_contest_2022 target_path.launch "

tmux attach -t "$session"

