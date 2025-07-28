#!/bin/bash

# launch serial_link 
tmux new-session -d -s fredo1 -n mavros
tmux send-keys -t fredo1:0 "./fo1_serial/build/serial_link $1" C-m

# launch serial_com
tmux split-window -h -t fredo1:0
tmux send-keys -t fredo1:0.1 './fo1_serial/build/serial_com' C-m

# attach 
tmux attach -t fredo1:0.1
