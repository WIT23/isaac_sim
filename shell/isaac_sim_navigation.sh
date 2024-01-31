session="isaac_sim_navigation"

tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window "isaac_sim"
tmux send-keys -t $session:$window "cd ~/.local/share/ov/pkg/isaac_sim-2022.2.0/" C-m
tmux send-keys -t $session:$window "./python.sh environment.py" C-m

window=1
tmux new-window -t $session:$window -n "navigation"
tmux new-window -t $session:$window -n "navigation"
tmux send-keys -t $session:$window "cd ~/isaac_sim_ws/" C-m
tmux send-keys -t $session:$window "source devel/setup.bash" C-m
tmux send-keys -t $session:$window "roslaunch isaac_sim navigation.launch" C-m
