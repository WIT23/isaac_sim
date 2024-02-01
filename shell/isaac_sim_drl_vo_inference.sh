session="isaac_sim_drl_vo_navigation"

tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window "isaac_sim"
tmux send-keys -t $session:$window "cd ~/.local/share/ov/pkg/isaac_sim-2022.2.0/" C-m
tmux send-keys -t $session:$window "./python.sh environment.py" C-m

window=1
tmux new-window -t $session:$window -n "drl_vo_navigation"
tmux send-keys -t $session:$window "sleep 15s" C-m
tmux send-keys -t $session:$window "cd ~/isaac_sim_ws/" C-m
tmux send-keys -t $session:$window "source devel/setup.bash" C-m
tmux send-keys -t $session:$window "roslaunch isaac_sim drl_vo_navigation.launch" C-m
