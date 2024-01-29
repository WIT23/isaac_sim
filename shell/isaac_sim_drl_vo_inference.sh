tmux new -s drl_vo_navigation
tmux send-keys -t 0 "tmux split-window" C-m
echo "hello world"
#tmux send-keys -t 0 "cd /home/gr-agv-lx91/.local/share/ov/pkg/isaac_sim-2022.2.0" C-m
#tmux send-keys -t 0 "./python.sh practice.py" C-m
#
#cd ~/isaac_sim_ws/
#source devel/setup.bash
#roslaunch isaac_sim drl_vo_navigation.launch