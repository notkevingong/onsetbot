#!/bin/bash
tmux new-session -d -s robot

tmux send-keys -t robot "source /opt/ros/humble/setup.bash && source ~/onset_ws/install/setup.bash && ros2 run onsetbot onset_gui" C-m
tmux split-window -h -t robot

tmux send-keys -t robot "source /opt/ros/humble/setup.bash && source ~/onset_ws/install/setup.bash && ros2 run onsetbot actuator_commands" C-m
tmux split-window -h -t robot

tmux send-keys -t robot "source /opt/ros/humble/setup.bash && source ~/onset_ws/install/setup.bash && ros2 run onsetbot stm32_bridge" C-m
tmux split-window -h -t robot

tmux send-keys -t robot "source /opt/ros/humble/setup.bash && source ~/onset_ws/install/setup.bash && ros2 topic echo /stm32_states" C-m
tmux split-window -h -t robot

tmux send-keys -t robot "source /opt/ros/humble/setup.bash && source ~/onset_ws/install/setup.bash && ros2 run onsetbot odrive_can_bridge" C-m
tmux split-window -h -t robot

tmux attach -t robot