alias nb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias tg='ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py'
alias tc='ros2 launch turtlebot3_cartographer cartographer.launch.py'
alias tt='ros2 run turtlebot3_teleop teleop_keyboard'

export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger


# Basic builds
alias cb='colcon build'
alias cbs='colcon build --symlink-install'

# Clean builds
alias cbc='colcon build --cmake-clean-first'
alias cbsc='colcon build --symlink-install --cmake-clean-first'

# Package-specific builds
alias cbp='colcon build --packages-select'
alias cbpc='colcon build --packages-select --cmake-clean-first'
alias cbps='colcon build --packages-select --symlink-install'
alias cbpsc='colcon build --packages-select --symlink-install --cmake-clean-first'

# Up-to builds
alias cbu='colcon build --packages-up-to'
alias cbuc='colcon build --packages-up-to --cmake-clean-first'
alias cbus='colcon build --packages-up-to --symlink-install'
alias cbusc='colcon build --packages-up-to --symlink-install --cmake-clean-first'


# Core ROS 2 Commands 
alias r='ros2'
alias rl='ros2 launch'
alias rr='ros2 run'

# Topics
alias rt='ros2 topic'
alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias rth='ros2 topic hz'

# Nodes
alias rn='ros2 node'
alias rnl='ros2 node list'
alias rni='ros2 node info'

# Services
alias rs='ros2 service'
alias rsl='ros2 service list'
alias rsc='ros2 service call'

# Parameters
alias rp='ros2 param'
alias rpl='ros2 param list'
alias rpg='ros2 param get'
alias rps='ros2 param set'

# Build Commands
alias cb='colcon build'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --packages-select'

# Navigation
alias cw='cd ~/ros2_ws'
alias cs='cd ~/ros2_ws/src'

# Source
alias sw='source install/setup.bash'
alias sr='source ~/.bashrc'

# Clean/Kill
alias kk='killall -9 gzserver gzclient'
alias cc='rm -rf build/ install/ log/'

# Launch Tools
alias rv='rviz2'
alias gz='gazebo'

