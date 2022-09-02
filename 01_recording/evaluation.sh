"""
USAGE:
Run this command in your terminal (if you use bash shell instead of zsh shell, you need to replace zsh with bash)
$ zsh <path_to_this_file>
"""

export PATH="$HOME/.poetry/bin:$PATH"
export PYTHONPATH=""
source /opt/ros/noetic/setup.zsh

source /home/ducanor/arena_ws/devel/setup.zsh
export PYTHONPATH=/home/ducanor/arena_ws/src/arena-rosnav:/home/ducanor/arena_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

export WORKON_HOME=/home/ducanor/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh

workon rosnav
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1" local_planner:="rosnav" scenario_file:="eval/obstacle_map1_obs05.json" use_recorder:="true"
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1" local_planner:="mpc" scenario_file:="eval/obstacle_map1_obs05.json" use_recorder:="true"
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1" local_planner:="dwa" scenario_file:="eval/obstacle_map1_obs05.json" use_recorder:="true"
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1" local_planner:="teb" scenario_file:="eval/obstacle_map1_obs05.json" use_recorder:="true"
