# Arena Evaluation
The Arena Evaluation package provides tools to record, evaluate, and plot navigational metrics to evaluate ROS navigation planners. It is best suited for usage with our [arena-rosnav repository](https://github.com/ignc-research/arena-rosnav) but can also be integrated into any other ROS-based project. 

It consists of 3 parts:
- [Data recording](#01-data-recording)
- [Data transformation and evaluation](#02-data-transformation-and-evaluation)
- [Plotting](#03-plotting)
## Prerequisites

```
pip install scikit-learn seaborn pandas matplotlib
```

## Integration into your project
To integrate this evaluation class into your project, clone it into your catkin workspace (or put it into your .rosinstall file). 
Then, inside your catkin workspace do:

```
rosws update
catkin_make
```

After successfully making, you can use the evaluation class by including the following part into the roslaunch file from which you will start your simulation:

```
  <!-- data recorder -->
  <group if="$(eval arg('use_recorder') == true)">
    <node pkg="arena-evaluation" name="data_recorder_node" type="data_recorder_node.py" />
    <param name="local_planner" value="$(arg local_planner)"/>
    <param name="waypoint_generator" value="$(arg waypoint_generator)"/>
    <param name="record_only_planner" value="$(arg record_only_planner)"/>
    <param name="scenario_file" value="$(arg scenario_file)"/>
    <param name="model" value="$(arg model)"/>
  </group>

```


## 01 Data Recording
To record data as csv file while doing evaluation runs set the flag `use_recorder:="true"` in your `roslaunch` command. For example:

```
workon rosnav
roslaunch arena_bringup start_arena_gazebo.launch world:="aws_house" scenario_file:="aws_house_obs05.json" local_planner:="teb" model:="turtlebot3_burger" use_recorder:="true"
```

The data will be recorded in `.../catkin_ws/src/forks/arena-evaluation/01_recording`.
The script stops recording as soon as the agent finishes the scenario and stops moving or after termination criterion is met. Termination criterion as well as recording frequency can be set in `data_recorder_config.yaml`.

```
max_episodes: 15 # terminates simulation upon reaching xth episode
max_time: 1200 # terminates simulation after x seconds
record_frequency: 0.2 # time between actions recorded
```

After doing all evaluation runs please remember to push your csv files or gather them in `/01_recording`.

NOTE: Leaving the simulation running for a long time after finishing the set number of repetitions does not influence the evaluation results as long as the agent stops running. Also, the last episode of every evaluation run is pruned before evaluating the recorded data.

NOTE: Sometimes csv files will be ignored by git so you have to use git add -f <file>. We recommend using the code below.
```
roscd arena-evaluation
git add -f .
git commit -m "evaluation run"
git pull
git push
```

## 02 Data Transformation and Evaluation
After finishing all the evaluation runs, recording the desired csv files and allocating them in `/01_recording` you can run the `get_metrics.py` script in `/02_evaluation`.
This script will evaluate the raw data recorded from the evaluation runs und summarize them in a compact dataset for each map, planner and scenario, which are all stored in one single JSON file with the following naming convention: `data_<timestamp>.json`. During this process all the csv files will be moved from `/01_recording` to `/02_evaluation` into a directory with the naming convention `data_<timestamp>`. The JSON file will be stored in `/02_evaluation`.

Some configurations can be set in the `get_metrics_config.yaml` file. Those are:
- `robot_radius`: dictionary of robot radii, relevant for collision measurement
- `time_out_treshold`: treshold for episode timeout in seconds
- `collision_treshold`: treshold for allowed number of collisions until episode deemed as failed

NOTE: Do NOT change the `get_metrics_config_default.yaml`!

We recommend using the code below:
```bash
workon rosnav && roscd arena-evaluation/02_evaluation && python get_metrics.py
```

NOTE: If you want to reuse csv files, simply move the desired csv files from the data directory to `/01_recording` and execute the `get_metrics.py` script again.

## 03 Plotting
The `get_plots.py` script grabs all `data.json` files located in `/02_evaluation` and moves them to `/03_plotting/data`. During the process the last in order JSON file from the grabbed files will be deemed as "most recent" file. If no file was grabbed, the last data.json used for plotting will remain the "most recent" file. Alternatively, it's possible to specify a `data.json` to be used for plotting. To specify a dataset set the following keys in the `get_plots_config.yaml`:

```
specify_data: true
specified_data_filename: <your_dataset>.json
```

For runnning the script recommend using the code below:
```bash
workon rosnav && roscd arena-evaluation/03_plotting && python get_plots.py
```

### Mandatory fields:
- `labels`
- `color_scheme`

Make sure for those fields **all** your local planner or planner-waypoint-generator combinations with the robot they were used on are defined. Examples:
- labels:
    - rlca_jackal: RLCA
    - rlca_turtlebot3_burger: RLCA
- color_scheme:
    - rlca_jackal

See the documentation [here](docs/fields.md) for an explanation of the possible parameters fields.

It is possible to plot qualitative and quantitative metrics. For a detailed description and examples see the documentation [here](docs/plotting_metrics.md)
# Mesure complexity of you map
1. run: `roscd arena-evaluation`
2. run: `python world_complexity.py --image_path {IMAGE_PATH} --yaml_path {YAML_PATH} --dest_path {DEST_PATH}`

with:\
 IMAGE_PATH: path to the floor plan of your world. Usually in .pgm format\
 YAML_PATH: path to the .yaml description file of your floor plan\
 DEST_PATH: location to store the complexity data about your map

Example launch:
```bash
python world_complexity.py --image_path ~/catkin_ws/src/forks/arena-tools/aws_house/map.pgm --yaml_path ~/catkin_ws/src/forks/arena-tools/aws_house/map.yaml --dest_path ~/catkin_ws/src/forks/arena-tools/aws_house
```
