Flex BT Turtlebot
================================

## Introduction

Implementations of the [ROS 2] and [FlexBE]-based [Flexible Behavior Trees] for use with the [Kobuki Turtlebot2].

This repository contains code that demonstrates the open-source ROS 2 [Flexible Behavior Trees] system using the
CHRISLab setup of [Kobuki Turtlebot2].

The demonstrations include installation and setup instructions for the [FlexBE System].

> NOTE:  As of 18-Oct-22 with the ROS Humble release an issue with the Navigation 2 costmap causes issue with
> default DDS implementation Fast DDS (ros-humble-rmw-fastrtps-cpp).  
> Switching to `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` seems to fix issue.
> Track https://github.com/ros-planning/navigation2/issues/3014 , 3018, and 2489 for more information.


----------------------

This package has a number of dependencies.  The quickest, and easiest method to get a demonstration up and running, is to follow the setup instructions below.  

This package has a number of dependencies.  

For CNU Robotics work, we typically use [CHRISLab Install] scripts to handle workspace
setup, install, and build.  The below repos are including in the install scripts there.


Quickly skim this README before installing or running anything:

This demonstration makes use of the following repositories:

<pre>
- git: {local-name: src/flexbe_app,               uri: 'https://github.com/FlexBE/flexbe_app.git',                    version: ros2-devel }
- git: {local-name: src/flexbe_behavior_engine,   uri: 'https://github.com/FlexBE/flexbe_behavior_engine.git',        version: ros2-devel }
- git: {local-name: src/flexible_navigation,      uri: 'https://github.com/FlexBE/flexible_navigation.git',           version: ros2-devel }
- git: {local-name: src/flexible_behavior_trees,  uri: 'https://github.com/FlexBE/flexible_behavior_trees.git',       version: ros2-devel }
- git: {local-name: src/flex_bt_turtlebot2_demo,  uri: 'https://github.com/CNURobotics/flex_bt_turtlebot2_demo.git',  version: ros2-devel }
- git: {local-name: src/ball_detector,            uri: 'https://github.com/CNURobotics/ball_detector.git',            version: ros2-devel }
- git: {local-name: src/chris_ros_turtlebot2,     uri: 'https://github.com/CNURobotics/chris_ros_turtlebot2.git',     version: ros2-devel }
- git: {local-name: src/chris_world_models,       uri: 'https://github.com/CNURobotics/chris_world_models.git',       version: ros2-devel }
- git: {local-name: src/image_pipeline,           uri: 'https://github.com/CNURobotics/image_pipeline.git',           version: humble-image-flip }
- git: {local-name: src/openni2_camera,           uri: 'https://github.com/CNURobotics/openni2_camera.git',           version: astra-humble }
</pre>

At this current stage, some Kobuki Turtlebot2 related packages are not released in ROS2 binary form, so we are using the following for source builds:
<pre>
- git: {local-name: src/sophus,                   uri: 'https://github.com/stonier/sophus.git',                            version: release/1.2.x }
- git: {local-name: src/ecl_tools,                uri: 'https://github.com/stonier/ecl_tools.git',                         version: devel }
- git: {local-name: src/ecl_lite,                 uri: 'https://github.com/stonier/ecl_lite.git',                          version: devel }
- git: {local-name: src/ecl_core.git,             uri: 'https://github.com/stonier/ecl_core.git',                          version: devel }
- git: {local-name: src/kobuki_ros,               uri: 'https://github.com/CNURobotics/kobuki_ros.git',                    version: humble-test }
- git: {local-name: src/kobuki_core,              uri: 'https://github.com/kobuki-base/kobuki_core.git',                   version: devel }
- git: {local-name: src/kobuki_ros_interfaces,    uri: 'https://github.com/kobuki-base/kobuki_ros_interfaces.git',         version: devel }
</pre>


Install in the `src` folder of your WORKSPACE_ROOT, and from the

<pre>
colcon build
. setup.bash
</pre>

> NOTE: Anytime you build new packages, you need to re-run the setup.bash script inside the workspace root.  
> Anytime you change a Python script or launch file, you need to re-run `colcon build` from the WORKSPACE_ROOT folder, but you only need to re-source `. setup.bash` when the package information and folders change.

On a new build, you must install the Java Script package that the FlexBE app will require:
<pre>
ros2 run flexbe_app nwjs_install
</pre>

> Note: With colcon, this will need to be re-run anytime the `install` folder is deleted as it is installed relative to the `flexbe_app` package.

 This version presumes use of the [FlexBE App] for the operator interface, which depends on states and behaviors that are exported as part of individual `package.xml`.


## Operation
---------

A number of start up scripts are provided in `flex_bt_turtlebot2_demo_bringup`

For hardware demonstration:
    <pre>
    export USE_SIM_TIME=False
    ros2 run flex_bt_turtlebot2_demo_bringup hw-tmux
    ros2 run flex_bt_turtlebot2_demo_bringup onboard-tmux
    </pre>
    And on OCS computer,
    <pre>
    ros2 run flex_bt_turtlebot2_demo_bringup ocs-tmux
    </pre>
    For onboard hw, the `tmux` is preferred.
    For OCS, either `ocs-tmux` or `launch-ocs` bash script is available.


For basic simulation demonstration,
    <pre>
    export USE_SIM_TIME=True
    ros2 run flex_bt_turtlebot2_demo_bringup launch-sim  
    ros2 run flex_bt_turtlebot2_demo_bringup launch-onboard
    ros2 run flex_bt_turtlebot2_demo_bringup launch-ocs  
    </pre>

    These may be started up on a single computer, or multiple computers if using networked simulation.

There are also associated `tmux` versions if preferred.

Using the above scripts will start the required software for demonstration.  See those scripts for details.

These scripts also make use of the following environment variables:
<pre>
export WORLD_MODEL=
export LOCALIZATION=slam # (e.g. slam, amcl, or cartographer)
export USE_SIM_TIME=true # (or false as appropriate)
export WORLD_MODEL=gazebo_creech_world #( if not set by launch, see the `chris_world_models` package for more world model setups.)
</pre>

Typically the `setup.bash` is created by the setup script created during our standard
install process [CHRISLab Install] .

>NOTE: With both tmux and launch scripts, the terminal will close if the started
> nodes completely shutdown. Manually starting each script as described below may be warranted for debugging.

### Manual start up of simulation demonstration:

To launch the simulation in separate terminals for debugging, use these commands in each terminal:

<pre>
# Simulation
ros2 launch chris_world_models ${WORLD_MODEL:=gazebo_creech_world}.launch.py use_sim_time:=True
ros2 launch chris_ros_turtlebot2 turtlebot_gazebo.launch.py use_sim_time:=True

# Onboard
# To use other (e.g. fake, amcl, or cartographer, set LOCALIZATION environment variable (e.g. export LOCALIZATION=amcl)
# To use other (e.g. flex, flex_multi, or flex_four_level, set FLEX_NAV_SETUP environment variable (e.g. export LOCALIZATION=flex)
ros2 launch flex_bt_turtlebot2_demo_bringup "${LOCALIZATION:=fake}.launch.py" use_sim_time:=True
ros2 launch simple_ball_detector ball_detector.launch.py use_sim_time:=True
ros2 launch flex_bt_turtlebot2_demo_bringup nav2_turtlebot.launch.py use_sim_time:=True
ros2 launch flexbe_onboard behavior_onboard.launch.py use_sim_time:=True

# Operator Control Station (OCS)
ros2 launch flex_bt_turtlebot2_demo_bringup rviz.launch.py use_sim_time:=True
ros2 run flexbe_mirror behavior_mirror_sm --ros-args --remap __node:="behavior_mirror" -p use_sim_time:=True
ros2 run flexbe_widget be_launcher --ros-args --remap __node:="behavior_launcher" -p use_sim_time:=True
ros2 run flexbe_app run_app --ros-args --remap __node:="flexbe_app" -p use_sim_time:=True

</pre>

-----

  > From Nav2 instructions: After starting, the robot initially has no idea where it is using SLAM techiques. 
  > By default, Nav2 waits for you to give it an approximate starting position.
  > Take a look at where the robot is in the Gazebo world, and find that spot on the map. 
  > Set the initial pose by clicking the “2D Pose Estimate” button in RViz, and then 
  > clicking on the map in that location. 
  > You set the orientation by dragging forward from the down click.

  > Note: 30-June-22 Humble release did not update static map layer, which prevented planning with AMCL.  
  > Disable static layer in params/nav2_turtlebot_params.yaml if you have issues with AMCL.

### Visualization

   Displays a standard view of transforms of Turtlebot2, sensor data, with maps, and paths displayed

   > NOTE: This is typically started by the `launch-ocs` script described above.

   * `ros2 launch flex_bt_turtlebot2_demo_bringup rviz.launch.py`

      This custom version adds the robot model, path, and global cost map to the default cartographer setup.
          * A `Path` to the RViz display and set the topic to `/high_level_planner/plan`
          * A `RobotModel` (uses the `/robot_description` topic)
          * A `Map` showing the topic `/global_costmap/costmap`
          * The `2D Goal Pose` publishes to the topic `/flex_nav_global/goal`

   You may want to add a `Camera` pointing to the `/ball_detector/image` to see the marked balls.
   These are relatively small and may not be that useful.
   It might be better to periodically run:
     * `ros2 run image_view image_saver --ros-args --remap image:=/ball_detector/image` or
     * `ros2 run image_view image_view --ros-args --remap image:=/ball_detector/image`  to see full size

   You can also add a `MarkerArray` with the topic `/ball_detector/ball_markers`.

   In the topic, you will need to set the Reliability to "Best Effort" for the topics to be received.
   (ROS 2 defaults to publishing sensor data as "Best Effort" to save band width for large data systems.)

  RViz and localization may generate errors until the "2D Pose Estimate" is set via RViz if using AMCL.

  > NOTE: With AMCL localization, RViz and localization may generate errors until the "2D Pose Estimate" is set via RViz.

  > NOTE: The simulation automatically starts RViz alongside the ROS 2 Cartographer command; verify topic settings as needed if using that version.

  > Note: The RViz configuration for the map topic uses `Transient Local` for durability. 
  > If the map server is using `Volatile`, then you will need to change to `Volatile` in order to visualize the global map.

### FlexBE Operation

After startup, all control is through the FlexBE App operator interface and RViz.  

First load the desired behavior through the `FlexBE Behavior Dashboard` tab.
  * `Turtlebot2 Nav2 BT`
    * Simplest example allows user to input goal via FlexBE state and RViz

  * `Turtlebot2 Nav2 Multi-BTs`
    * Basic navigation using multiple separate BTs

  * `Turtlebot2 Patrol and Investigate`
    * Allows user to input location of charging station and multiple waypoints to patrol using RVIZ.
    * Patrols and periodically moves to recharge station
    * This uses a battery status topic.  A simple simulated battery drain and charge can be run with:
      * `ros2 launch flex_bt_turtlebot2_demo_bringup turtlebot_sim_battery.launch.py use_sim_time:=True `
    * Looks for detected ball messages and changes behavior based on detected color
       * Be sure ball detector is launched and balls are added to environment
       * To drop some balls at random locations in simulation

          * `ros2 launch chris_world_models creech_random_balls.launch.py`

            The launch file specifies 8 blue, 2 red, and 4 green balls dropped randomly in Creech world.

            You can edit the launch file to change quantities, or use the `add_balls.launch.py` and edit the `param/balls.csv` file
            if you want to start with specified locations.

Execute the behavior via the `FlexBE Runtime Control` tab.
* The system requires the operator to input a `2D Nav Goal` via the `RViz` screen
  * If the system is in `low` autonomy or higher, the system will request a global plan as soon as the goal is received
  * If the autonomy level is `off`, then the operator will need to confirm receipt by clicking the `done` transition.
* After requesting a path to the goal, the resulting plan will be visualized in the `RViz` window.  
  * If the system is not in full autonomy mode, the operator must confirm that the system should execute the plan via the `FlexBE UI`  
  * If the operator sets the `Runtime Executive` to `full` autonomy, the plan will automatically be executed.  
  * In less than `full` autonomy, the operator can request a recovery behavior at this point.
* Once execution of this plan is complete, `FlexBE` will seek permission to continue planning
  * In `full` autonomy, the system will automatically transition to requesting a new goal
  * In any autonomy level less than `full`, the system will require an operator decision to continue

Whenever a plan is being executed, the `FlexBE` state machine transitions to a concurrent node that uses on line  planners to refine the plans as the robot moves, and also monitors the Turtlebot bumper status for collision.  
The operator can terminate the execution early by selecting the appropriate transition in the `FlexBE UI`.  
If this low level plan fails, the robot will request permission to initiate a recovery behavior; 
in `full` autonomy the system automatically initiates the recovery.

---

## Publications

Please use the following publication for reference when using Flexible Behavior Trees:

- Joshua M. Zutell, David C. Conner, and Philipp Schillinger, ["Flexible Behavior Trees: In search of the mythical HFSMBTH for Collaborative Autonomy in Robotics"](https://doi.org/10.48550/arXiv.2203.05389), March 2022.


### Further Publications for FlexBE

- Joshua Zutell, David C. Conner and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation ,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon, April 2022.

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Stefan Kohlbrecher et al. ["A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots."](http://dx.doi.org/10.3389/frobt.2016.00031) Frontiers in Robotics and AI 3 (2016): 31.

- Alberto Romay et al., [“Collaborative autonomy between high-level behaviors and human operators for remote manipulation tasks using different humanoid robots,”](http://dx.doi.org/10.1002/rob.21671) Journal of Field Robotics, September 2016.

---

[FlexBE]: https://flexbe.github.io
[FlexBE App]: https://github.com/FlexBE/flexbe_app
[FlexBE System]: https://github.com/FlexBE/flexbe_behavior_engine
[Flexible Behavior Trees]: https://github.com/FlexBE/flexible_behavior_trees
[Kobuki ROS]: https://github.com/kobuki-base/kobuki_ros
[Koubki ROS Interfaces]: https://github.com/kobuki-base/kobuki_ros_interfaces
[laser filters]: https://github.com/ros-perception/laser_filters
[ROBOTIS Turtlebot3]: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
[ROS 2]: https://docs.ros.org/en/foxy/index.html
[ROS 2 Installation]: https://docs.ros.org/en/foxy/Installation.html
[ROS 2 Navigation2]: https://navigation.ros.org/
[ROS 2 Cartographer]: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html
[urg_node]: https://github.com/ros-drivers/urg_node
[CHRISLab Install]: https://github.com/CNURobotics/chris_install
