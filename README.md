# Franka Learning from Demonstrations for ROS2

The original package [Franka Learning from Demonstration](https://github.com/platonics-delft/franka_learning_from_demonstrations) by Giovanni Franzese ([Platonics Delft](https://github.com/orgs/platonics-delft)).
This is the forked version for ROS2 and utilizing [PandaPy](https://github.com/JeanElsner/panda-py) backend.

## Installation of the python controller to perform learning from demonstration and object localization. You can install this also on another computer connected to the same network as the robot. 

```
mkdir -p lfd_ros2_ws/src
cd lfd_ros2_ws/src
git clone https://github.com/imitrob/franka_learning_from_demonstration_ros2

conda env create -f franka_learning_from_demonstrations/environment.yml
conda activate lfd
cd ..
colcon build --symlink-install
source install/setup.bash
```
ROS2 installs the packages to build folder. Make a symbolic links to use materials such as trajectories, configs, templates.
```
ln -s ~/lfd_ros2_ws/src/franka_learning_from_demonstrations/object_localization/cfg ~/lfd_ros2_ws/build/object_localization/cfg
ln -s ~/lfd_ros2_ws/src/franka_learning_from_demonstrations/object_localization/config ~/lfd_ros2_ws/build/object_localization/config
ln -s ~/lfd_ros2_ws/src/franka_learning_from_demonstrations/trajectory_data/trajectories ~/lfd_ros2_ws/build/trajectory_data/trajectories
ln -s ~/lfd_ros2_ws/src/gesture_actions/links ~/lfd_ros2_ws/build/gesture_actions/links
```
Please remember to source the workspace in every terminal you open.

## Let's start to learn some skills! 

### Send the robot to the home position and record the current template for the localization 

Send the robot to the home position. The robot will move in front of the robot and we can specify the z axis, i.e. the robot vertical height as one of the input to the script. For example, to send the robot to the home position at 0.4 m from the table, run: 
``` bash
ros2 launch skills_manager home_launch.py height:="0.4" 
```

Record the current template for the localization 
``` bash
ros2 launch object_localization record_template_launch.py name_template:="new_template"
```
The persistent `lfd_server` homes and reserves the robot only while the headless
template recorder freezes RGB-D and pose. Cropping happens after release.
### Kinesthetic Demonstration 

Be sure that the camera is running using: 

```bash
ros2 launch object_localization camera_launch.py
```

You can now record a demonstration with:

```bash
ros2 launch skills_manager record_skill_launch.py name_skill:='action__object'
```
This asks the persistent `lfd_server` to home, localize `object`, and record.
Pass `homing:="false"` to record from wherever the arm already is.

All the trajectories are saved in the folder `trajectory_data/trajectories/` with the name you gave to the skill.
This folder is a ros package that is used to save and load the demonstrations and save them. We used this folder to have a ligher repository and save all the demonstration in this other one. 

During demonstration the robot is recording the Cartesian pose of the robot and the current image from the camera for each time step (the control freqeuncy is set to 20 Hz).
During execution, the robot tracks the recorded trajectory while trying to correct for the discrepancy between the current image at that timestep and the one recorded during demonstration.
This increases the reliability of critical picking and insertion tasks. 

### Execute Learned Skill

For executing the skill you can run the active localizer in one terminal and the skill manager in another terminal. 

```bash
ros2 launch object_localization localization_launch.py
```

```bash
ros2 launch skills_manager play_skill_launch.py name_skill:='action__object'
```

### Recover from pose divergence without restarting

Pose divergence pauses the active operation; it does not discard the trajectory.

- **Small error:** when position error exceeds 5 cm or orientation error exceeds
  0.05 rad (2.9°), stop advancing and wait for both errors to return within tolerance.
- **Large error:** above 10 cm or 0.1 rad (5.7°), or after 5 seconds without tracking,
  hold the measured pose and wait for explicit resume. Contact alone never pauses,
  because placing, inserting and spiral search touch on purpose; a stuck contact
  pauses through the 5 s timeout. Resume is refused while libfranka reports contact.
- A recorded waypoint that rotates more than 0.1 rad in one step is a command, not
  divergence: the robot turns to it in 0.025 rad steps without pausing.
- Clear the obstruction and check the path before resuming. Press **Space** while
  the keyboard listener is active, or call:

  ```bash
  ros2 service call /lfd/resume_motion std_srvs/srv/Trigger '{}'
  ```

  Notebook callers can use `robot.resume_motion()` from another thread. A successful
  response accepts the recovery request; it does not mean recovery has finished.
- Resume moves in a slow straight Cartesian line from the current measured pose to the
  saved waypoint, then continues the same operation. IK only checks that the waypoint
  itself is reachable; it does not plan the path. Gripper actions are not replayed.
  An unreachable target or another tracking timeout leaves the operation paused.
  Press **e** to end a paused skill execution, or **Esc** / cancel the action.
- A pause during active localization discards that localization result and localizes
  again from the recovered pose.
- Other robot operations remain rejected while recovery owns the robot.
  `/lfd/operation_status` reports `waiting_for_tracking` (after 1 s), `paused_tracking`,
  or `recovering`; an idle server whose controller stopped reports
  `controller_unavailable`. The skills dashboard shows these phases.

No automatic trajectory reversal, obstacle avoidance or collision checking is provided.
Invalid poses and controller faults, including libfranka reflexes, still fail the
operation. Hand-guided recording does not treat deliberate manual movement as tracking
failure or command roll correction.

Known issues, not fixed yet:
- Keys `m`, `n`, `c`, `o` (and Desk buttons mapped to them) move the robot or gripper
  from the keyboard thread. An exception there stops the listener, so Space and Esc
  stop working for the rest of that operation.
- The keyboard listener is global: Space typed in any window resumes a pause. Outside
  a pause, Space toggles `pause` instead.
- Esc, cancel and failed operations call `gripper.stop()`, which releases grasp force,
  so a carried object can drop.

Calibration attributes on `Panda`: `attractor_distance_threshold` (m),
`tracking_angle_tolerance` (rad), `position_recovery_limit` (m),
`orientation_recovery_limit` (rad), `tracking_timeout` (s), and `recovery_dt` (s).
Default recovery pacing is 0.05 s per step, with at most 2 mm translation and
0.025 rad rotation per step. Validate the thresholds, speed, payload, and contact
settings on the robot before use; hardware-free tests do not establish physical safety.

### Commands during demonstration
During demonstration or during execution, it is possible to give feedback to the learned skill using the computer keyboard. 

- Press `e` to stop the recording.

### Pause the demonstration 
- Press 'Space tab' to pause/start the demonstration. Be carefull that when you start again you are close to a point you where when you paused it

#### Gripper commands:

- Press `c` to close the gripper.
- Press `o` to open the gripper.

#### Camera feedback:

- Press `k` to add a label to enable the local camera feedback from that point on.
- Press `l` to add a label to disable the local camera feedback from that point on.

#### Haptic feedback:

- Press `z` to add a label to enable the haptic feedback from that point on
- Press `x` to add a label to disable the haptic feedback from that point on

#### Stiffen the orientation of the end-effector:

- Press `m` makes the robot stiffen the orientation of the end-effector in the current configuration
- Press `n` makes the robot orientation stiffness to be null again


The motivation for explicitly labeling or disabling the haptic and local camera feedback is that during a long trajectory, the user can explicitly teach the robot to use or not use that skill. For example, it makes sense to have the haptic feedback only active when performing insertion tasks, so that the robot will not start spiraling when external forces are perceived but they are not due to critical insertion tasks. It is worth noticing that if no insertion task is performed, in case of large force, the robot would temporarily stop the execution of the motion until the disturbance is gone. This increases safety when interacting with humans.

### Give interactive Corrections during execution when running each skill separately and without the localizer 
```bash
ros2 launch skills_manager play_skill_launch.py localize_box:=false name_skill:='skill'
```

#### Directional feedback:

- Press `w` to locally shift the trajectory in positive x in robot frame 
- Press `s` to locally shift the trajectory in negative x in robot frame 
- Press `a` to locally shift the trajectory in positive y in robot frame 
- Press `d` to locally shift the trajectory in negative y in robot frame 
- Press `u` to locally shift the trajectory in positive z in robot frame 
- Press `j` to locally shift the trajectory in negative z in robot frame 

This feedback is useful when, after the demonstration, the robot will, for example, not press the button hard enough. However, by pressing `j`, the trajectory is locally modified, and when executing again, the robot will apply enough force.

### Speed feedback:
You can make the robot to go faster locally by 
- Press `f` to lcoally make the motion faster. 
 
 This interactive feedback can be used to make the robot faster during the execution of the motion. 

You can also correct when the gripper has to close or if you want to active/disactivate the haptic feedback or the camera feedback, see previous Section. 

At the end of every play back, the computer will ask if to overwrite the old trajectory or not. If the changes are accepted, the robot will remember the feedback in the next executions.

That's it! Have fun!
