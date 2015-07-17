# TODO

* [ ] Teleop

# Recipe

* [x] agents dont like to see walls, especially up close.
* [ ] agents like to get closer to goal, when going forward, including forward turns.
  * [x] agents like to go straight forward, until goals are enabled.
* [ ] agents like to eat goals.

# Usage

```
roslaunch kulbu_base sim.launch
roslaunch kulbu_slam rat.launch use_rat_odom:=false topic_odom:=/kulbu/odometry/filtered
rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/kulbu/diff_drive_controller/cmd_vel

roslaunch rosbridge_server rosbridge_websocket.launch # ROSLibJS
node src/dqn.js
rqt_plot /dqn/reward:epsilon
rqt_plot /dqn/avg_reward
rostopic pub -1 /dqn/pause std_msgs/Bool -- '1'       # Pause DQN.
rostopic pub -1 /dqn/save std_msgs/String -- 'file'   # Save DQN as JSON.
rostopic pub -1 /dqn/load std_msgs/String -- 'file'   # Load DQN from JSON.
rostopic pub -1 /dqn/set_age std_msgs/String -- '"100000"' # FIXME: Datatype.
```

# Results

* Forward reward proportional by goal reward.
  * Learns to turn towards goal then sprint to exploit reward. Overshooting.


## `temporal_window`:

* 2: "Get away from that thing on the left"
* 4: "I'm turning around this other way regardless"

# Future work


## RatSLAM

* Discard experiences with *many* links.
* Quality metric for LV. Don't link low quality experiences.
* Reject closures with vastly different to magnetic reading.

## DQN
