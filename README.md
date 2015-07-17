# ROS + DQN + RatSLAM

Training Deep Q-Learning neural network based on ConvNetJS demo to use sonar range sensors and RatSLAM goals.

* [ConvNetJS - demo](http://cs.stanford.edu/people/karpathy/convnetjs/demo/rldemo.html)
* [ROSLibJS](https://github.com/RobotWebTools/roslibjs/)
* [RatSLAM fork](https://github.com/mryellow/ratslam)

# TODO

* [ ] Teleop

# Recipe

* agents dont like to see walls, especially up close.
* agents like to go straight forward (including forward turns).
* agents like to look straight at goals, especially up close.

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

* Forward reward proportional to goal distance.

>  Learns to turn towards goal then sprint to exploit reward. Overshooting. Actually not a bad behavior as it sets up a lot of poses in the same area, gradually branching out. Which builds a quite concise PoseCell map.

* Proxitmity to goal plus forward reward proportional to goal distance.

> Learns to maintain it's distance from the goal so as to maximise forward rewards. Also builds decent PoseCell maps with lots of loops in the same area.

## `temporal_window`:

* 2: "Get away from that thing on the left"
* 4: "I'm turning around this other way regardless"
* 10: "Built for speed I'm able to detect and avoid dead-ends/confined spaces"

# Future work


## RatSLAM

* Reverse goal order and tweak for use on exploration tasks.
* Discard experiences with *many* links.
* Quality metric for LV. Don't link low quality experiences.
* Reject closures with vastly different magnetic reading.

## DQN

* Further test [Dropout uncertainty](https://github.com/yaringal/DropoutUncertaintyDemos/).
* Implement in [Caffe](https://github.com/muupan/dqn-in-the-caffe) [fork](https://github.com/mhauskn/dqn) or [Theano](https://github.com/spragunr/deep_q_rl) if not [Torch](https://github.com/kuz/DeepMind-Atari-Deep-Q-Learner)
