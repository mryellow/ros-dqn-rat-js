# ROS + DQN + RatSLAM

Training Deep Q-Learning neural network based on ConvNetJS demo to use sonar range sensors and RatSLAM goals.

* [ConvNetJS - demo](http://cs.stanford.edu/people/karpathy/convnetjs/demo/rldemo.html)
* [ROSLibJS](https://github.com/RobotWebTools/roslibjs/)
* [RatSLAM fork](https://github.com/mryellow/ratslam) (extended ROS integration)

# Status

Got busy and distracted, it works well enough for direct goal seeking and that may be enough to train up an agent which makes pretty maps in RatSLAM (if not straying too far before turning back). Have some decent [experiments](https://github.com/mryellow/reinforcejs/tree/demo-multiagent) with [ReinforceJS](http://cs.stanford.edu/people/karpathy/reinforcejs/). Finding goals on the other side of walls and traps will require a different implementation, namely Actor-critic and/or Actor-mimic style architectures to get around these opstacles (when a goal can be seen on the other side of a trap). 

# Setup

```
npm install
bower install
```

# TODO

* [ ] Teleop.
* [ ] Integrate IMU/tilt/odom feedback.
* [ ] Catkin-ise.
* [ ] Define custom ROS messages.
* [ ] LTM/STM with long-term sets of "important" experiences.
* [ ] Save/load DQN experience sets.

# Usage

```
roslaunch kulbu_base sim.launch world:=rat1
roslaunch kulbu_slam rat.launch use_rat_odom:=false topic_odom:=/kulbu/odometry/filtered
rosrun turtlebot_teleop turtlebot_teleop_key /turtlebot_teleop/cmd_vel:=/kulbu/diff_drive_controller/cmd_vel

roslaunch rosbridge_server rosbridge_websocket.launch # ROSLibJS
node src/main.js
node src/main.js --noise # Generate noise on extra sensors.
node src/ratsim.js # Simulate RatSLAM goals for training.
rqt_plot /dqn/reward:epsilon
rqt_plot /dqn/avg_reward:avg_loss

rostopic pub -1 /dqn/status std_msgs/String -- '"{\"learning\": true, \"moving\": true, \"sensors\": false}"' # TODO: Custom message format.
rostopic pub -1 /dqn/save std_msgs/String -- 'file'   # Save DQN as JSON.
rostopic pub -1 /dqn/load std_msgs/String -- 'file'   # Load DQN from JSON.
rostopic pub -1 /dqn/set_age std_msgs/String -- '"100000"' # FIXME: Datatype.
```

# Future work

## RatSLAM

* [x] Reverse goal order and tweak for use on exploration tasks.
* [ ] Discard experiences with *many* links.
* [ ] Quality metric for LV. Don't link low quality experiences.
* [ ] Reject closures with vastly different magnetic reading?
* [ ] Implement multi Experience Maps [RatSLAM on Humanoids](https://www2.informatik.uni-hamburg.de/wtm/ps/M%C3%BCller_ICANN2014_CR.pdf)

## DQN

* Further test [Dropout uncertainty](https://github.com/yaringal/DropoutUncertaintyDemos/).
* Implement in [Caffe](https://github.com/muupan/dqn-in-the-caffe) [fork](https://github.com/mhauskn/dqn) or [Theano](https://github.com/spragunr/deep_q_rl) if not [Torch](https://github.com/kuz/DeepMind-Atari-Deep-Q-Learner)
