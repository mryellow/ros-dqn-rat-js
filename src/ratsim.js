#!/usr/bin/env node

var ROSLIB = require('roslib');

var config = require('./config.js');
var Ros    = require('./ros.js');

// TODO: Throw errors on invalid config.

var ros = new Ros();

service_get_model = new ROSLIB.Service({
  ros:  ros._ros,
  name: 'gazebo/get_model_state',
  serviceType: 'gazebo_msgs/GetModelState'
});

service_set_model = new ROSLIB.Service({
  ros:  ros._ros,
  name: 'gazebo/set_model_state',
  serviceType: 'gazebo_msgs/SetModelState'
});

var updateGoal = function (goal_x, goal_y) {
  if (typeof(goal_x) === 'undefined' || typeof(goal_y) === 'undefined') return;

  // Request robot pose.
  var req = new ROSLIB.ServiceRequest({ model_name: config.ratsim_opts.robot_model });
  service_get_model.callService(req, function (result) {
      if (result.success) {
        //`tan(rad) = (y1-y2)/(x1-x2)`
        var rad = Math.atan2(goal_y - result.pose.position.y, goal_x - result.pose.position.x);

        //`Hypotenuse = (y1-y2)/sin(rad)`
        var dis = Math.abs((goal_y - result.pose.position.y)/Math.sin(rad));

        // Minus robot pose from goal direction.
        rad -= (result.pose.orientation.z * Math.PI);
        if (rad > Math.PI) {
          rad -= 2 * Math.PI;
        } else if (rad < -Math.PI) {
          rad += 2 * Math.PI;
        }

        // Publish SubGoal relative to this position.
        pubGoal(rad, dis);

        // If the goal distance is below threshold, publish a new one nearby.
        if (dis < config.ratsim_opts.goal_reached) {
          console.log('Goal reached');
          var new_x = result.pose.position.x + Math.random() - 0.5;
          var new_y = result.pose.position.y + Math.random() - 0.5;
          moveGoal(new_x, new_y);
        }
      } else {
        console.log('Error', result.status_message);
      }
  });

};

var moveGoal = function(x, y) {
  if (typeof(x) === 'undefined' || typeof(y) === 'undefined') return;
  console.log('moveGoal', x.toFixed(3), y.toFixed(3));

  var req = new ROSLIB.ServiceRequest({
    model_state: {
      model_name: config.ratsim_opts.goal_model,
      pose: {
        position: { x: x, y: y ,z: config.ratsim_opts.goal_height },
        orientation: { x: 0, y: 0, z: 0, w: 0 }
      },
      twist: {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      },
      reference_frame: 'world'
    }
  });

  service_set_model.callService(req, function(result) {
    //console.log('setGoalRes', result);
    if (!result.success) console.log('Error', result.status_message);
  });
};

var pubGoal = function(rad, dis) {
  if (typeof(dis) === 'undefined' || typeof(rad) === 'undefined') return;
  //console.log('pubGoal', rad.toFixed(3), dis.toFixed(3));

  var goal = new ROSLIB.Message({
    /*
    header: {
      seq: 0,
      stamp: {
        secs: now/1000,
        nsecs: now
      },
      frame_id: "1"
    },
    */
    id:  0,
    dis: dis,
    rad: rad
  });

  ros.pubTopic(
    '/ratslam/ExperienceMap/SubGoal',
    'ratslam_ros/TopologicalGoal',
    goal
  );
};


var cnt = 0;
var position = {};
var tick = function() {

  // Every `move_every` ticks, move goal model.
  if (cnt > config.ratsim_opts.goal_timeout * config.ratsim_opts.main_loop) cnt = 0;
  // Do one straight away.
  if (cnt === 0) {
    position = {
      x: Math.random() * (config.ratsim_opts.bounds.x.max - config.ratsim_opts.bounds.x.min) + config.ratsim_opts.bounds.x.min,
      y: Math.random() * (config.ratsim_opts.bounds.y.max - config.ratsim_opts.bounds.y.min) + config.ratsim_opts.bounds.y.min
    };
    moveGoal(position.x, position.y);
  }

  // Get robot's current position and publish goal.
  updateGoal(position.x, position.y);

  cnt++;
};

var timer = setInterval(tick, 1000/config.ratsim_opts.main_loop); // Hz
tick();
