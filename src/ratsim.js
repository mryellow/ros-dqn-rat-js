#!/usr/bin/env node

var ROSLIB = require('roslib');

var config = require('./config.js');
var Ros    = require('./ros.js');

// TODO: Throw errors on invalid config.

var ros = new Ros();

/**
 * Store current goal position.
 * @property {object} saved_goal
 */
var saved_goal = {};

/**
 * Calculate direction and distance to goal.
 * @function updateGoal
 * @param {float} robot_x
 * @param {float} robot_y
 * @param {float} robot_r
 * @param {float} goal_x
 * @param {float} goal_y
 */
var updateGoal = function (robot_x, robot_y, robot_r, goal_x, goal_y) {
  if (typeof(robot_x) === 'undefined' || typeof(robot_y) === 'undefined' ||
    typeof(goal_x) === 'undefined' || typeof(goal_y) === 'undefined') return;
  //console.log('updateGoal', robot_x.toFixed(4), robot_y.toFixed(4), robot_r.toFixed(4), goal_x.toFixed(4), goal_y.toFixed(4));

  //`tan(rad) = Opposite / Adjacent = (y2-y1)/(x2-x1)`
  var rad = Math.atan2(goal_y - robot_y, goal_x - robot_x);

  //`Hypotenuse = (y2-y1)/sin(rad)`
  var dis = Math.abs((goal_y - robot_y)/Math.sin(rad));

  // Minus robot pose from goal direction.
  rad -= robot_r;
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
    var new_x = robot_x + Math.random() - (config.ratsim_opts.goal_reached*3);
    var new_y = robot_y + Math.random() - (config.ratsim_opts.goal_reached*3);
    moveGoal(new_x, new_y);
  }
};

/**
 * Move goal to new location.
 * @function moveGoal
 * @param {integer} x
 * @param {integer} y
 */
var moveGoal = function(x, y) {
  if (typeof(x) === 'undefined' || typeof(y) === 'undefined') return;
  //console.log('moveGoal', x.toFixed(3), y.toFixed(3));
  saved_goal = {
    x: x,
    y: y
  };

  /**
   * gazebo_msgs/ModelState
   * {string} model_name
   * {geometry_msgs/Pose} pose
   * {geometry_msgs/Twist} twist
   * {string} reference_frame
   */
  var msg = new ROSLIB.Message({
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
  });

  ros.pubTopic(
    '/gazebo/set_model_state',
    'gazebo_msgs/ModelState',
    msg
  );
};

/**
 * Publish simulated RatSLAM SubGoal message.
 * @function pubGoal
 * @param {float} rad
 * @param {float} dis
 */
var pubGoal = function(rad, dis) {
  if (typeof(dis) === 'undefined' || typeof(rad) === 'undefined') return;
  //console.log('pubGoal', rad.toFixed(3), dis.toFixed(3));

  var msg = new ROSLIB.Message({
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
    msg
  );
};

var cnt = 0;
/**
 * Gazebo model states topic.
 * @callback getState
 * @param {object} message
 */
var getState = function(message) {
  var idx = message.name.indexOf(config.ratsim_opts.robot_model);

  // Every `move_every` ticks, move goal model.
  if (cnt > config.ratsim_opts.goal_timeout * 50) cnt = 0;
  // Do one straight away.
  if (cnt === 0) {
    var position = {};
    position = {
      x: Math.random() * (config.ratsim_opts.bounds.x.max - config.ratsim_opts.bounds.x.min) + config.ratsim_opts.bounds.x.min,
      y: Math.random() * (config.ratsim_opts.bounds.y.max - config.ratsim_opts.bounds.y.min) + config.ratsim_opts.bounds.y.min
    };
    console.log('moveGoal', position.x.toFixed(3), position.y.toFixed(3));
    moveGoal(position.x, position.y);
  }

  // Periodically update the goal, sometimes it misses. Only for human, robot doesn't need it.
  if (cnt % 200 === 0) moveGoal(saved_goal.x, saved_goal.y);

  // Extract angles from quaternion.
  var q = message.pose[idx].orientation;
  //var roll? = Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
  //var pitch? = Math.asin(-2.0*(q.x*q.z - q.w*q.y));
  var yaw = Math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
  //console.log(yaw.toFixed(3), pitch.toFixed(3), roll.toFixed(3));

  // Get publish goal position relative to current.
  updateGoal(
    message.pose[idx].position.x,
    message.pose[idx].position.y,
    yaw,
    saved_goal.x,
    saved_goal.y
  );

  cnt++;
};

/**
 * Main loop.
 * Slaved to gazebo topic.
 * Throttled to 50Hz matching RatSLAM.
 */
ros.subTopic(
  '/gazebo/model_states',
  'gazebo_msgs/ModelStates',
  getState,
  50
);
