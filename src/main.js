#!/usr/bin/env node

var ROSLIB = require('roslib');
var fs = require('fs');
var jStat = require('jStat').jStat;

//http://mlg.eng.cam.ac.uk/yarin/blog_3d801aa532c1ce.html
//GLOBAL.convnetjs   = require('./vendor/uncertain/convnet.js');
//GLOBAL.deepqlearn  = require('./vendor/uncertain/deepqlearn.js');

// Global to bring into scope for ConvNetJS.
//GLOBAL.convnetjs   = require('./vendor/convnet.js'); // Not included in repo.
GLOBAL.convnetjs   = require('../bower_components/convnetjs/build/convnet.js'); // Compiled with `cd bower_components/convnetjs/compile && ant -lib yuicompressor-2.4.8.jar -f build.xml`
GLOBAL.deepqlearn  = require('../bower_components/convnetjs/build/deepqlearn.js');
GLOBAL.cnnutil     = require('../bower_components/convnetjs/build/util.js');

var config = require('./config.js');
var Ros    = require('./ros.js');
var Robot  = require('./robot.js');
var Rat    = require('./rat.js');
var Agent  = require('./agent.js');
var Utils  = require('./utils.js');

var ros = new Ros();
var rob = new Robot(ros, '/kulbu', 'diff_drive_controller/cmd_vel');
var rat = new Rat(ros);

/**
 * Initialise sensor positions.
 * @function
 * @return {object}
 */
var initSensors = function() {
  var res = {};
  for (var j in config.sensors) {
    if (config.sensors.hasOwnProperty(j)) {
      var fov   = config.sensors[j].fov;
      var types = config.sensors[j].types;
      var range = config.sensors[j].range;
      for (var i=0; i<config.sensors[j].names.length; i++) {
        var rad = (i-((config.sensors[j].names.length-1)/2))*fov;
        if (typeof(res[j]) === 'undefined') res[j] = [];
        res[j].push({
          name:      config.sensors[j].names[i],
          angle:     rad,
          fov:       fov,
          max_range: range,
          max_type:  config.sensors[j].types
        });
      }
    }
  }

  return res;
};

/**
 * Initialise Agent
 */
var agt = new Agent(
  ros,
  initSensors(),
  config.actions,
  config.brain_opts
);

/**
 * Overload `random_action` to randomly turn on the spot.
 * Simulated robot with wheels out front not as slippery as original demo.
 * @function
 * @return {integer}
 */
var repeat_cnt = 0;
var repeat_act = 0;
agt.brain.random_action_legacy = agt.brain.random_action;
agt.brain.random_action = function() {
    // Going badly for a long time, or at start of training.
    // Robot physics make getting unstuck harder than original, give it a hand.
    if (agt.brain.average_reward_window.get_average() < 0.6 || agt.brain.epsilon > 0.3) {
        var seed = convnetjs.randi(0, 150);
        if (seed < 1) {
            repeat_act = convnetjs.randi(3, 5);
            repeat_cnt = 30;
            console.log('Repeat: ' + repeat_act);
        }
        if (repeat_cnt > 0) {
            repeat_cnt--;
            return repeat_act;
        }
    }
    return agt.brain.random_action_legacy();
};

/**
 * Lookup a sensor array by name.
 * @function
 * @param {array} arr
 * @param {string} name
 * @return {mixed}
 */
var findByName = function(arr, name) {
  for (var i=0; i<arr.length; i++) {
    if (arr[i].name === name) {
      return arr[i];
    }
  }
  return;
};

/**
 * Lookup a sensor array by view direction.
 * @function
 * @param {array} arr
 * @param {float} rad
 * @return {mixed}
 */
var findByAngle = function(arr, rad) {
  for (var i=0; i<arr.length; i++) {
    if (rad > arr[i].angle - (arr[i].fov/2) && rad < arr[i].angle + (arr[i].fov/2)) {
      return arr[i];
    }
  }
  return;
};

/**
 * Lookup a sensor array by view direction.
 * @function
 * @param {array} arr
 */
var resetSensors = function(arr) {
  for (var i=0; i<arr.length; i++) {
    arr[i].sensed_proximity = arr[i].max_range;
    arr[i].sensed_type = -1;
    arr[i].updated = true;
  }
};

/**
 * Inform agent of range sensors from ROS
 * @callback getRange
 * @param {object} message
 * @todo validate message
 */
var getRange = function(message) {
  var frameId = message.header.frame_id;
  var topicName = frameId.replace('_link', '');
  var s = findByName(agt.sensors.eyes, topicName);
  if (s) {
    s.sensed_proximity = message.range;
    s.sensed_type = 0;
    s.updated = true;
  }
  //console.log('getRange', message.range, e);
};

/**
 * Inform agent of sub goals from RatSLAM
 * @callback getGoal
 * @param {object} message
 */
var getGoal = function(message) {
  // Reset all goal sensors.
  resetSensors(agt.sensors.nostrils);

  // Update the matching sensor.
  var s = findByAngle(agt.sensors.nostrils, message.rad);
  if (s) {
    s.sensed_proximity = message.dis;
    s.sensed_type = 0;
    s.updated = true;
  }
  // Record for rewarding later.
  agt.addGoal(message.id, message.dis, message.rad);
};

/**
 * Create goals each time RatSLAM map updates
 * @callback getMap
 * @param {object} message
 */
var getMap = function(message) {
  var past_exp = message.node[message.node.length - config.goal_distance];
  if (past_exp && past_exp.pose && past_exp.pose.position && agt.brain.epsilon < 0.5 && message.node.length < 50000) {
    // FIXME: Use `id` instead. No need to map distances and lookup nearest, we already have it.
    rat.createGoal(past_exp.pose.position.x, past_exp.pose.position.y);
  }

  // Reset now as we won't get a SubGoal signal when goals are completed.
  // FIXME: Not reset when goal is lacking and map is yet to arrive.
  resetSensors(agt.sensors.nostrils);
};

// Subscribe to each range sensors ROS topic.
for (var i=0; i<config.sensors.eyes.names.length; i++) {
  rob.subRange(config.sensors.eyes.names[i], getRange);
}

// Subscribe to RatSLAM `SubGoal`.
rat.subGoal(getGoal);

// Subscribe to RatSLAM `Map`.
rat.subMap(getMap);

var utils = new Utils(ros, '/dqn', agt);

/**
 * Main loop.
 */
agt.repeat_cnt = 0; // Counter for limiting logs to repeating events.
var actionix = 0;

//var wait_cnt = 0;
var timer_cnt = 0;
var timer_time = 0;
var tick = function() {
  var time_start = new Date().getTime();
  var i,j;

  // Foward
  agt.forward();

  // Mark eyes/sensors as not updated.
  var num_sens = 0;
  for (j in agt.sensors) {
    if (agt.sensors.hasOwnProperty(j)) {
      for (i=0; i<agt.sensors[j].length; i++) {
        agt.sensors[j][i].updated = false;
        num_sens++;
      }
    }
  }

  // Execute the move.
  if (utils.moving) rob.doMove(agt.linX, agt.angZ);

  // Keep track of how many repeats of same command.
  // FIXME: Keeps counting when paused.
  if (actionix === agt.actionix) {
    agt.repeat_cnt++;
  } else {
    agt.repeat_cnt = 0;
  }

  // Give the state a chance to change.
  var timer = setInterval(function() {
    // Wait for an update from all eyes/sensors.
    var i;
    var updated = 0;
    for (j in agt.sensors) {
      if (agt.sensors.hasOwnProperty(j)) {
        for (i=0; i<agt.sensors[j].length; i++) {
          if (agt.sensors[j][i].updated) updated++;
        }
      }
    }
    // TODO: Wait for sensors also.
    //console.log('updated', updated, num_sens);
    //wait_cnt++;

    if (updated >= num_sens) {
      //wait_cnt = 0;
      clearInterval(timer);

      // agents like to look at goals, especially up close, but not through walls
      /*
      var eye = agt.eyes[findEye('range_0')];
      var ran = agt.sensors[findByName(agt.sensors.nostrils, 'goal_range')];
      var dir = agt.sensors[findSensor('goal_direction')];
      if (dir.active && ran.sensed_value < ran.max_value) {
        // Inversely proportional to the square of the distance.
        var ran_factor = 1/Math.pow(ran.sensed_value, 2);

        // FIXME: Use all eyes or expose agent proximity reward?
        var wall_factor = eye.sensed_proximity/eye.max_range;

        // Proportional to the closeness to centre of view.
        var cen_factor = jStat.normal.pdf(dir.sensed_value, 180, 45)*100;

        var dir_reward = 5 * ran_factor * wall_factor * cen_factor;
        /
        console.log(
          'dir_reward',
          ' =:'+dir_reward.toFixed(5),
          ' r:'+ran_factor.toFixed(3),
          ' w:'+wall_factor.toFixed(3),
          ' c:'+cen_factor.toFixed(3),
          parseInt(dir.sensed_value)
        );
        /
        agt.digestion_signal += dir_reward;
      }
      */

      // Backward
      agt.backward();
      actionix = agt.actionix;

      if (timer_time > 5000) {
        console.log(
          'fps:' + (timer_cnt/(timer_time/1000)).toFixed(1),
          'e:' + agt.brain.epsilon.toFixed(2),
          'l:' + agt.brain.average_loss_window.get_average().toFixed(2),
          'r:' + agt.brain.average_reward_window.get_average().toFixed(2)
        );
        timer_time = 0;
        timer_cnt = 0;
      }
      var time_end = new Date().getTime();
      timer_time += time_end - time_start;
      timer_cnt++;

      tick();
    }
  }, (1000/config.main_loop)); // Hz, Faster than sonars but checking for their updates.
};
tick();
