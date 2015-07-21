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
 * Initialise eye positions.
 * @function
 * @param {integer} fov
 * @return {array}
 */
var initEyes = function(fov) {
  var res = [];
  if (!fov) fov = (15*Math.PI/180); // Default to 15deg.
  for (var i=0; i<config.eyes.length; i++) {
    var rad = ((i-3)*fov);
    config.eyes[i].angle = rad;
    config.eyes[i].fov = fov;
  }

  return config.eyes;
};

/**
 * Initialise Agent
 */
var agt = new Agent(
  ros,
  initEyes(),
  config.sensors,
  config.actions,
  config.agent_opts,
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
 * Lookup eye index in config by name.
 * @function
 * @param {string} name
 * @return {integer}
 */
var findEye = function(name) {
  return findByName(config.eyes, name);
};

/**
 * Lookup sensor index in config by name.
 * @function
 * @param {string} name
 * @return {integer}
 */
var findSensor = function(name) {
  return findByName(config.sensors, name);
};

/**
 * Lookup index in an array by name.
 * @function
 * @param {string} name
 * @return {integer}
 */
var findByName = function(arr, name) {
  for (var i=0; i<arr.length; i++) {
    if (arr[i].name === name) {
      return i;
    }
  }
  return -1;
};

/**
 * Set goal sensors to defaults.
 * @function resetGoalSensors
 */
var resetGoalSensors = function() {
  var s_ran = agt.sensors[findSensor('goal_range')];
  var s_dir = agt.sensors[findSensor('goal_direction')];
  s_ran.sensed_value = s_ran.max_value;
  s_dir.sensed_value = s_dir.max_value/2; // Default to middle.
  // Will be re-enabled by signal on `goal_range`.
  s_dir.active = false;
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
  var e = agt.eyes[findEye(topicName)];
  e.sensed_proximity = message.range;
  e.sensed_type = 0;
  e.updated = true;
  //console.log('getRange', message.range, e);
};

/**
 * Inform agent of sub goals from RatSLAM
 * @callback getGoal
 * @param {object} message
 */
var getGoal = function(message) {
  var s_ran = agt.sensors[findSensor('goal_range')];
  if (s_ran.active) {
    var s_dir = agt.sensors[findSensor('goal_direction')];
    if (message.dis > 0) {
      s_ran.sensed_value = message.dis;
      s_dir.sensed_value = (message.rad+Math.PI)*(180/Math.PI); // 0-360 degrees.
    } else {
      s_ran.sensed_value = s_ran.max_value;
      s_dir.sensed_value = s_dir.max_value/2; // Default to middle.
    }
    s_ran.updated = true;
    s_dir.updated = true;

    // Signal to direction sensor when out of range.
    s_dir.active = (s_ran.sensed_value < s_ran.max_value);
    //console.log('getGoal', s_ran.sensed_value, s_dir.sensed_value);

    // Record for rewarding later.
    agt.addGoal(message.id, message.dis, message.rad);
  }
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
  resetGoalSensors();
};

// Subscribe to each range sensors ROS topic.
for (var i=0; i<config.eyes.length; i++) {
  rob.subRange(config.eyes[i].name, getRange);
}

// Periodically activate goal sensor with noise.
// Hope to make transition to training goals smoother, wall crashes confuse RatSLAM.
// TODO: Switch in `status` topic.
if (process.argv[2] === '--noise') {
  console.log('Generate noise on goal sensors.');
  var inner_timer;
  var timerNoise = function() {
    if (typeof(inner_timer) !== "undefined") {
      clearInterval(inner_timer);
      clearInterval(noise_timer);
      noise_timer = setInterval(timerNoise, convnetjs.randi(1, 10)*1000);
    }

    var s_ran = agt.sensors[findSensor('goal_range')];
    var s_dir = agt.sensors[findSensor('goal_direction')];
    s_ran.sensed_value = s_ran.max_value;
    s_dir.sensed_value = s_dir.max_value;
    s_ran.active = !s_ran.active;
    s_dir.active = s_ran.active;
    console.log('Noise:', s_ran.active);
    if (s_ran.active) {
      inner_timer = setInterval(function() {
        s_ran.sensed_value = convnetjs.randf(0.1, s_ran.max_value*2);
        if (s_ran.sensed_value < s_ran.max_value) {
          s_dir.sensed_value = convnetjs.randi(0, s_dir.max_value);
          s_dir.active = true;
        } else {
          s_dir.sensed_value = s_dir.max_value/2;
          s_dir.active = false;
        }
      }, 1000/50); // Update sensors with fake goals at 50Hz
    }
  };

  // Initialise main noise loop.
  noise_timer = setInterval(timerNoise, 10000);
} else {
  // Subscribe to RatSLAM `SubGoal`.
  rat.subGoal(getGoal);

  // Subscribe to RatSLAM `Map`.
  rat.subMap(getMap);
}

var utils = new Utils(ros, '/dqn', agt);

/**
 * Main loop.
 */
agt.repeat_cnt = 0; // Counter for limiting logs to repeating events.
var actionix = 0;

var timer_cnt = 0;
var timer_time = 0;
var tick = function() {
  var time_start = new Date().getTime();
  var i;

  // Foward
  agt.forward();

  // Mark eyes/sensors as not updated.
  for (i=0; i<agt.eyes.length; i++) {
    agt.eyes[i].updated = false;
  }
  for (i=0; i<agt.sensors.length; i++) {
    agt.sensors[i].updated = false;
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
    for (i=0; i<agt.eyes.length; i++) {
      if (agt.eyes[i].updated) updated++;
    }
    // TODO: Wait for sensors also.
    //console.log('updated', updated);

    if (updated >= agt.eyes.length) {
      clearInterval(timer);

      // agents like to look at goals, especially up close, but not through walls
      var eye = agt.eyes[findEye('range_0')];
      var ran = agt.sensors[findSensor('goal_range')];
      var dir = agt.sensors[findSensor('goal_direction')];
      if (dir.active && ran.sensed_value < ran.max_value) {
        // Inversely proportional to the square of the distance.
        var ran_factor = 1/Math.pow(ran.sensed_value, 2);

        // FIXME: Use all eyes or expose agent proximity reward?
        var wall_factor = eye.sensed_proximity/eye.max_range;

        // Proportional to the closeness to centre of view.
        var cen_factor = jStat.normal.pdf(dir.sensed_value, 180, 45)*100;

        var dir_reward = 0.1 * ran_factor * wall_factor * cen_factor;
        /*
        console.log(
          'dir_reward',
          ' =:'+dir_reward.toFixed(5),
          ' r:'+ran_factor.toFixed(3),
          ' w:'+wall_factor.toFixed(3),
          ' c:'+cen_factor.toFixed(3),
          parseInt(dir.sensed_value)
        );
        */
        agt.digestion_signal += dir_reward;
      }

      // Backward
      agt.backward();
      actionix = agt.actionix;

      if (timer_time > 5000) {
        console.log(
          'fps:' + (timer_cnt/(timer_time/1000)).toFixed(1),
          'e:' + agt.brain.epsilon.toFixed(2)
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
