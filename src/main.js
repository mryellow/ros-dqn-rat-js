#!/usr/bin/env node

var ROSLIB = require('roslib');
var fs = require('fs');

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
 * Bespoke function to initialise eye positions for specific robot.
 * @function
 * @return {array}
 */
var initEyes = function() {
  var res = [];
  var small_fov = (15*Math.PI/180);
  // Two large rear eyes from remainder of circle.
  var large_fov = ((2*Math.PI) - (6*small_fov))/2;

  var rad = ((0-3)*small_fov) - (large_fov/2);
  res.push({
    name:  config.eye_names[0],
    angle: rad,
    fov:   large_fov
  });
  for (var k=0; k<7; k++) {
    rad = ((k-3)*small_fov);
    res.push({
      name:  config.eye_names[k+1],
      angle: rad,
      fov:   small_fov
    });
  }
  rad = rad + (large_fov/2);
  res.push({
    name:  config.eye_names[8],
    angle: rad,
    fov:   large_fov
  });

  return res;
};

//console.log('Config:', config);

/**
 * Initialise Agent
 */
var agt = new Agent(
  ros,
  initEyes(),
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
 * Inform agent of range sensors from ROS
 * @callback getRange
 * @param {object} message
 */
var getRange = function(message) {
  var frameId = message.header.frame_id;
  var topicName = frameId.replace('_link', '');
  //console.log('getRange', message.range, config.eye_names.indexOf(topicName));
  agt.eyes[config.eye_names.indexOf(topicName)].sensed_proximity = message.range;
  agt.eyes[config.eye_names.indexOf(topicName)].sensed_type = 0;
  agt.eyes[config.eye_names.indexOf(topicName)].updated = true;
};

/**
 * Inform agent of sub goals from RatSLAM
 * @callback getGoal
 * @param {object} message
 */
var getGoal = function(message) {
  // Find matching eye.
  var num_eyes = agt.eyes.length;
  for (var i=0; i<num_eyes; i++) {
    var e = agt.eyes[i];
    var min = e.angle - (e.fov/2);
    var max = e.angle + (e.fov/2);

    if (
      message.dis > 0 &&
      // Within FoV of an eye;
      message.rad > min &&
      message.rad < max
      /* &&
      // Closer than wall;
        (
          e.sensed_type !== 0 ||
          (
            e.sensed_type === 0 &&
            e.sensed_proximity < e.max_range &&
            message.dis < e.sensed_proximity
          )
        )
      */
      ) {
      // TODO: Publish goal sight direction for debugging.
      //console.log('ang', i, e.angle, message.dis);
      e.sensed_goal = Math.min(e.goal_range, message.dis); // limit to within `goal_range`.

    // Force sensed type reset, allow `sensed_proximity` to be set by Sonar.
    // for signal moving away.
    // for sitting on the goal `dis == 0`, move around a little.
    } else {
      // FIXME: Goals not invalidated if no message, no message if no goals.
      e.sensed_goal = e.goal_range;
    }
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
};

// Subscribe to each range sensors ROS topic.
for (var i=0; i<config.eye_names.length; i++) {
  // FIXME: Skip for goal only sensors.
  rob.subRange(config.eye_names[i], getRange);
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

var timer_cnt = 0;
var timer_time = 0;
var tick = function() {
  var time_start = new Date().getTime();

  // Foward
  agt.forward();

  // Mark sensors as not updated.
  for (var i=0; i<agt.eyes[i].length; i++) {
    agt.eyes[i].updated = false;
  }

  // Execute the move.
  if (utils.moving) rob.doMove(agt.linX, agt.angZ);

  // Keep track of how many repeats of same command.
  if (actionix === agt.actionix) {
    agt.repeat_cnt++;
  } else {
    agt.repeat_cnt = 0;
  }

  // Give the state a chance to change.
  var timer = setInterval(function() {
    // Wait for an update from all sonar sensors.
    var updated = 0;
    for (var j=0; j<config.eye_names.length; j++) {
      if (config.eye_names[j].indexOf('range') !== -1 && agt.eyes[j].updated) {
        updated++;
      }
    }

    // TODO: Wait for update to sub_goal.
    //console.log('updated', updated);

    if (updated >= 7) {
      clearInterval(timer);

      // agents like to look at goals, especially up close, but not through walls
      if (agt.eyes[config.eye_names.indexOf('range_0')].sensed_goal < agt.eyes[config.eye_names.indexOf('range_0')].goal_range) {
        var mid_eye = agt.eyes[config.eye_names.indexOf('range_0')];
        // Inversely proportional to the square of the distance.
        var goal_factor = 1/Math.pow(Math.min(0.01, mid_eye.sensed_goal)*100, 2);
        // Reduced by proximity to walls.
        var wall_factor = mid_eye.sensed_proximity/mid_eye.max_range;

        agt.digestion_signal += 0.5 * goal_factor * wall_factor;
        console.log('digest goal', mid_eye.sensed_proximity, mid_eye.sensed_goal, agt.digestion_signal);
      }

      // Backward
      agt.backward();
      actionix = agt.actionix;

      if (timer_time > 5000) {
        console.log('fps:' + (timer_cnt/(timer_time/1000)).toFixed(1));
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
