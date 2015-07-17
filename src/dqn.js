#!/usr/bin/env node

// Draft, encapsulation to come.

// TODO: Not here, just a note, RatSLAM shouldn't create ViewTemplates when hard up against a wall.

/**
 * DQN.
 */


/*
Deg:
45 - 30 - 15 - 0 + 15 + 30 + 45 + 60 + 75 + 90 + 105 + 120 + 135 + 150 +
Rad:
0.261799388
0.523598776
0.785398163
*/

// Connecting to ROS
var ROSLIB = require('roslib');
var fs = require('fs');

//http://mlg.eng.cam.ac.uk/yarin/blog_3d801aa532c1ce.html
//var convnetjs   = require('./vendor/uncertain/convnet.js');
//var deepqlearn  = require('./vendor/uncertain/deepqlearn.js');

GLOBAL.convnetjs   = require('./vendor/convnet.js'); // Not included in repo.
GLOBAL.deepqlearn  = require('../bower_components/convnetjs/build/deepqlearn.js');
GLOBAL.cnnutil     = require('../bower_components/convnetjs/build/util.js');

// TODO: Encapsulate config.
// Two extra rear sensors for goals.
var eye_names = [
  'goal_rear_l',
  'range_3l',
  'range_2l',
  'range_1l',
  'range_0',
  'range_1r',
  'range_2r',
  'range_3r',
  'goal_rear_r'
];

// Create goal how many experiences from end of map.
var goal_distance = 20;


var ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});



/*
// A 2D vector utility
var Vec = function(x, y) {
  this.x = x;
  this.y = y;
};

Vec.prototype = {
  // utilities
  dist_from: function(v) { return Math.sqrt(Math.pow(this.x-v.x,2) + Math.pow(this.y-v.y,2)); },
  length: function() { return Math.sqrt(Math.pow(this.x,2) + Math.pow(this.y,2)); },

  // new vector returning operations
  add: function(v) { return new Vec(this.x + v.x, this.y + v.y); },
  sub: function(v) { return new Vec(this.x - v.x, this.y - v.y); },
  rotate: function(a) {  // CLOCKWISE
    return new Vec(this.x * Math.cos(a) + this.y * Math.sin(a),
                   -this.x * Math.sin(a) + this.y * Math.cos(a));
  },

  // in place operations
  scale: function(s) {
      this.x *= s; this.y *= s;
  },
  normalize: function() { var d = this.length(); this.scale(1.0/d); }
};
*/

// Eye sensor has a maximum range and senses walls
var Eye = function(angle, fov) {
  console.log('Creating eye', angle, fov);
  this.angle            = angle;
  this.fov              = (fov)?fov:(15*Math.PI/180); // Default 15deg.
  this.max_range        = 4;
  this.sensed_proximity = 4;
  this.sensed_type      = -1; // what does the eye see?

  // Extra sensor dealing exclusively with goals.
  // TODO: Refactor to binary, forward quater, "nose".
  this.goal_range  = 10;
  this.sensed_goal = 10;

  // Watch for updates, syncing framerate to sensors.
  this.updated = false;
};

// RatSLAM Goal log for rewarding distance.
var Goal = function(id, dis, rad) {
  //console.log('Creating goal', id, dis, rad);
  this.id  = id;
  this.dis = dis;
  this.rad = rad;
};

var RosTopic = function(name, type) {
  if (!name || !type) return;
  return new ROSLIB.Topic({
    ros:          ros,
    name:         name,
    messageType:  type
  });
};

var RosMsgFloat = function(val) {
  return new ROSLIB.Message({
    data: parseFloat(val)
  });
};

// A single agent
var Agent = function() {
  // positional information
  /*
  this.p = new Vec(50, 50);
  this.op = this.p; // old position
  this.angle = 0; // direction facing
  */

  this.actions = [];
  // Faster turning when advancing.
  this.actions.push([1.0,0.0]);
  this.actions.push([1.0,-3.0]);
  this.actions.push([1.0,3.0]);
  this.actions.push([0.0,-4.0]);
  this.actions.push([0.0,4.0]);

  // properties
  // this.rad = 10; // ???
  this.eyes = [];

  // Initialise eye positions.
  var small_fov = (15*Math.PI/180);
  // Two large rear eyes from remainder of circle.
  var large_fov = ((2*Math.PI) - (6*small_fov))/2;

  var rad = ((0-3)*small_fov) - (large_fov/2);
  this.eyes.push(new Eye(rad, large_fov));
  for (var k = 0; k < 7; k++) {
    rad = ((k-3)*small_fov);
    this.eyes.push(new Eye(rad));
  }
  rad = rad + (large_fov/2);
  this.eyes.push(new Eye(rad, large_fov));

  // RatSLAM goals for rewarding distance.
  this.goals = [];

  this.pub_stats = {
    // Publish DQN statistics to `stats`.
    //learning: RosTopic('/dqn/learning', 'std_msgs/Float32'),
    reward:     RosTopic('/dqn/reward', 'std_msgs/Float32'),
    //action:   RosTopic('/dqn/action', 'std_msgs/Float32'),
    epsilon:    RosTopic('/dqn/epsilon', 'std_msgs/Float32'),
    //avg_loss: RosTopic('/dqn/avg_loss', 'std_msgs/Float32'),
    avg_reward: RosTopic('/dqn/avg_reward', 'std_msgs/Float32')
    //age:      RosTopic('/dqn/age', 'std_msgs/Float32')
  };

// 5 eyes, 3 color = 15.
// 7 eyes, 3 color = 21.
// 27 = 9 eyes, each sees 3 numbers (wall, green, red thing proximity)
// 36 = 9 eyes, each sees 3 numbers (wall, green, red thing proximity), and goals.
  var num_inputs = 36;
  var num_actions = 5; // 5 possible angles agent can turn
  var temporal_window = 4; // amount of temporal memory. 0 = agent lives in-the-moment :)
  var network_size = num_inputs*temporal_window + num_actions*temporal_window + num_inputs;

  // the value function network computes a value of taking any of the possible actions
  // given an input state. Here we specify one explicitly the hard way
  // but user could also equivalently instead use opt.hidden_layer_sizes = [20,20]
  // to just insert simple relu hidden layers.
  var layer_defs = [];
  layer_defs.push({type: 'input', out_sx: 1, out_sy: 1, out_depth: network_size});
  layer_defs.push({type: 'fc', num_neurons: 80, activation: 'relu'});
  //layer_defs.push({type:'dropout', drop_prob:0.2}); // Uncertainty approximation.
  layer_defs.push({type: 'fc', num_neurons: 80, activation: 'relu'});
  layer_defs.push({type: 'regression', num_neurons: num_actions});

  // options for the Temporal Difference learner that trains the above net
  // by backpropping the temporal difference learning rule.
  var tdtrainer_options = {learning_rate: 0.001, momentum: 0.0, batch_size: 64, l2_decay: 0.01};

  var opt = {};
  opt.behavior_policy = 'greedy';
  //opt.behavior_policy = 'thompson';

  opt.temporal_window = temporal_window;
  opt.experience_size = 30000;
  opt.start_learn_threshold = 1000;
  opt.gamma = 0.7;
  opt.learning_steps_total = 200000;
  //opt.learning_steps_total = 150000;
  opt.learning_steps_burnin = 3000;
  opt.epsilon_min = 0.05;
  opt.epsilon_test_time = 0.05;
  opt.layer_defs = layer_defs;
  opt.tdtrainer_options = tdtrainer_options;

  // Prefer turning, to easier get unstuck while training.
  opt.random_action_distribution = [0.1, 0.15, 0.15, 0.3, 0.3];

  var brain = new deepqlearn.Brain(num_inputs, num_actions, opt); // woohoo

  // Overload `random_action` to randomly turn on the spot.
  var repeat_cnt = 0;
  var repeat_act = 0;
  brain.random_action_legacy = brain.random_action;

  brain.random_action = function() {
      // Going badly for a long time, or at start of training.
      // Robot physics make getting unstuck harder than original, give it a hand.
      if (brain.average_reward_window.get_average() < 0.6 || brain.epsilon > 0.3) {
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
      return brain.random_action_legacy();
  };

  this.brain = brain;

  this.reward_bonus = 0.0;
  this.digestion_signal = 0.0;

  // outputs on world
  this.linX = 0.0; // Linear velocity forwards.
  this.angZ = 0.0; // Angular velocity rotation.

  //this.prevactionix = -1; // ??
};

Agent.prototype = {
  forward: function() {
    // in forward pass the agent simply behaves in the environment
    // create input to brain
    // TODO: Signal when loop closure, so net can correlate difference from goal eyes improvement.
    var num_eyes = this.eyes.length;
    var input_array = new Array(num_eyes * 1); // FIXME: ``* 1` Shouldn't it be the size?
    for (var i=0; i<num_eyes; i++) {
      var e = this.eyes[i];
      input_array[i*4] = 1.0; // Wall
      input_array[i*4+1] = 1.0; // Type 1
      input_array[i*4+2] = 1.0; // Type 2
      input_array[i*4+3] = Math.min(e.goal_range, e.sensed_goal)/e.goal_range; // Goal, normalize to [0,1]
      if (e.sensed_type !== -1) {
        // sensed_type is 0 for wall, 1 for food and 2 for poison.
        // lets do a 1-of-k encoding into the input array
        input_array[i*4 + e.sensed_type] = e.sensed_proximity/e.max_range; // normalize to [0,1]
        /*
        console.log('input0: ' + input_array[i*4]);
        console.log('input1: ' + input_array[i*4+1]);
        console.log('input2: ' + input_array[i*4+2]);
        console.log('input3: ' + input_array[i*4+3]);
        */
      }
    }

    // get action from brain
    var actionix = this.brain.forward(input_array);
    var action = this.actions[actionix];
    this.actionix = actionix; //back this up

    // demultiplex into behavior variables
    this.linX = action[0]*1;
    this.angZ = action[1]*1;
  },
  backward: function() {
    // in backward pass agent learns.
    // compute reward
    var proximity_reward = 0.0;
    var num_eyes = this.eyes.length;
    for (var i=0; i<num_eyes; i++) {
      var e = this.eyes[i];
      // agents dont like to see walls, especially up close
      proximity_reward += e.sensed_type === 0 ? e.sensed_proximity/e.max_range : 1.0;

      // Debug log, goal spotting, shown in sync with brain.
      /*
      if (e.sensed_goal < e.goal_range) {
        console.log('a', i, e.sensed_goal, Math.min(e.goal_range, e.sensed_goal)/e.goal_range);
      }
      */
    }
    proximity_reward = proximity_reward/(num_eyes-2); // FIXME: Hack to remove 2 extra eyes.
    //proximity_reward = Math.min(1.0, proximity_reward * 2); // FIXME: Helps keep walls in context?
    proximity_reward = Math.min(1.0, proximity_reward);
    //console.log('Prox:' + proximity_reward);

    // agents like to be near goals
    var goal_factor = 0.0;
    var goal_reward = 0.0;
    if (this.goals[this.goals.length-1] && this.goals[this.goals.length-1].dis > 0) {
      // FIXME: Only if below max, so there is a reading to correlate...
      goal_factor = Math.max(0.0, Math.min(1.0, 1/this.goals[this.goals.length-1].dis));

      // FIXME: Redundant, forward bonus implies this.
      // Although, it is forward only... Then forward towards goal is redundant..
      // and then all of them are redundant if there is a digestion reward... have to be close and facing to digest.
      // However, irregular rewards, distance is clear and gradual in each action.
      // Does rewarding forward with goal instead encourage skirting walls exploiting forward reward close to wall?
      // FIXME: Encourages hitting walls, use digest signal instead, reverse goals in RatSLAM
      //goal_reward = 0.1 * goal_factor * proximity_reward;
      //goal_reward = 0.05 * goal_factor * proximity_reward;

      // TODO: Check goal angle here, reward for forward. Net can divulge this from forward reward, but want pose.
    }

    // agents like to go straight forward, more-so towards goals. // FIXME: "near" goals... side-effect, max towards goal.
    var forward_reward = 0.0;
    // TODO: Reward fast turns only once epsilon drops.
    if ((this.actionix === 0 || this.actionix === 1 || this.actionix === 2) && proximity_reward > 0.85) { // 0.75 a little too close for corners. Rewards stuck wheel.
      // FIXME: Closer to goal = more forward. Instead of eye reward, stop goal reward when goal is behind?
      //forward_reward = (0.05 + (0.05 * goal_factor)) * proximity_reward;
      forward_reward = 0.05 * proximity_reward;
      // Half as much for forward turns.
      // FIXME: Right?
      /*
      if (this.actionix === 1 || this.actionix === 2) {
        forward_reward = forward_reward / 2;
      }
      */
    }

    // agents like to eat good things
    var digestion_reward = this.digestion_signal;
    this.digestion_signal = 0.0;

    // TODO: Extra digestion signal for goal in centre eye and close range?
    // agents like to look at goals when up close, overruling the reward from running through them
    if (agent.eyes[eye_names.indexOf('range_0')].sensed_goal < agent.eyes[eye_names.indexOf('range_0')].goal_range) {
      digestion_reward += 0.05 * (1/agent.eyes[eye_names.indexOf('range_0')].sensed_goal) * proximity_reward;
      console.log('digest goal', agent.eyes[eye_names.indexOf('range_0')].sensed_goal, digestion_reward);
    }

    var reward = proximity_reward + forward_reward + goal_reward + digestion_reward;
    if (this.cnt > 4) {
      console.log(reward.toFixed(5), this.actionix, this.cnt, this.linX.toFixed(1), this.angZ.toFixed(1));
      console.log('rewards', forward_reward, goal_reward);
    }

    var msg;
    /*
    msg = RosMsgFloat((this.brain.learning)?1:0);
    this.pub_stats.learning.publish(msg);
    */
    msg = RosMsgFloat(reward);
    this.pub_stats.reward.publish(msg);
    /*
    msg = RosMsgFloat(this.actionix);
    this.pub_stats.action.publish(msg);
    */

    msg = RosMsgFloat(this.brain.epsilon);
    this.pub_stats.epsilon.publish(msg);
    /*
    msg = RosMsgFloat(this.brain.average_loss_window.get_average());
    this.pub_stats.avg_loss.publish(msg);
    */
    msg = RosMsgFloat(this.brain.average_reward_window.get_average());
    this.pub_stats.avg_reward.publish(msg);
    /*
    msg = RosMsgFloat(this.brain.age);
    this.pub_stats.age.publish(msg);
    */
    // pass to brain for learning
    this.brain.backward(reward);
  }
};

var agent = new Agent();

/**
 * ROS pub/sub.
 */

// Publish twist to `cmd_vel`.
var pub_cmdvel = RosTopic('/kulbu/diff_drive_controller/cmd_vel', 'geometry_msgs/Twist');

// Subscribe to each "eyes" ROS messages.
var getRange = function(message) {
  var frameId = message.header.frame_id;
  var topicName = frameId.replace('_link', '');
  /*
  console.log('Received message on ' +
    frameId + ':' +
    message.range + ':' + eye_names.indexOf(topicName));
  */
  agent.eyes[eye_names.indexOf(topicName)].sensed_proximity = message.range;
  agent.eyes[eye_names.indexOf(topicName)].sensed_type = 0;
  agent.eyes[eye_names.indexOf(topicName)].updated = true;
};

var sub_eyes = [];
for (var i = 0; i < eye_names.length; i++) {
  var topic = RosTopic('/kulbu/' + eye_names[i], 'sensor_msgs/Range');
  topic.subscribe(getRange);
  sub_eyes.push(topic);
}

var getGoal = function(message) {
  // Find matching eye.
  var num_eyes = agent.eyes.length;
  for (var i=0; i<num_eyes; i++) {
    var e = agent.eyes[i];
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
      e.sensed_goal = e.goal_range;
    }
  }

  // Record for rewarding later.
  agent.goals.push(new Goal(message.id, message.dis, message.rad));

  // TODO: If Goal ID has changed, publish an "eat" reward?
  // Does it only change when eaten? Not really, it changes when shortcut too.

  // Truncate log, only need a few to gauge getting closer.
  agent.goals = agent.goals.slice(-2);
};

// Subscribe to sub goals from RatSLAM.
var sub_rat_goal = RosTopic('/ratslam/ExperienceMap/SubGoal', 'ratslam_ros/TopologicalGoal');

var temp_last_rad;
sub_rat_goal.subscribe(getGoal);

// Publish RatSLAM Goal.
var pub_goal = RosTopic('/ratslam/ExperienceMap/SetGoalPose', 'geometry_msgs/PoseStamped');

// Subscribe to RatSLAM experiences.
var sub_rat_map = RosTopic('/ratslam/ExperienceMap/Map', 'ratslam_ros/TopologicalMap');
sub_rat_map.subscribe(function(message) {
  // TODO: Switch to turn on/off goal creation.

  // TODO: Instead of waiting for map, watch TopologicalActions for link closures.
  // "If it feels a long way from familiar ground, head back." Temporal distance to last link/closure?
  // "If a corner has been passed, head back." Corner?
  // "If it has been awhile, head back." Set goal back a few experiences.

  var past_exp = message.node[message.node.length-goal_distance];
  if (past_exp && past_exp.pose && past_exp.pose.position && agent.brain.epsilon < 0.5 && message.node.length < 50000) {
    //var now = new Date().getTime();
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
      pose: {
        position: {
          x: past_exp.pose.position.x,
          y: past_exp.pose.position.y,
          z: 0
        },
        orientation: {
          x: 0,
          y: 0,
          z: 0,
          w: 0
        }
      }
    });
    console.log(goal.pose.position);
    pub_goal.publish(goal);
  }

});

// Publish to ROS `cmd_vel`.
var pub_cmdvel = RosTopic('/kulbu/diff_drive_controller/cmd_vel', 'geometry_msgs/Twist');

/**
 * DQN Utility topics.
 */
// Subscribe to topic for pause/resume of DQN.
// Allowing for restart of RatSLAM, which doesn't like movement during init.
var dqn_paused = false;
var sub_dqn_transport = RosTopic('/dqn/pause', 'std_msgs/Bool');
sub_dqn_transport.subscribe(function(message) {
  // TODO: Re-factor `learning` into separate switch.
  agent.brain.learning = dqn_paused = (message.data);

  // Reset goal sensors.
  // TODO: Encapsulate, duplication.
  var num_eyes = agent.eyes.length;
  for (var i=0; i<num_eyes; i++) {
    var e = agent.eyes[i];
    e.sensed_goal = e.goal_range;
  }
  agent.goals = [];

  console.log('DQN Paused', (message.data));
});

// Set DQN Network age.
// TODO: Use an int type, string for convenience.
var sub_dqn_age = RosTopic('/dqn/set_age', 'std_msgs/String');
sub_dqn_age.subscribe(function(message) {
  agent.brain.age = parseInt(message.data);
  console.log('DQN age set '+ message.data);
});

// Load DQN Network.
var sub_dqn_load = RosTopic('/dqn/load', 'std_msgs/String');
sub_dqn_load.subscribe(function(message) {
  // Sanitise filename.
  var file = message.data.replace(/[^a-z0-9]/i, '_');
  var json = agent.brain.value_net.toJSON();

  fs.readFile('./brains/'+file+'.json', function read(err, data) {
    if (err) {
      return console.log('Error', err);
    }

    agent.brain.value_net.fromJSON(JSON.parse(data));

    // Set age to 60% through.
    // FIXME: Probably best to leave this up to user. This is why it crashes into walls again...
    //agent.brain.age = 0.75 * agent.brain.learning_steps_total;
    agent.brain.age = agent.brain.learning_steps_total;

    // Reset goal sensors.
    // TODO: Encapsulate, duplication.
    var num_eyes = agent.eyes.length;
    for (var i=0; i<num_eyes; i++) {
      var e = agent.eyes[i];
      e.sensed_goal = e.goal_range;
    }
    agent.goals = [];

    console.log('DQN loaded from '+ file);
  });
});

// Save DQN Network.
var sub_dqn_save = RosTopic('/dqn/save', 'std_msgs/String');
sub_dqn_save.subscribe(function(message) {
  // Sanitise filename.
  var file = message.data.replace(/[^a-z0-9]/i, '_');
  var json = agent.brain.value_net.toJSON();

  fs.writeFile('./brains/'+file+'.json', JSON.stringify(json, null, 1), function(err) {
    if (err) {
      return console.log('Error', err);
    }
    console.log('DQN saved to '+ file);
  });
});

/**
 * Main loop Tick.
 */
agent.cnt = 0; // Counter for limiting logs to repeating events.
var actionix = 0;
var tick = function() {
  if (!dqn_paused) {
    // Foward
    agent.forward();

    // Mark sensors as not updated.
    for (var i=0; i<eye_names.length; i++) {
      agent.eyes[i].updated = false;
    }

    // Publish chosen action from agent.
    var twist = new ROSLIB.Message({
      linear: {
        x: agent.linX,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: agent.angZ
      }
    });
    pub_cmdvel.publish(twist);

    // Keep track of how many repeats of same command.
    if (actionix === agent.actionix) {
      agent.cnt++;
    } else {
      agent.cnt = 0;
    }
  }

  // Give the state a chance to change.
  var timer = setInterval(function() {
    // Wait for an update from all sonar sensors.
    var updated = 0;
    for (var j=0; j<eye_names.length; j++) {
      //console.log(j, eye_names[j], agent.eyes[j].updated);
      if (eye_names[j].indexOf('range') !== -1 && agent.eyes[j].updated) {
        updated++;
      }
    }

    // TODO: Wait for update to sub_goal.
    //console.log('updated', updated);

    if (updated >= 7) {
      clearInterval(timer);
      // Backward
      agent.backward();
      actionix = agent.actionix;
      tick();
    }
  }, (1000/30)); // Hz, Faster than sonars but checking for their updates.
  // FIXME: Does this timing mean `cmd_vel` hasn't had time to do much?
};
tick();
