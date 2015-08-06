/**
 * Sensor sensor has a maximum range and senses walls
 * @class
 * @constructor {object} input
 */
var Sensor = function(input) {
  console.log('Creating sensor', input.name);
  this.name             = (input && input.name)?      input.name:'';
  this.angle            = (input && input.angle)?     input.angle:0;
  this.fov              = (input && input.fov)?       input.fov:(15*Math.PI/180); // Default 15deg.
  this.max_range        = (input && input.max_range)? input.max_range:4;
  this.max_type         = (input && input.max_type)?  input.max_type:1;
  this.sensed_proximity = this.max_range;
  this.sensed_type      = -1; // what does the eye see?

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

/**
 * A single agent
 * @class Agent
 * @param {Ros} ros
 * @param {object} sensors
 * @param {array} actions
 * @param {object} brain_opts
 */
var Agent = function(ros, sensors, actions, brain_opts) {
  if (!ros) throw new Exception('ROS instance must be passed to RatSLAM.');
  this.ros = ros;
  var i,j;

  // TODO: Validate given configs and throw errors.

  this.repeat_cnt = 0;

  // Initialise eyes from config passed in.
  // TODO: Handle any number of sensor types.
  var num_inputs = 0;
  this.sensors = {};
  for (j in sensors) {
    if (sensors.hasOwnProperty(j)) {
      for (i=0; i<sensors[j].length; i++) {
        if (typeof(sensors[j][i].angle) !== 'undefined' && typeof(sensors[j][i].fov) !== 'undefined') {
          if (typeof(this.sensors[j]) === 'undefined') this.sensors[j] = [];
          this.sensors[j].push(new Sensor(sensors[j][i]));
          num_inputs += sensors[j][i].max_type;
        }
      }
    }
  }

  this.actions = (actions)?actions:[
    // Default actions.
    [1.0,0.0],
    [1.0,-3.0],
    [1.0,3.0],
    [0.0,-4.0],
    [0.0,4.0]
  ];

  // Remember RatSLAM goals for rewarding distance.
  // FIXME: We only need the last one right? Was expecting to compare them...
  this.goals = [];

  var num_actions     = this.actions.length;
  var temporal_window = brain_opts.temporal_window; // amount of temporal memory. 0 = agent lives in-the-moment :)
  var network_size = num_inputs*temporal_window + num_actions*temporal_window + num_inputs;

  // the value function network computes a value of taking any of the possible actions
  // given an input state. Here we specify one explicitly the hard way
  // but user could also equivalently instead use opt.hidden_layer_sizes = [20,20]
  // to just insert simple relu hidden layers.
  var layer_defs = [];
  layer_defs.push({type: 'input', out_sx: 1, out_sy: 1, out_depth: network_size});
  layer_defs.push({type: 'fc', num_neurons: 80, activation: 'relu'});
  //layer_defs.push({type: 'dropout', drop_prob: 0.2}); // Uncertainty approximation.
  layer_defs.push({type: 'fc', num_neurons: 80, activation: 'relu'});
  layer_defs.push({type: 'regression', num_neurons: num_actions});

  brain_opts.layer_defs = layer_defs;

  var brain = new deepqlearn.Brain(num_inputs, num_actions, brain_opts); // woohoo

  this.brain = brain;

  this.reward_bonus = 0.0;
  this.digestion_signal = 0.0;

  // outputs on world
  this.linX = 0.0; // Linear velocity forwards.
  this.angZ = 0.0; // Angular velocity rotation.
};

Agent.prototype = {
  /**
   * Add RatSLAM goal to memory for later reward
   * @method addGoal
   * @param {integer} id
   * @param {float} dis
   * @param {float} rad
   */
  addGoal: function(id, dis, rad) {
    // TODO: If Goal ID has changed, publish an "eat" reward?
    // Does it only change when eaten? Not really, it changes when shortcut too.
    this.goals.push(new Goal(id, dis, rad));
    // Truncate log, only need a few to gauge getting closer.
    this.goals = this.goals.slice(-2);
  },
  forward: function() {
    // in forward pass the agent simply behaves in the environment

    // create input to brain
    var i,j;
    var idx = 0;
    var num_inputs = 0;
    for (j in this.sensors) {
      if (this.sensors.hasOwnProperty(j)) {
        num_inputs += this.sensors[j].length;
      }
    }
    var input_array = new Array(num_inputs * 1);

    var idx_last = 0;
    for (j in this.sensors) {
      if (this.sensors.hasOwnProperty(j)) {
        for (i=0; i<this.sensors[j].length; i++) {
          var s = this.sensors[j][i];
          idx = (i * s.max_type)+idx_last;
          for (k=0; k<s.max_type; k++) {
            input_array[idx+k] = 1.0;
          }
          if (s.sensed_type !== -1) {
            input_array[idx+s.sensed_type] = s.sensed_proximity/s.max_range; // normalize to [0,1]
          }
        }
        // Offset the next sensor group by this much.
        idx_last = this.sensors[j].length * this.sensors[j][0].max_type;
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
    var num_eyes = this.sensors.eyes.length;
    for (var i=0; i<num_eyes; i++) {
      var e = this.sensors.eyes[i];
      // agents dont like to see walls, especially up close
      proximity_reward += e.sensed_type === 0 ? e.sensed_proximity/e.max_range : 1.0;
    }
    proximity_reward = proximity_reward/num_eyes;
    proximity_reward = Math.min(1.0, proximity_reward * 2);

    // agents like to be near goals
    var goal_factor = 0.0;
    var goal_reward = 0.0;
    /*
    Deprecated, interesting results, come back to this with more experiments.
    if (this.goals[this.goals.length-1] && this.goals[this.goals.length-1].dis > 0) {
      goal_factor = Math.max(0.0, Math.min(1.0, 1/this.goals[this.goals.length-1].dis));
      //goal_reward = 0.1 * goal_factor * proximity_reward;
    }
    */

    // agents like to go straight forward, more-so towards goals. // FIXME: "near" goals... side-effect, max towards goal.
    var forward_reward = 0.0;
    // TODO: Put thresholds in config.
    // TODO: Refactor to overloadable functions like `random_action`.
    if ((this.actionix === 0 || this.actionix === 1 || this.actionix === 2)) {
      // Some forward reward, some forward goal reward.
      // Instead of proximity threshold, a lower limit of 0.2.
      forward_reward = 0.1 * Math.pow(proximity_reward, 2);
      // Half as much for forward turns.
      if (this.actionix === 1 || this.actionix === 2) {
        forward_reward = forward_reward / 2;
      }
    }

    // agents like to eat good things
    var digestion_reward = this.digestion_signal;
    this.digestion_signal = 0.0;

    var reward = proximity_reward + forward_reward + goal_reward + digestion_reward;

    // Log repeating actions.
    // FIXME: Age stops increasing when not learning, spams log.
    if (this.brain.age % 10 === 0) {
      console.log(
        ' a:'+this.actionix,
        //'/'+this.repeat_cnt,
        ' =:'+reward.toFixed(5),
        ' p:'+proximity_reward.toFixed(3),
        ' f:'+forward_reward.toFixed(3),
        //' g:'+goal_reward.toFixed(3),
        ' d:'+digestion_reward.toFixed(3)
      );
    }

    // Pause publishing statistics when not learning.
    if (this.brain.learning) {
      //this.ros.pubTopic('/dqn/learning',   'std_msgs/Float32', this.ros.createStdMsg('float', (this.brain.learning)?1:0));
      //this.ros.pubTopic('/dqn/reward',     'std_msgs/Float32', this.ros.createStdMsg('float', reward));
      //this.ros.pubTopic('/dqn/action',   'std_msgs/Float32', this.ros.createStdMsg('float', this.actionix));
      //this.ros.pubTopic('/dqn/epsilon',    'std_msgs/Float32', this.ros.createStdMsg('float', this.brain.epsilon));
      this.ros.pubTopic('/dqn/avg_loss',   'std_msgs/Float32', this.ros.createStdMsg('float', this.brain.average_loss_window.get_average()));
      this.ros.pubTopic('/dqn/avg_reward', 'std_msgs/Float32', this.ros.createStdMsg('float', this.brain.average_reward_window.get_average()));
      //this.ros.pubTopic('/dqn/age',      'std_msgs/Float32', this.ros.createStdMsg('float', this.brain.age));
    }

    // pass to brain for learning
    this.brain.backward(reward);
  }
};

module.exports = Agent;
