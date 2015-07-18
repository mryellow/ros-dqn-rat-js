/**
 * Eye sensor has a maximum range and senses walls
 * @class
 * @constructor {object} input
 */
var Eye = function(input) {
  console.log('Creating eye', input);
  this.name             = (input && input.name)?      input.name:'';
  this.angle            = (input && input.angle)?     input.angle:0;
  this.fov              = (input && input.fov)?       input.fov:(15*Math.PI/180); // Default 15deg.
  this.max_range        = (input && input.max_range)? input.max_range:4;
  this.sensed_proximity = this.max_range;
  this.sensed_type      = -1; // what does the eye see?

  // Extra sensor dealing exclusively with goals.
  // TODO: Refactor to binary, "nose". Could be interesting with forward turns.
  this.goal_range       = (input && input.goal_range)?input.goal_range:10;
  this.sensed_goal      = this.goal_range;

  // Watch for updates, syncing framerate to sensors.
  this.updated = false;

  // TODO: Watch for goal updates also?
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
 * @param {array} eyes
 * @param {array} actions
 * @param {object} agent_opts
 * @param {object} brain_opts
 */
var Agent = function(ros, eyes, actions, agent_opts, brain_opts) {
  if (!ros) throw new Exception('ROS instance must be passed to RatSLAM.');
  this.ros = ros;

  // Initialise eyes from config passed in.
  this.eyes = [];
  for (var i = 0; i < eyes.length; i++) {
    if (typeof(eyes[i].angle) !== "undefined" && typeof(eyes[i].fov) !== "undefined") {
      this.eyes.push(new Eye(eyes[i]));
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
  this.goals = [];

  var num_inputs      = agent_opts.num_inputs;
  var num_actions     = agent_opts.num_actions;
  var temporal_window = brain_opts.temporal_window; // amount of temporal memory. 0 = agent lives in-the-moment :)
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
      //if (e.sensed_goal < e.goal_range) console.log('a', i, e.sensed_goal, Math.min(e.goal_range, e.sensed_goal)/e.goal_range);
    }
    proximity_reward = proximity_reward/(num_eyes-2); // FIXME: Hack to remove 2 extra eyes.
    proximity_reward = Math.min(1.0, proximity_reward);

    // agents like to be near goals
    var goal_factor = 0.0;
    var goal_reward = 0.0;

    if (this.goals[this.goals.length-1] && this.goals[this.goals.length-1].dis > 0) {
      // FIXME: Only if below max, so there is a reading to correlate with?
      goal_factor = Math.max(0.0, Math.min(1.0, 1/this.goals[this.goals.length-1].dis));
      // FIXME: Redundant, forward bonus and goal view implies this.
      //goal_reward = 0.1 * goal_factor * proximity_reward;
    }

    // agents like to go straight forward, more-so towards goals. // FIXME: "near" goals... side-effect, max towards goal.
    var forward_reward = 0.0;
    // TODO: Reward fast turns only once epsilon drops?
    if ((this.actionix === 0 || this.actionix === 1 || this.actionix === 2) && proximity_reward > 0.85) { // 0.75 a little too close for corners. Rewards stuck wheel.
      // TODO: Closer to goal = more forward. Instead of eye reward, stop goal reward when goal is behind?

      // Some forward reward, some forward goal reward.
      //forward_reward = (0.05 + (0.05 * goal_factor)) * proximity_reward;
      forward_reward = 0.05 * proximity_reward;
      // Half as much for forward turns.
      //if (this.actionix === 1 || this.actionix === 2) {
      //  forward_reward = forward_reward / 2;
      //}
    }

    // agents like to eat good things
    var digestion_reward = this.digestion_signal;
    this.digestion_signal = 0.0;

    var reward = proximity_reward + forward_reward + goal_reward + digestion_reward;

    // Log repeating actions.
    if (this.cnt > 4) {
      console.log(reward.toFixed(5), this.actionix, this.cnt, this.linX.toFixed(1), this.angZ.toFixed(1));
      console.log('rewards', forward_reward, goal_reward, digestion_reward);
    }

    // Publish statistics to ROS topics.
    this.ros.pubTopic('/dqn/learning',   'std_msgs/Float32', this.ros.createStdMsg('float', (this.brain.learning)?1:0));
    this.ros.pubTopic('/dqn/reward',     'std_msgs/Float32', this.ros.createStdMsg('float', reward));
    //this.ros.pubTopic('/dqn/action',   'std_msgs/Float32', this.ros.createStdMsg('float', this.actionix));
    this.ros.pubTopic('/dqn/epsilon',    'std_msgs/Float32', this.ros.createStdMsg('float', this.brain.epsilon));
    //this.ros.pubTopic('/dqn/avg_loss', 'std_msgs/Float32', this.ros.createStdMsg('float', this.brain.average_loss_window.get_average()));
    this.ros.pubTopic('/dqn/avg_reward', 'std_msgs/Float32', this.ros.createStdMsg('float', this.brain.average_reward_window.get_average()));
    //this.ros.pubTopic('/dqn/age',      'std_msgs/Float32', this.ros.createStdMsg('float', this.brain.age));

    // pass to brain for learning
    this.brain.backward(reward);
  }
};

module.exports = Agent;
