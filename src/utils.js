var fs = require('fs');
var path = require('path');

/**
 * Brain utils.
 * @class Utils
 * @param {Ros} ros
 * @param {string} namespace
 * @param {Agent} agent
 */
var Utils = function(ros, namespace, agent) {
  if (!ros) throw new Exception('ROS instance must be passed to Utils.');
  var _self = this;

  this.ros          = ros;
  this.namespace    = (namespace)?namespace:'/dqn';
  this.agent        = agent;

  // Subscribe to topic for pause/resume of DQN.
  // Allowing for restart of RatSLAM, which doesn't like movement during init.
  this.dqn_paused = false;

  /**
   * Pause/resume DQN.
   */
  this.ros.subTopic(
    this.namespace + '/pause',
    'std_msgs/Bool',
    function(message) {
      // TODO: Re-factor `learning` into separate switch.
      _self.agent.brain.learning = this.dqn_paused = (message.data);

      // Reset goal sensors.
      // TODO: Encapsulate, duplication.
      var num_eyes = agent.eyes.length;
      for (var i=0; i<num_eyes; i++) {
        var e = agent.eyes[i];
        e.sensed_goal = e.goal_range;
      }
      agent.goals = [];

      console.log('DQN Paused', (message.data));
    }
  );

  /**
   * Set DQN Network age.
   */
  // FIXME: Not a string.
  this.ros.subTopic(
    this.namespace + '/set_age',
    'std_msgs/String',
    function(message) {
      _self.agent.brain.age = parseInt(message.data);
      console.log('DQN age set '+ message.data);
    }
  );

  /**
   * Load DQN from JSON file.
   */
  this.ros.subTopic(
    this.namespace + '/load',
    'std_msgs/String',
    function(message) {
      // Sanitise filename.
      var file = message.data.replace(/[^a-z0-9]/i, '_');

      // TODO: Configurable path to brains.
      fs.readFile(path.resolve(__dirname, '../brains/'+file+'.json'), 'UTF-8', function read(err, data) {
        if (err) {
          return console.log('Error', err);
        }

        // Load the JSON.
        _self.agent.brain.value_net.fromJSON(JSON.parse(data));

        // TODO: Configuration left up to the user.
        _self.agent.brain.age = 2 * _self.agent.brain.learning_steps_total;
        // FIXME: Pause learning, or increase age?
        //_self.agent.brain.learning = false;

        // Reset goal sensors.
        // TODO: Encapsulate, duplication.
        var num_eyes = agent.eyes.length;
        for (var i=0; i<num_eyes; i++) {
          var e = agent.eyes[i];
          e.sensed_goal = e.goal_range;
        }
        _self.agent.goals = [];

        console.log('DQN loaded from '+ file);
      });
    }
  );

  /**
   * Save DQN to JSON file.
   */
  this.ros.subTopic(
    this.namespace + '/save',
    'std_msgs/String',
    function(message) {
      // Sanitise filename.
      var file = message.data.replace(/[^a-z0-9]/i, '_');
      var json = _self.agent.brain.value_net.toJSON();
      fs.writeFile(path.resolve(__dirname, '../brains/'+file+'.json'), JSON.stringify(json, null, 1), {encoding: 'utf8'}, function(err) {
        if (err) {
          return console.log('Error', err);
        }
        console.log('DQN saved to '+ file);
      });
    }
  );

};

module.exports = Utils;
