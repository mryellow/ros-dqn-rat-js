var ROSLIB = require('roslib');

/**
 * Robot.
 * @class Robot
 * @param {Ros} ros
 * @param {string} namespace
 * @param {string} topic
 */
var Robot = function(ros, namespace, topic) {
  if (!ros) throw new Exception('ROS instance must be passed to Robot.');
  this.ros          = ros;
  this.namespace    = (namespace)?namespace:'/';
  this.topic_cmdvel = (topic)?topic:'cmd_vel';
};

/**
 * Execute a move action
 * @method doMove
 * @param {float} linX
 * @param {float} angZ
 */
Robot.prototype.doMove = function(linX, angZ) {
  //console.log('Robot_doMove');
  var twist = new ROSLIB.Message({
    linear: {
      x: linX,
      y: 0,
      z: 0
    },
    angular: {
      x: 0,
      y: 0,
      z: angZ
    }
  });

  this.ros.pubTopic(
    this.namespace + '/' + this.topic_cmdvel,
    'geometry_msgs/Twist',
    twist
  );
};

/**
 * Subscribe to range sensors
 * @method subRange
 * @param {string} name
 * @param {function} callback
 */
Robot.prototype.subRange = function(name, callback) {
  //console.log('Robot_subRange', name);
  this.ros.subTopic(
    this.namespace + '/' + name,
    'sensor_msgs/Range',
    callback
  );
};

module.exports = Robot;
