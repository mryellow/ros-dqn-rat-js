var ROSLIB = require('roslib');

/**
 * ROSLibJS abstractionwrapper.
 * @class Rat
 */
var Ros = function(uri) {
  this._subs = [];
  this._pubs = [];
  this.uri   = (uri)?url:'ws://localhost:9090';

  // FIXME: Recreate if URI changes. Fine for using construtor to set...
  this._ros = new ROSLIB.Ros({
    url: this.uri
  });

  this._ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });
  // TODO: Throw errors.
  this._ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });
  this._ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });
};

/**
 * Create topic for pub/sub
 * @method createTopic
 * @param {string} topic
 * @param {string} type
 * @param {integer} rate
 * @return {ROSLib.Topic}
 */
Ros.prototype.createTopic = function(topic, type, rate) {
  //console.log('Ros_createTopic', topic, type);
  if (!rate) rate = 0;
  if (!topic || !type) return;
  return new ROSLIB.Topic({
    ros:           this._ros,
    name:          topic,
    messageType:   type,
    throttle_rate: rate
  });
};

/**
 * Generate various `std_msgs`
 * @method createStdMsg
 * @param {string} type
 * @param {mixed} val
 * @return {ROSLIB.Message}
 */
Ros.prototype.createStdMsg = function(type, val) {
  //console.log('Ros_createMsg', type, val);
  switch (type) {
    case 'float':
      return new ROSLIB.Message({
        data: parseFloat(val)
      });

    default:
      return null;
  }
};

/**
 * Publish a ROS message
 * @method pubTopic
 * @param {string} topic
 * @param {string} type
 * @param {ROSLIB.Message} msg
 */
Ros.prototype.pubTopic = function(topic, type, msg) {
  //console.log('Ros_pubTopic');
  // Create topic if it doesn't exist.
  if (!this._pubs[topic]) {
    this._pubs[topic] = this.createTopic(
      topic,
      type
    );
  }
  this._pubs[topic].publish(msg);
};

/**
 * Subscribe to a ROS topic
 * @method subTopic
 * @param {string} topic
 * @param {string} type
 * @param {function} callback
 * @param {integer} rate
 */
Ros.prototype.subTopic = function(topic, type, callback, rate) {
  //console.log('Ros_subTopic');
  if (!rate) rate = 0;
  var sub = this.createTopic(topic, type, rate).subscribe(callback);
  this._subs.push(sub);
};

module.exports = Ros;
