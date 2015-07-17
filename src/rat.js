var ROSLIB = require('roslib');

/**
 * RatSLAM.
 * @class Rat
 * @param {Ros} ros
 * @param {string} namespace
 */
var Rat = function(ros, namespace) {
  if (!ros) throw new Exception('ROS instance must be passed to RatSLAM.');
  this.ros          = ros;
  this.namespace    = (namespace)?namespace:'/ratslam';
};

/**
 * Create goal on Experience Map
 * @method createGoal
 * @param {float} x
 * @param {float} y
 */
Rat.prototype.createGoal = function(x, y) {
  //console.log('Rat_createGoal');
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
        x: parseFloat(x),
        y: parseFloat(y),
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
  console.log('goal', goal.pose.position);
  this.ros.pubTopic(
    this.namespace + '/ExperienceMap/SetGoalPose',
    'geometry_msgs/PoseStamped',
    goal
  );
};

/**
 * Subscribe to RatSLAM experiences.
 * @method subMap
 * @param {function} callback
 */
Rat.prototype.subMap = function(callback) {
  //console.log('Rat_subMap');
  this.ros.subTopic(
    this.namespace + '/ExperienceMap/Map',
    'ratslam_ros/TopologicalMap',
    callback
  );
};

/**
 * Subscribe to sub goals from RatSLAM.
 * @method subGoal
 * @param {function} callback
 */
Rat.prototype.subGoal = function(callback) {
  //console.log('Rat_subGoal');
  this.ros.subTopic(
    this.namespace + '/ExperienceMap/SubGoal',
    'ratslam_ros/TopologicalGoal',
    callback
  );
};

module.exports = Rat;
