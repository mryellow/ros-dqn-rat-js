var nostril_fov = 2*Math.PI/31; // 31 per revolution

var nostrils = [];
for (var x=1; x<=31; x++) {
  nostrils.push('nostril_'+x);
}

module.exports = {
  main_loop: 30, // Hz
  goal_distance: 20, // How many RatSLAM experiences ago to set goal?
  sensors: {
    eyes: {
      names: ['range_3l','range_2l','range_1l','range_0','range_1r','range_2r','range_3r'],
      fov: 15*Math.PI/180, // 15deg
      range: 4,
      types: 1
    },
    nostrils: {
      names: nostrils,
      fov: nostril_fov,
      range: 25,
      types: 1
    }
  },
  actions: [
    [1.0,0.0],
    [1.0,-3.0], // FIXME: 4 will work... less forward? pub some of these 1 time to see.
    [1.0,3.0],
    [0.0,-4.0],
    [0.0,4.0]
  ],
  brain_opts: {
    temporal_window: 2,
    behavior_policy: 'greedy', // TODO: Implement 'thompson' Dropout uncertainty.
    experience_size: 100000,
    start_learn_threshold: 1000,
    gamma: 0.7,
    learning_steps_total: 300000,
    learning_steps_burnin: 3000,
    epsilon_min: 0.05,
    epsilon_test_time: 0.05,
    // options for the Temporal Difference learner that trains the net
    // by backpropping the temporal difference learning rule.
    tdtrainer_options: {
      learning_rate: 0.001,
      momentum: 0.0,
      batch_size: 64,
      l2_decay: 0.01
    },
    // Prefer turning, to easier get unstuck while training.
    random_action_distribution: [0.1, 0.15, 0.15, 0.3, 0.3]
  },
  ratsim_opts: {
    goal_timeout: 60, // Seconds
    goal_model: 'rat_goal',
    goal_height: 1.25,
    goal_reached: 0.2,
    robot_model: 'kulbu',
    bounds: {
      x: {
        min: -5,
        max: 5
      },
      y: {
        min: -1,
        max: 5
      }
    }
  }
};
