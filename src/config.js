module.exports = {
  main_loop: 30, // Hz
  goal_distance: 20, // How many RatSLAM experiences ago to set goal?
  eyes: [
    /*
    {
      name: 'goal_rear_l', // TODO: Refactor `range_4l` 90deg to side.
      max_range: 4
    },
    */
    {
      name: 'range_3l',
      max_range: 4
    },
    {
      name: 'range_2l',
      max_range: 4
    },
    {
      name: 'range_1l',
      max_range: 4
    },
    {
      name: 'range_0',
      max_range: 4
    },
    {
      name: 'range_1r',
      max_range: 4
    },
    {
      name: 'range_2r',
      max_range: 4
    },
    {
      name: 'range_3r',
      max_range: 4
    }
    /*
    {
      name: 'goal_rear_r', // TODO: Refactor `range_4r` 90deg to side.
      max_range: 4
    }
    */
  ],
  sensors: [
    {
      name: 'goal_range',
      max_value: 20
    },
    {
      name: 'goal_direction',
      max_value: 360
    }
  ],
  actions: [
    [1.0,0.0],
    [1.0,-3.0], // FIXME: 4 will work... less forward? pub some of these 1 time to see.
    [1.0,3.0],
    [0.0,-4.0],
    [0.0,4.0]
  ],
  agent_opts: {
    // 27 = 9 eyes, each sees 3 numbers (wall, green, red thing proximity)
    // 36 = 9 eyes, each sees 3 numbers (wall, green, red thing proximity), and goals.
    // 25 = 7 eyes, 3 types, plus 2 sensors, with their active flag.
    num_inputs: 25,
    num_actions: 5,
    sensed_types: 3 // sensed_type is 0 for wall, 1 for food and 2 for poison.
  },
  brain_opts: {
    temporal_window: 1,
    behavior_policy: 'greedy', // TODO: Implement 'thompson' Dropout uncertainty.
    experience_size: 30000,
    start_learn_threshold: 1000,
    gamma: 0.7,
    learning_steps_total: 200000,
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
  }
};
