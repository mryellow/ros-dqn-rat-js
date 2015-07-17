module.exports = {
  main_loop: 30, // Hz
  goal_distance: 20,
  max_range: 4,
  goal_range: 10,
  eye_names: [
    'goal_rear_l',
    'range_3l',
    'range_2l',
    'range_1l',
    'range_0',
    'range_1r',
    'range_2r',
    'range_3r',
    'goal_rear_r'
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
    num_inputs: 36,
    num_actions: 5
  },
  brain_opts: {
    temporal_window: 10,
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
