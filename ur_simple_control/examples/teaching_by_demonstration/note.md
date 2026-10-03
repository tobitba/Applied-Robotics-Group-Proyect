2 options for data collection
-----------------------------
- a) collect only point-postures and/or end-effector poses
- b) collect whole trajectories

2 options for the underlying controller
----------------------------------------
- a) just put the robot to freedrive
- b) compliant controller where it tries to remain at the spot you pushed it to

3 options total depending on what you collected
-----------------------------------------------
- a) path planning + traj. gen. through selected list of postures/poses
- b) simply replay the collected trajectory
- c) dmp multiple trajectories onto themselves
