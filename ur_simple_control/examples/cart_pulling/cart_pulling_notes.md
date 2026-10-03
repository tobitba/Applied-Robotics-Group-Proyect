# stuff to try
--------------
once the the basic version of the control is done you can try putting in something extra to the objective function of the ocp maybe?

# a thing to take care of
-------------------------
if the end-effector is not at the starting point of the path, it goes to shit. it can't make the path because the path is constantly changing.
there are 3 different ways to solve the problem:
1) don't update the path until the end-effector is close enough
2) run a behavior tree where you don't do path following unless you are at the path, and have a different controller get you to the point (this should be just initialization)
3) transform the path differently so that it goes to the desired height, i.e. change the path to make it followable

discussion.
1) is not a good idea because we want the path planner to run on the side and do it's thing. it is resposible for the path, control is responsible to follow the path. the path is guaranteed to exist, and it is our duty to deal with non-linearity to get us on the path.
2) a totally valid solution and makes sense in the final version. does add some complexity because you then need to make sure you designed the state transitions well.
3) just a bit of math, can't fail. does not fuck with the path planner, introduces no new concepts - it's just a different transformation of the 2D path. once put in place there is no reason for it not to work.

conclusion:
we go with option 2) because option 2) is false! it does not account for orientation! we want a fixed orientation which changes along the path. this makes interpolating to the 6d path weird. hence we need to swap to a clik if the initial error is too big.
you could also try "freezing" the first point tbh


# should the reference for the base be just a translation or a full pose?
-------------------------------------------------------------------------------
on one level we don't care, but then again do know what it should be. so idk 
- ideally just test this shit (annoying as fuck to implement but what can you do)



# the desided joint space position
name:
- myumi_001_yumi_robr_joint_1
- myumi_001_yumi_robr_joint_2
- myumi_001_yumi_robr_joint_3
- myumi_001_yumi_robr_joint_4
- myumi_001_yumi_robr_joint_5
- myumi_001_yumi_robr_joint_6
- myumi_001_yumi_robr_joint_7
- myumi_001_yumi_robl_joint_1
- myumi_001_yumi_robl_joint_2
- myumi_001_yumi_robl_joint_3
- myumi_001_yumi_robl_joint_4
- myumi_001_yumi_robl_joint_5
- myumi_001_yumi_robl_joint_6
- myumi_001_yumi_robl_joint_7
position:
- 1.1781062998438787
- -0.000595335227562165
- -1.749208945070357
- 0.4106104712156747
- -2.060408639509931
- 0.3044975499507672
- 1.7246295661055797
- -0.7019778380440242
- 0.0394624047824147
- 1.1381740635109037
- 0.40438559848372346
- 1.5945463973962233
- 0.3724310942226761
- -1.3882579393586854
velocity: []
effort: []

# better
name:
- myumi_001_yumi_robr_joint_1
- myumi_001_yumi_robr_joint_2
- myumi_001_yumi_robr_joint_3
- myumi_001_yumi_robr_joint_4
- myumi_001_yumi_robr_joint_5
- myumi_001_yumi_robr_joint_6
- myumi_001_yumi_robr_joint_7
- myumi_001_yumi_robl_joint_1
- myumi_001_yumi_robl_joint_2
- myumi_001_yumi_robl_joint_3
- myumi_001_yumi_robl_joint_4
- myumi_001_yumi_robl_joint_5
- myumi_001_yumi_robl_joint_6
- myumi_001_yumi_robl_joint_7
position:
- 1.177726666230488
- -0.0014802503425588292
- -1.7605729197578341
- 0.4215065284996249
- -1.9415690706067803
- 0.2803584248036936
- 1.6919710884998835
- -0.7879804975484437
- 0.22470208644656367
- 1.3406988091378869
- 0.35854547216754384
- 1.7128065956042273
- 0.22850344929702915
- -1.5302555581528383
velocity: []
effort: []


