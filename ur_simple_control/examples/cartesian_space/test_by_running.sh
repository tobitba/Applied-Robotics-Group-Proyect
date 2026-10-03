#!/bin/bash
# the idea here is to run all the runnable things
# and test for syntax errors 
#
# ################
# single arm 
# ###############
# damped pseudoinverse
runnable="clik_point_to_point.py --randomly-generate-goal --ctrl-freq=-1 --no-plotter --no-visualizer --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="clik_point_to_point.py --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="clik_point_to_point.py --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

######################
# whole body single arm
######################
# damped pseudoinverse 
runnable="clik_point_to_point.py --robot=heron --randomly-generate-goal --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="clik_point_to_point.py --robot=heron --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="clik_point_to_point.py --robot=heron --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

############################
# whole body single arm only
############################
# damped pseudoinverse 
runnable="clik_point_to_point.py --robot=heron --mode upper_body --randomly-generate-goal --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="clik_point_to_point.py --robot=heron --mode upper_body --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="clik_point_to_point.py --robot=heron --mode upper_body --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

######################
# dual arm
# ####################
# damped pseudoinverse
runnable="dual_arm_clik.py --robot=yumi --randomly-generate-goal  --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="dual_arm_clik.py --robot=yumi --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="dual_arm_clik.py --robot=yumi --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

######################
# dual arm, left arm only
# ####################
# damped pseudoinverse
runnable="clik_point_to_point.py --robot=yumi --mode=left_arm_only --randomly-generate-goal  --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="clik_point_to_point.py --robot=yumi --mode=left_arm_only --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="clik_point_to_point.py --robot=yumi --mode=left_arm_only --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

######################
# dual arm, right arm only
# ####################
# damped pseudoinverse
runnable="clik_point_to_point.py --robot=yumi --mode=right_arm_only --randomly-generate-goal  --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="clik_point_to_point.py --robot=yumi --mode=right_arm_only --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="clik_point_to_point.py --robot=yumi --mode=right_arm_only --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# ##########################
# whole body dual arm
# ##########################
# damped pseudoinverse 
runnable="dual_arm_clik.py --robot=myumi --randomly-generate-goal  --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="dual_arm_clik.py --robot=myumi --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="dual_arm_clik.py --robot=myumi --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# ##########################
# whole body dual arm, arms only
# ##########################
# damped pseudoinverse 
runnable="dual_arm_clik.py --robot=myumi --mode=upper_body --randomly-generate-goal  --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="dual_arm_clik.py --robot=myumi --mode=upper_body --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="dual_arm_clik.py --robot=myumi --mode=upper_body --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

######################
# whole body dual arm, left arm only
# ####################
# damped pseudoinverse
runnable="clik_point_to_point.py --robot=myumi --mode=left_arm_only --randomly-generate-goal  --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="clik_point_to_point.py --robot=myumi --mode=left_arm_only --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="clik_point_to_point.py --robot=myumi --mode=left_arm_only --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

######################
# whole body dual arm, right arm only
# ####################
# damped pseudoinverse
runnable="clik_point_to_point.py --robot=myumi --mode=right_arm_only --randomly-generate-goal  --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP quadprog
runnable="clik_point_to_point.py --robot=myumi --mode=right_arm_only --randomly-generate-goal --ik-solver=QPquadprog --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable

# QP proxsuite
runnable="clik_point_to_point.py --robot=myumi --mode=right_arm_only --randomly-generate-goal --ik-solver=QPproxsuite --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=20000"
echo $runnable
python $runnable
