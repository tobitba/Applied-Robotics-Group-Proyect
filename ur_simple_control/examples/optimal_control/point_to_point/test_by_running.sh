#!/bin/bash
# the idea here is to run all the runnable things
# and test for syntax errors 
# TODO: make these work with different modes by making robot.model a property which outputs truncated models

##########################################################################################################
#                                         point to point
# ################################################################################################
# single arm - ee reference
# ###############
# ocp
runnable="croco_ee_reference_p2p_ocp.py --ctrl-freq=-1 --no-plotter --no-visualizer --max-iterations=2"
echo $runnable
python $runnable

# mpc
runnable="croco_ee_reference_p2p_mpc.py --max-solver-iter 10 --n-knots 30  --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2"
echo $runnable
python $runnable

# whole body single arm - ee reference
# -------------------------------------
# ocp
runnable="croco_ee_reference_p2p_ocp.py --robot=heron --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2"
echo $runnable
python $runnable
# mpc
runnable="croco_ee_reference_p2p_mpc.py --max-solver-iter 10 --n-knots 30 --robot=heron --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2"
echo $runnable
python $runnable

# dual arm - dual ee reference
# ocp TODO: missing

# mpc
runnable="croco_dual_ee_reference_p2p_mpc.py --max-solver-iter 10 --n-knots 30 --robot=yumi --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2"
echo $runnable
python $runnable

# whole body dual arm - dual ee reference
# --------------------------
# ocp TODO: missing
#
# mpc
runnable="croco_dual_ee_reference_p2p_mpc.py --max-solver-iter 10 --n-knots 30 --robot=myumi --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2"
echo $runnable
python $runnable

# whole body single arm  - base + ee reference
# ----------------------------------
# ocp TODO: missing
#
# mpc
runnable="croco_base_and_ee_reference_p2p.py --max-solver-iter 10 --n-knots 30 --robot=heron --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2"
echo $runnable
python $runnable
# 
#
# whole body dual arm base + ee reference
# ----------------------------------
# ocp TODO: missing
# mpc 
runnable="croco_base_and_dual_ee_reference_p2p.py --max-solver-iter 10 --n-knots 30 --robot=myumi --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2"
echo $runnable
python $runnable
