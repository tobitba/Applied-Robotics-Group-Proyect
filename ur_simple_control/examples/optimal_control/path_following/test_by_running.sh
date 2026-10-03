#!/bin/bash
# the idea here is to run all the runnable things
# and test for syntax errors 
# TODO: make these work with different modes by making robot.model a property which outputs truncated models

##########################################################################################################
#                                         fixed path
# ################################################################################################
# single arm - ee reference
# ###############
runnable="croco_ee_reference_path_following_mpc.py --ctrl-freq=-1 --no-plotter --no-visualizer --max-iterations=2 --no-draw-new --no-planner"
echo $runnable
python $runnable


####################################################################################
#                                path from planner
#################################################################33

runnable="croco_ee_reference_path_following_mpc.py --ctrl-freq=-1 --no-plotter --no-visualizer --max-iterations=2 --no-draw-new --planner"
echo $runnable
python $runnable
