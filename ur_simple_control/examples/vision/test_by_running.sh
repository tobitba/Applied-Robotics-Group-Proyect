#!/bin/bash
# NOTE: it works if:
# 1) a window pops up with opencv camera feed
# 2) the plotter produces random result (it's random test data representing camera processing output)
runnable="./vision/camera_no_lag.py --max-iterations=1500 --no-visualize-manipulator"
echo $runnable
python $runnable
