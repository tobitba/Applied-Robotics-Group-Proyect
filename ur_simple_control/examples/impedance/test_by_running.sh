#!/bin/bash

# joint-space point impedance ur5e
runnable="point_impedance_control.py --robot=ur5e --no-cartesian-space-impedance --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2000"
echo $runnable
python $runnable

# joint-space point impedance heron
runnable="point_impedance_control.py --robot=heron --no-cartesian-space-impedance --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2000"
echo $runnable
python $runnable

# cartesian-space point impedance ur5e
runnable="point_impedance_control.py --robot=ur5e  --cartesian-space-impedance --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2000"
echo $runnable
python $runnable

# cartesian-space point impedance heron
runnable="point_impedance_control.py --robot=heron --cartesian-space-impedance --ctrl-freq=-1 --no-visualizer --no-plotter --max-iterations=2000"
echo $runnable
python $runnable
