#!/bin/bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /home/student/miniconda3/miniconda.sh
bash /home/student/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm /home/student/miniconda3/miniconda.sh
export PATH=/home/student/miniconda3/bin:$PATH
source /home/student/miniconda3/bin/activate
pip install -e ./SimpleManipulatorControl/python/
conda config --add channels conda-forge 
conda install --solver=classic -y pinocchio crocoddyl -c conda-forge
pip install matplotlib meshcat ur_rtde argcomplete \
            qpsolvers ecos example_robot_data meshcat_shapes \
            pyqt6 opencv-python

