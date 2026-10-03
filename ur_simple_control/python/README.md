# installation
------------
first you MUST update pip, otherwise it won't work:
python3 -m pip install --upgrade pip setuptools wheel build
then install with 
pip install --user -e . 
from this directory. now the package is editable, which there's a chance you'd want

# description
---------
- organized as a library called smc (Simple Manipulator Control), 
  made into a python package. the hope is that if this
  is done well enough, you could mix-and-match different components
  and just have them work as intended. on the other hand,
  the code should still be simple enough to afford the quickest possible prototyping,
  which is the point of having it all in python anyway
- initial python solution is age-old code which needs to be remapped into the 
  libraries used here. it will sit here until everything it offers has been translated.
  this primarily concerns all the inverse kinematics algorithms in there,
  primarily QP 
    --> which could be outsourced to the pink library as it's literally QP ik,
        but written competently and tested 

# runnable things
---------------
are in the examples folder
