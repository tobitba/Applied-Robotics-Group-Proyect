## goal
-----------
use crocoddyl to compute a whole trajectory in advance.
the trajectory can be followed with some other controller,
with controls here being feed-forward or whatever

# Massive todo
--------------
## path following
path following as currently defined is not good. you absolutely have to have it via a smooth parametrized path.
this does imply extending existing crocoddyl. but hey, you have to do it if you want correct path following.
## 


## alternative, easier for students
-------------------------------------
is to use casadi with pinocchio.casadi
as it's way more transparent.
examples of this stack are available in tutorials, hopefully they map
it to Simple Manipulator Control themselves 


