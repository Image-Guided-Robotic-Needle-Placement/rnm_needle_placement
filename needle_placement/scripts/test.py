#!/usr/bin/env python3
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym
import roboticstoolbox as rtb

panda = rtb.models.URDF.Panda()
print(panda)

point = SE3(0.6, -0.5, 0.0)
point_sol = panda.ikine_LM(point) 
print("\nInverse Kinematics Solution :\n" ,point_sol)