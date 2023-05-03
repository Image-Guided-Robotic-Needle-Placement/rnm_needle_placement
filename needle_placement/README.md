**To run the code from this directory**

```ros
$ cd catkin_ws/src
$ git clone git@github.com:Image-Guided-Robotic-Needle-Placement/rnm_needle_placement.git
$ cd rnm_needle_placement && git checkout selva
```
**To make it in ros package structure**

```ros
$ mv needle_placement ../
$ mv README.md ../ && cd ..
$ rm -rf rnm_needle_placement
```

**Building the package**

- Make sure you are in `$ catkin_ws/src`

```ros
catkin_make
```

**To run the node**

```ros
rosrun needle_placement forward_kinematics.py
```


