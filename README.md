# TIAGo mobile robot utilizing automated planning and scheduling in support for the elderly
The `rosplan_interface_tiago` packages are implementation of my thesis (of which abstract is below).
The most work is put into TA, AHFP & HD scenarios and only these are discussed in the thesis.

## Abstract
This work contains the design and description of the implementation process of an application that aims to use a social robot to support elderly.
It is based on famous Robot Operating System.
One of the main assumptions for this application is to extract certain scenarios of the robot's work from all the actions it can perform.

The part related to automated planning and scheduling is implemented using the PDDL language.
It was then integrated with the rest of the ROS based application using the ROSPlan framework.
PDDL domains were modelled so they would represent mentioned scenarios of the robot's work.
The robot is able to perform some specific tasks (called actions) within each of them.
Each such PDDL action has a form of a finite state machine created using the SMACH package, which forced the division of all robot actions into certain basic, indivisible states.
The whole application was implemented using the TIAGo mobile robot.
Its simulator was also used for research and testing purposes.

Thanks to the nature of the PDDL language and the use of finite state machines, the application has a modular structure - it is easy to develop and understand.
The system can be run using both real robot and its simulator effortlessly, which improves research and testing.
In particular, it can be easily implemented on another similar mobile robot.

The realization of the work has helped to understand the difficulties in the field of elderly care.
It also highlights the advantages and disadvantages of using automated planning in social robotics, especially the essence of its use in this environment.

## Requirements
Minimal requirements:
* ROS Kinetic
* PAL's TIAGo simulation workspace
* Workspace with `rosplan_interface_tiago` packages must expand the PAL's one
