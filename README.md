# Module description

This module allows to carry out a three-finger stable grasp, according to the Task 3-1 of the TACMAN project. First, the fingers move towards the object and stop when a contact is detected, then the hand configuration changes to improve stability, exploiting a Gaussian mixture model trained using learning by demonstration.

## Dependencies

* YARP
* icub-main
* icub-contrib

## How to compile

For Linux users: go to the folder `plant-identification` and type the following commands.
```
mkdir build
cd build
ccmake ..
make install
```

## How to run

* run `skinManager`
* run `plantIdentification --whichHand <hand> --icub <icub>`, where:
	- `<hand>` is the hand to use (left/right)
	- `<icub>` is the iCub to use (iit_black/iit_purple/tum_darmstadt)
* connect to the rpc port `/plantIdentification/cmd:i`, several commands are available, in particular:
	- `grasp` runs the stable grasp
	- `open` stops the current task and open the hand
	- `help` lists all the other commands available

## Publications

Hierarchical Grasp Controller using Tactile Feedback,
M. Regoli, U. Pattacini, G. Metta, L. Natale,
IEEE International Conference on Humanoid Robots, 2016