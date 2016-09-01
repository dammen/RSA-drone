*** Repo for Robot Software Architecture Assignment 2 ***

The AR Drone creates its own wifi that you need to connect to in order to receive data from the drone.

To do this, on your computer connect to the drones wifi (its called ar_dronexxxx where xxxx are some numbers).
Then in virtual machine, you should check you IP is something along the lines of (cant remember, but different to unswrobotics IP. Will update when i find out).

Make sure your ROS IP is set to this and ROS_MASTER_URI.

If you put the following in your .bashrc, you (should) never have to worry about it again:

`export ROS_IP=/`hostname -I | tr -d ' '/``
export ROS_MASTER_URI=http://${ROS_IP}:11311