## RoboBoat Thrust Mapping

Package that provides basic thrust mapping for the RoboBoat. It maps `cmd_vel` messages to raw thruster commands.

As of testing on 05/13/21, we are using the low-level Navio interface to the ESCs. The launch file now calls the node [roboboat_navio_thrust.py](https://github.com/CRAWlab/roboboat/blob/dev/roboboat_thrust_mapping/nodes/roboboat_navio_thrust.py). For more inforamtion on the low-level Navio interface, you can see [its documentation](https://docs.emlid.com/navio2/) and the examples in the [GitHub Repository](https://github.com/emlid/Navio2).

We were trying to do this via `mavros`'s [`rc/override` topic](http://wiki.ros.org/mavros#mavros.2FPlugins.Subscribed_Topics-2). Core setup emulates that from [this post](https://discuss.bluerobotics.com/t/simulating-manual-control-using-mavros/1745/64). This seemed not to work.
