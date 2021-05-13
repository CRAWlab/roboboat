## RoboBoat Thrust Mapping

Package that provides basic thrust mapping for the RoboBoat. It maps `cmd_vel` messages to raw thruster commands.

As of 05/13/21, we are trying to do this via `mavros`'s [`rc/override` topic](http://wiki.ros.org/mavros#mavros.2FPlugins.Subscribed_Topics-2). Core setup emulates that from [this post](https://discuss.bluerobotics.com/t/simulating-manual-control-using-mavros/1745/64).