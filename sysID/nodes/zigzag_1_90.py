#! /usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
# Initialize the node
rospy.init_node('sysID_vel_commands', anonymous=True)
# Create a publisher to publish cmd_vel tell it to move
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
# 10 Hz update rate - update to match your setup
rate = rospy.Rate(10)
time_step=1/10
# Define the move command's Twist as a Twist message
move_cmd = Twist()
seq=0

#define vel commands


desired_surge_vel=1
#complete 180 turn in 2 seconds
time2turn=2 
desired_yaw_vel=np.pi/time2turn      
turnsteps=int(time2turn/time_step)
turndirection=1

vels=np.zeros([int(64/time_step),3])
stagel=np.trunc(int(60/time_step)/10)

vels[0:int(2/time_step),0]=np.linspace(0,desired_surge_vel,int(2/time_step))
vels[int(2/time_step):,0]=desired_surge_vel
vels[-int(2/time_step):,0]=np.linspace(desired_surge_vel,0,int(2/time_step))

vels[0:int(2/time_step),2]=-np.pi/4
cstep=int(2/time_step)

for ii in range(0,10):
    CenterOfTurnS=cstep+int(stagel/2)
    turnstart=int(np.trunc(CenterOfTurnS-turnsteps))
    turnend=int(np.trunc(CenterOfTurnS+turnsteps)+1)
    vels[turnstart:turnend+1,2]=desired_yaw_vel*turndirection
    turndirection=turndirection*-1
    cstep+=stagel

vels[-int(2/time_step):,2]=np.pi/4


try:
    while not rospy.is_shutdown():        
            # assign the translational velocity commands
            move_cmd.linear.x = vels[seq,0]
            move_cmd.linear.y = vels[seq,1]
            # assign the angular velocity command
            move_cmd.angular.z = vels[seq,2]
            #stop commands after path complete
            if seq>=len(vels[0]):
                move_cmd.linear.x = 0
                move_cmd.linear.y = 0
                move_cmd.angular.z = 0
            # Then publish it
            cmd_vel.publish(move_cmd)
            rate.sleep()
            cmd_vel.publish(move_cmd)
            seq+=1
except (KeyboardInterrupt, SystemExit):
    cmd_vel.publish(Twist()) # publish all zeros to stop