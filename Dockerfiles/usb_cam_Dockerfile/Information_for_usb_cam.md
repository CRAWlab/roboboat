## RPi -- Docker-compose up 
     -- change directories to the /usb_cam_node repo containing docker-compose.yml file and run "docker-compose up"
     -- after you capture what you need hit CTRL+c and wait for it to exit then run "docker-compose down"        
     -- you can use sftp pi@<ip>:/home/pi/BagData/<name_of_Bagfile> on the host machine and it will copy files from the RPi to the current directory on the host machine 
     -- Then you will need to fix the bagfile
     -- Must run rosbag reindex <BagFileName.bag.active> 
     -- Then run rosbag fix <FixedBagFile.bag.active> <NewBagFileName.bag>
     --Launch Rviz to see data and rosbag play <Bagfile>