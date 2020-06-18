/* ----------------------------------------------------------------------------

PointCloudSync.cpp

PointCloud2 concatenataion from the two ZED cameras

Created: 06/18/2020
   - Benjamin Armentor
   - benarmentor@gmail.com

 Modified:
   *

TODO:
   * 
---------------------------------------------------------------------------- */

// Might need to link directories in CMakeLists.txt
#include <pcl_ros/io/concatenate_data.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Define namespaces
using namespace std;

// How often the main loop should run
const double LOOP_FREQ = 10; // Loop frequency (Hz)

// Define message containers
sensor_msgs::PointCloud2 bow_PCL;
sensor_msgs::PointCloud2 stern_PCL;
sensor_msgs::PointCloud2 merged_PCL;

// Callback function for Point Clouds from the ZED camera mounted at the bow
void bow_callback(const sensor_msgs::PointCloud2 msg)
{
  bow_PCL.header = msg.header;
  bow_PCL.height = msg.height;
  bow_PCL.width = msg.width;
  bow_PCL.fields = msg.fields;
  bow_PCL.is_bigendian = msg.is_bigendian;
  bow_PCL.point_step = msg.point_step;
  bow_PCL.row_step = msg.row_step;
  bow_PCL.data = msg.data;
  bow_PCL.is_dense = msg.is_dense;

  return;
}

//Callback function for Point Clouds from the ZED camera mounted at the stern
void stern_callback(const sensor_msgs::PointCloud2 msg)
{
  stern_PCL.header = msg.header;
  stern_PCL.height = msg.height;
  stern_PCL.width = msg.width;
  stern_PCL.fields = msg.fields;
  stern_PCL.is_bigendian = msg.is_bigendian;
  stern_PCL.point_step = msg.point_step;
  stern_PCL.row_step = msg.row_step;
  stern_PCL.data = msg.data;
  stern_PCL.is_dense = msg.is_dense;

  return;
}

// Main function
int main(int argc, char **argv)
{

  // Initialize ROS and the node
  ros::init(argc, argv, "PointCloudSync");

  // Finish initializing the node via NodeHandle
  // Second NodeHandle call will close the node down.
  ros::NodeHandle n;

  // Define the rate of the main loop. Effectively the sampling rate of the controller
  ros::Rate loop_rate(LOOP_FREQ); // (Hz)

  // Define the subscribers
  ros::Subscriber bow_sub = n.subscribe("/bow/zed/points2", 1, bow_callback);
  ros::Subscriber stern_sub = n.subscribe("/stern/zed/points2", 1, stern_callback);

  // Initialize a Point Cloud Combiner
  pcl_ros::PointCloudConcatenateDataSynchronizer::combineClouds cloudCombiner;

  // Main ROS loop
  while (ros::ok())
  {
    // Prevent the loop from saturating a CPU core
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}