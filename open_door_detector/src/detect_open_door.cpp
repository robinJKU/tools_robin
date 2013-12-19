/**
 * Title:	Open Door Detector
 * Author: 	Reiter Andreas & Reisenberger Johannes
 * Date:	19.12.2013
 * Summary:	Provides a service to detect if a door is open. If this is the case, it returns the position of the middle of the door.
 * 		Otherwise it returns position 0.
 * Input Arguments:	aperture_angle
 *			wall_distance
 *
 * Output Argument:	door_pos	
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "open_door_detector/detect_open_door.h"

bool detect_door(open_door_detector::detect_open_door::Request  &req,
         open_door_detector::detect_open_door::Response &res);

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

const int DETECTION_THRESHOLD=25;

//global Param
sensor_msgs::LaserScan last_scan_in;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_open_door");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("detect_open_door", detect_door);
  ROS_INFO("Ready to detect open door.");
  ros::Subscriber sub = n.subscribe("/scan",1000,laserCallback);
  ros::spin();

  return 0;
}

//Service
bool detect_door(open_door_detector::detect_open_door::Request  &req,
         open_door_detector::detect_open_door::Response &res)
{
  if(req.aperture_angle <= 0) req.aperture_angle = 3;
  
  int range_points = last_scan_in.ranges.size();
  
  int range_start = range_points/2 - req.aperture_angle/(2*last_scan_in.angle_increment);
  if(range_start < 0) range_start = 0;
  
  int range_end = range_points/2 + req.aperture_angle/(2*last_scan_in.angle_increment);
  if(range_end > range_points) range_end = range_points-1;
  
  int door_start;
  int door_end;
  int n_short = 0;
  int n_long = 0;
  bool isDoor = false;
  float laser_dist;
  
  // detect door
  for(int i=range_start; i<=range_end; i++){
    laser_dist = req.wall_distance/cos(last_scan_in.angle_min + i*last_scan_in.angle_increment);
    if(isDoor) {
      if(last_scan_in.ranges[i] <= laser_dist) {
	n_short++;
      } else {
	n_short = 0;
      }
      if(n_short > DETECTION_THRESHOLD) {
	door_end = i - DETECTION_THRESHOLD + 1;
	break;
      }
    } else {
      if(last_scan_in.ranges[i] > laser_dist || last_scan_in.ranges[i] != last_scan_in.ranges[i]) {
	n_long++;
      } else {
	n_long = 0;
      }
      if(n_long > DETECTION_THRESHOLD) {
	isDoor = true;
	door_start = i - DETECTION_THRESHOLD + 1;
	door_end = range_end;
      }
    }
  }
  
  
  // provide the response
  if(isDoor) {
    res.door_pos.pose.position.x = req.wall_distance;
    float mid_door = (door_start + door_end)/2;
    float angle_mid_door = last_scan_in.angle_min + mid_door*last_scan_in.angle_increment;
    res.door_pos.pose.position.y=tan(angle_mid_door)*req.wall_distance; 
  } else {
    res.door_pos.pose.position.x=0;
    res.door_pos.pose.position.y=0;
  }
  res.door_pos.pose.position.z=0;
  res.door_pos.pose.orientation.w=0;
  res.door_pos.pose.orientation.x=0;
  res.door_pos.pose.orientation.y=0;
  res.door_pos.pose.orientation.z=1;
  
  res.door_pos.header.frame_id = "/camera_link";
  res.door_pos.header.stamp.now();
  
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  last_scan_in = *scan_in;
}