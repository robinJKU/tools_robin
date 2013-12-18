/**
 * Title:	People Detection
 * Author: 	Reiter Andreas & Reisenberger Johannes
 * Date:	17.12.2013
 * Summary:	Calculates the most dominant person (weighted function from x- and y-distance) 
 * 		and publishes a tf from the projection to the floor (/dominant_person).
 * 		Furthermore publishes a tf of the average position of all detected persons, projected to the floor (/group).
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <limits>

tf::StampedTransform zTransform(const tf::StampedTransform transform);
float getCosts(const tf::StampedTransform transform);
int getMinIndex (int n, float *values);

//ROS Params
std::string base_name; 

// global Constants
const int MAX_PERSONS = 20;
const std::string DOM_PERSON_NAME = "/dominant_person";
const std::string GROUP_NAME = "/group";




int main( int argc, char** argv )
{
  ros::init(argc, argv, "robin_people_detection");
  // Set Markernode and publish rate 10Hz
  ros::NodeHandle node("~");
  ros::Rate r(10);
  
  tf::TransformListener li;
  tf::TransformBroadcaster br;
  std::string torso_name;
  std::string err_msg;
  std::string parent;
  
  node.param<std::string>("base_name", base_name,"/base_footprint");
  
  float costs[MAX_PERSONS] = {std::numeric_limits<float>::max()};
   
  float sum_x;
  float sum_y;
  int nTransforms;
   
  int i;
   
  // Initialize Transformation
  tf::StampedTransform transforms[MAX_PERSONS];
  tf::StampedTransform transformZ0;
    
  ros::Time now;
 
  
  
  while (ros::ok()) {
    now = ros::Time::now();
    sum_x = 0;
    sum_y = 0;
    nTransforms = 0;
       
    for(i=1; i <= MAX_PERSONS; i++){
      // get name of next torso-tf
      std::stringstream ss;
      ss << i;
      torso_name="torso_"+ss.str();
      
      costs[i-1] = std::numeric_limits<float>::max();
      
      if(li.frameExists(torso_name)&&li.getParent(torso_name, now-ros::Duration(1), parent)) {
	try{
	  //get transformation
	  li.waitForTransform(base_name ,torso_name, now, ros::Duration(1) );
	  li.lookupTransform(base_name ,torso_name , now, transforms[i-1]);
 	  costs[i-1] = getCosts(transforms[i-1]);
	  nTransforms++;
	  sum_x += transforms[i-1].getOrigin().getX();
	  sum_y += transforms[i-1].getOrigin().getY();	        
	      
	}//try{...
	catch (tf::TransformException ex) {
	  ROS_ERROR("%s",ex.what());
	}//catch (tf::TransformException ex)
      }// if(li.frameExists(torso_name)&&li.getParent(to....
    }//for(i=1; i <= MAX_PERSONS; i++)
       
    // publish dominant person
    i = getMinIndex(MAX_PERSONS,costs);
    if(i >= 0) {
      std::stringstream ss;
      ss << (i+1);
      torso_name="torso_"+ss.str();
	
      transformZ0 = zTransform(transforms[i]);
      br.sendTransform(tf::StampedTransform(transformZ0, ros::Time::now(), base_name, DOM_PERSON_NAME)); 
	
    }//if(i >= 0)
       
    // publish average position (group)
    if(nTransforms > 0) {
      tf::StampedTransform groupTransf;
      groupTransf.setOrigin(tf::Vector3(sum_x/nTransforms,sum_y/nTransforms,0));
      groupTransf.setRotation(tf::Quaternion(tf::Vector3(0,0,1),0.0));
      br.sendTransform(tf::StampedTransform(groupTransf, ros::Time::now(), base_name, GROUP_NAME));
    }//if(nTransforms > 0)
       
    // Call Callbacks for good measure 
    ros::spinOnce();
    
    r.sleep();
  }// while (ros::ok())
}// main

tf::StampedTransform zTransform(const tf::StampedTransform transform) {
  /**
   * Sets the z-value of transform to 0 and returns the new transform.
   */
  tf::StampedTransform newTransform;
  newTransform.setOrigin(tf::Vector3(transform.getOrigin().getX(),transform.getOrigin().getY(),0));
  newTransform.setRotation(transform.getRotation());
  return newTransform;
}

float getCosts(const tf::StampedTransform transform) {
  /**
   * Calculates costs for the given transform (x+0.4*y)
   */
  return transform.getOrigin().getX()+transform.getOrigin().getY()*0.4;
}

int getMinIndex (int n, float *values) {
  /**
   * Returns the index of the minimal value in the array with the length n.
   */
  float min = values[0];
  int ind = 0;
  for(int i=1; i < n; i++) {
    if(values[i] < min) {
      min = values[i];
      ind = i;  
    } 
  }
  
  if(min >= std::numeric_limits<float>::max())
  {
    ind = -1;
  }
  return ind;
}





