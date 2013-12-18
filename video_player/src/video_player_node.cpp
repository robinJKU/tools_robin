#include "ros/ros.h"
#include "video_player/PlayVideoSrv.h"
#include "stdlib.h"
#include <gscam/gscam.h>

int METHOD;

// Service Funktion
bool playVideo(video_player::PlayVideoSrv::Request  &req, video_player::PlayVideoSrv::Response &res){

	/**
	 The delivery parameter must be: "/home/student/Downloads/Video1"
	 all video files must be named as:"video.avi"
	 all sh files must be named as: openvideo.sh"
	*/
	
	if(METHOD == 1){
		// play the video in vlc player		
		std::string gsconfig_param = "";
		gsconfig_param = req.videoPath + "/openVideo.sh";
		system(gsconfig_param.c_str());
		
	} else if(METHOD == 2) {
		ros::NodeHandle n, nh_private("~");
		gscam::GSCam gscam_driver(n, nh_private);
		
		ROS_INFO("Setting Param\n");
		
		// assamble string for filePath
		std::string gsconfig_param = "";
		gsconfig_param = "filesrc location=" + req.videoPath + "/Video.avi ! avidemux name=demux demux.video_00 ! decodebin  ! ffmpegcolorspace  ! video/x-raw-rgb ! identity name=ros ! fakesin";	
		
		// video streamen
		nh_private.setParam("gscam_config", gsconfig_param);
		if(!gscam_driver.configure()) {
			ROS_FATAL("Failed to configure gscam!");
		} else if(!gscam_driver.init_stream()) {
			ROS_FATAL("Failed to initialize gscam stream!");
		} else {
			gscam_driver.publish_stream();
		}
		
		// clean all up
		ROS_INFO("Cleaning up stream and exiting...");
		gscam_driver.cleanup_stream();
		ROS_INFO("GStreamer stream stopped!");
		
	} else {
		ROS_INFO("METHOD variable false");
	}
	
	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "play_video_server");

	ros::NodeHandle n, nh_private("~");	
	
	/**
	  mode_I: the video is played at vlc player
	  mode_II: video is streamed
	*/
	std::string mode_;
	nh_private.param<std::string>("mode", mode_, "mode_II");
	if(mode_.compare("mode_I") == 0){
		METHOD = 1; // = open VLC Player
	} else {
		METHOD = 2; // = stream
	}
	
	ros::ServiceServer service = n.advertiseService("/wheeled_robin/application/play_video", playVideo);

	ROS_INFO("Ready to play video");
	
	ros::spin();
	
	return 0;
}
