#include "ros/ros.h"
#include "video_player/PlayVideoSrv.h"
#include <cstdlib>

int main(int argc, char **argv){
	
	ros::init(argc, argv, "NbrOfVideoToPlay");
	if (argc != 2){
		ROS_INFO("usage: which video to play X");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<video_player::PlayVideoSrv>("/wheeled_robin/application/play_video");
	video_player::PlayVideoSrv srv;
	
	srv.request.videoPath = argv[1];
	if (client.call(srv)){
		ROS_INFO("done playing");
	} else {
		ROS_ERROR("Video not found");
		return 1;
	}

	return 0;
}
