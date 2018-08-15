#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include "std_srvs/SetBool.h"
#include "ros/ros.h"

ros::Publisher move_status_pub;

bool srv_callback_(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
        std_msgs::Bool sta_;
	if (request.data)
	{
		sta_.data = true;
		response.message = "Command True sent!";
	}
	else
	{
		sta_.data = false;
		response.message = "Command False sent!";
	}	
	response.success = true;
	move_status_pub.publish(sta_);
	ROS_INFO("PUBLISHED!");
	return true;		
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "start_stop_server");
	ros::NodeHandle nodeHandle;
	ros::ServiceServer service = nodeHandle.advertiseService("start_stop", srv_callback_);
	move_status_pub = nodeHandle.advertise<std_msgs::Bool>("/move_status", 1);
	ros::spin();
	return 0;
}
