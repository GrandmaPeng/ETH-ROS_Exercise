#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include "std_srvs/SetBool.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "start_stop_client");
	if (argc != 2)
	{
		ROS_INFO("argument: true or false");
		return 1;
	}

	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<std_srvs::SetBool>("start_stop");
	std_srvs::SetBool srv;
	if (argv[1])
	{
		srv.request.data = true;
	}
	else
	{
		srv.request.data = false;
	}

	if (client.call(srv))
	{
		ROS_INFO("Called to : %s", argv[1]);
	}
	else
	{
		ROS_ERROR("Failed to call service start_stop");
		return 1;
	}

	return 0;
}
