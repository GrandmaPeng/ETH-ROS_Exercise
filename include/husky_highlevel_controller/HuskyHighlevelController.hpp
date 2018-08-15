#pragma once

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

private:
	void scanCallback(const sensor_msgs::LaserScan& msg);
	void srvCallback(const std_msgs::Bool trigger);
	void markerType();
	ros::NodeHandle nodeHandle_;
	ros::Subscriber scanSubscriber_;
	std::string scanTopic_;
	std::int32_t subscribeQueueSize_;
	float x_pillar;
	float y_pillar;
	float alpha_pillar;
	ros::Publisher cmd_pub_;
	ros::Publisher vis_pub_;
	geometry_msgs::Twist vel_msg_;
	bool status_;
	ros::Subscriber startStopSubscriber_;
	ros::ServiceClient client;
	std_srvs::SetBool srv;
};

} /* namespace */
