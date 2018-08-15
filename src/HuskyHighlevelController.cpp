#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
		nodeHandle_(nodeHandle) {
	if (!(nodeHandle.getParam("scan_topic_name", scanTopic_)
			& nodeHandle.getParam("scan_queue_size", subscribeQueueSize_))) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscribeQueueSize_,
			&HuskyHighlevelController::scanCallback, this);
	cmd_pub_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	vis_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>(
			"visualization_marker", 0);
	startStopSubscriber_ = nodeHandle_.subscribe("/move_status",
			subscribeQueueSize_, &HuskyHighlevelController::srvCallback, this);

	status_ = true;
}

HuskyHighlevelController::~HuskyHighlevelController() {
}

void HuskyHighlevelController::scanCallback(const sensor_msgs::LaserScan& msg) {
	std::vector<float> range = msg.ranges;
	/*
	 float small = ranges[0];
	 for (float v:ranges)
	 {
	 if(small > v)
	 {
	 small = v;
	 }
	 }
	 ROS_INFO_STREAM("Min range: " + std::to_string(small));
	 */
	float small = range[0];
	int arr_size = floor((msg.angle_max - msg.angle_min) / msg.angle_increment);
	for (int i = 0; i < arr_size; i++) {
		if (range[i] < small) {
			small = range[i];
			alpha_pillar = (msg.angle_min + i * msg.angle_increment);
		}
	}

	x_pillar = small * cos(alpha_pillar);
	y_pillar = small * sin(alpha_pillar);
	ros::NodeHandle nh;
	client = nh.serviceClient<std_srvs::SetBool>("start_stop");
	if (x_pillar < 0.3 && y_pillar < 0.3) {
		srv.request.data = false;
	} else {

		srv.request.data = true;

	}
	client.call(srv);
	float p_gain_vel = 0.5;
	float p_gain_ang = 0.1;

	if (x_pillar > 0.1 && status_) {
		vel_msg_.linear.x = x_pillar * p_gain_vel;
		vel_msg_.angular.z = -y_pillar * p_gain_ang;
	} else {
		vel_msg_.linear.x = 0;
		vel_msg_.angular.z = 0;
	}

	cmd_pub_.publish(vel_msg_);
	markerType();

	ROS_INFO("Pillar offset angle(rad): %lf", alpha_pillar);
	ROS_INFO("Pillar x distance(m): %lf", x_pillar);
	ROS_INFO("Pillar y distance(m): %lf", y_pillar);
}

void HuskyHighlevelController::srvCallback(const std_msgs::Bool trigger) {
	if (trigger.data)
		status_ = true;
	else
		status_ = false;

}

void HuskyHighlevelController::markerType() {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_laser";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x_pillar;
	marker.pose.position.y = y_pillar;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	vis_pub_.publish(marker);

}

} /* namespace */
