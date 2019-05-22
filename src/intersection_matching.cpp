//intersection_matchig
//author : Yoshitaka Nagai

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <iostream>

#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"


class IntersectionMatching
{
public:
	IntersectionMatching();

	void peak_deg_callback(const std_msgs::Int32MultiArray::Ptr&);
	void estimated_pose_callback(const nav_msgs::OdometryConstPtr&);
	void edge_callback(const amsl_navigation_msgs::EdgeConstPtr&);
	void node_edge_map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);
	
	void intersection_matching_manager(void);
	
	//bool intersection_detect_mode_manager(void);
	void intersection_detect_mode_manager(void);
	bool intersection_recognizer(void);
	
private:
	ros::NodeHandle nh, n;
	
	ros::Subscriber peak_deg_sub;
	ros::Subscriber edge_sub;
	ros::Subscriber estimated_pose_sub;
	
	ros::Publisher intersection_flag_pub;

	nav_msgs::Odometry estimated_pose;
	
	std_msgs::Int32MultiArray peak_deg;
	std_msgs::Bool intersection_flag;

	amsl_navigation_msgs::Edge edge;
	amsl_navigation_msgs::NodeEdgeMap node_edge_map;

	double EDGE_PROGRESS_THRESHOLD_MAX;
	double EDGE_PROGRESS_THRESHOLD_MIN;
	double DISTANCE2NODE_THRESHOLD;
	
	float x_node;
	float y_node;
	float x_pose;
	float y_pose;
	float distance2node;

	int DETECT_MODE;
	int next_node_id;

	bool peak_deg_callback_flag = false;
	bool estimated_pose_callback_flag = false;
	bool edge_callback_flag = false;
	bool node_edge_map_callback_flag = false;
	bool intersection_detect_mode_flag = false;
	bool intersection_recognize_flag = false;
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "intersection_matching");

	
	IntersectionMatching ism;
	ism.intersection_matching_manager();

	return 0;
}



IntersectionMatching::IntersectionMatching(void)
{
	//intersection_flag.data = false;
	
	n.getParam("EDGE_PROGRESS_THRESHOLD_MAX",EDGE_PROGRESS_THRESHOLD_MAX);
	n.getParam("EDGE_PROGRESS_THRESHOLD_MIN",EDGE_PROGRESS_THRESHOLD_MIN);
	n.getParam("EDGE_PROGRESS_THRESHOLD_MIN",DISTANCE2NODE_THRESHOLD);
	n.getParam("DETECT_MODE",DETECT_MODE);
	
	peak_deg_sub = nh.subscribe("/peak/deg", 1, &IntersectionMatching::peak_deg_callback, this);
	edge_sub = nh.subscribe("/node_edge_map/map", 1, &IntersectionMatching::edge_callback, this);
	estimated_pose_sub = nh.subscribe("/estimated_pose/pose", 1, &IntersectionMatching::estimated_pose_callback, this);

	intersection_flag_pub = nh.advertise<std_msgs::Bool>("/intersection_flag", 1);
}


void IntersectionMatching::intersection_matching_manager(void)
{
	//IntersectionMatching ism;

	ros::Rate r(100);
	while(ros::ok()){
		//if(peak_deg_callback_flag && estimated_pose_callback_flag && edge_callback_flag){
			std::cout << "flags : true" << std::endl;
			intersection_flag.data = intersection_recognizer();
			intersection_flag_pub.publish(intersection_flag);
		//}
		r.sleep();
		ros::spinOnce();
	}
}


void IntersectionMatching::peak_deg_callback(const std_msgs::Int32MultiArray::Ptr &msg)
{
	peak_deg = *msg;
	peak_deg_callback_flag = true;
	//std::cout << "peak_deg_callback_flag = true" << std::endl;
}


void IntersectionMatching::estimated_pose_callback(const nav_msgs::OdometryConstPtr &msg)
{
	estimated_pose = *msg;
	estimated_pose_callback_flag = true;
	//std::cout << "estimated_pose_callback_flag = true" << std::endl;
}


void IntersectionMatching::edge_callback(const amsl_navigation_msgs::EdgeConstPtr &msg)
{
	edge = *msg;
	
	
	std::cout << "edge_callback_flag = false" << std::endl;

	next_node_id = edge.node1_id;
	edge_callback_flag = true;
	
	std::cout << "edge_callback_flag = true" << std::endl;
	std::cout << "next_node_id : " << next_node_id << std::endl;
	//std::cout << "edge_callback_flag = true" << std::endl;
}

void IntersectionMatching::node_edge_map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr &msg)
{
	node_edge_map = *msg;
	node_edge_map_callback_flag = true;
	//std::cout << "node_edge_map_callback_flag = true" << std::endl;
}


//bool IntersectionMatching::intersection_detect_mode_manager(void)
void IntersectionMatching::intersection_detect_mode_manager(void)
{
	//std::cout << "detect_mode switching" << std::endl;

	switch(DETECT_MODE){
		case 1:
			if(edge.progress > (float)EDGE_PROGRESS_THRESHOLD_MIN && edge.progress < (float)EDGE_PROGRESS_THRESHOLD_MAX){ 
				std::cout << "detect_mode" << std::endl;
				intersection_detect_mode_flag = true;
			}else{
				intersection_detect_mode_flag = false;
				std::cout << "not detect_mode" << std::endl;
			}
			break;	
		
		case 2:
			x_pose = estimated_pose.pose.pose.position.x;
			y_pose = estimated_pose.pose.pose.position.y;

			x_node = node_edge_map.nodes[next_node_id].point.x;
			y_node = node_edge_map.nodes[next_node_id].point.y;
			
			distance2node = sqrt(pow(x_node - x_pose, 2) + pow(y_node - y_pose, 2));
			
			if(distance2node > (float)DISTANCE2NODE_THRESHOLD){
				std::cout << "detect_mode" << std::endl;
				intersection_detect_mode_flag = true;
			}else{
				intersection_detect_mode_flag = false;
				std::cout << "not detect_mode" << std::endl;
			}
			break;
	}
	//return intersection_detect_mode_flag;
}


bool IntersectionMatching::intersection_recognizer(void)
{
	intersection_detect_mode_manager();

	if(intersection_detect_mode_flag){
		if(peak_deg.data.size() > 2){
			std::cout << "intersection!!!!!!!!!!!!!!" << std::endl;
			intersection_recognize_flag = true;	
		}
	}else{
		intersection_recognize_flag = false;
		//std::cout << "not intersection :(" << std::endl;
	}

	return intersection_recognize_flag;	
}
