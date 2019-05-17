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

/*
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

typedef pcl::PointXYZINormal PointINormal;
typedef pcl::PointCloud<PointINormal> CloudINormal;
typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;
*/

/*
double get_yaw(geometry_msgs::Quaternion q)
{
	double r, p, y;
	tf::Quaternion quat(q.x, q.y, q.z, q.w);
	tf::Matrix3x3(quat).getRPY(r, p, y);
	return y;
}
*/


class IntersectionMatching
{
public:
	IntersectionMatching();

	void peak_deg_callback(const std_msgs::Int32MultiArray::Ptr&);
	void estimated_pose_callback(const nav_msgs::OdometryConstPtr&);
	void edge_callback(const amsl_navigation_msgs::EdgeConstPtr&);
	
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

	float edge_progress_threshold_max;
	float edge_progress_threshold_min;

	bool peak_deg_callback_flag;
	bool estimated_pose_callback_flag;
	bool edge_callback_flag;
	bool intersection_detect_mode_flag;
	bool intersection_recognize_flag;
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
	peak_deg_callback_flag = false;
	estimated_pose_callback_flag = false;
	edge_callback_flag = false;
	intersection_detect_mode_flag = false;
	intersection_recognize_flag = false;
	intersection_flag.data = false;

	n.getParam("edge_progress_threshold_max",edge_progress_threshold_max);
	n.getParam("edge_progress_threshold_min",edge_progress_threshold_min);
	
	peak_deg_sub = nh.subscribe("/peak/deg", 1, &IntersectionMatching::peak_deg_callback, this);
	edge_sub = nh.subscribe("/edge/certain", 1, &IntersectionMatching::edge_callback, this);
	estimated_pose_sub = nh.subscribe("/estimated_pose/pose", 1, &IntersectionMatching::estimated_pose_callback, this);

	intersection_flag_pub = nh.advertise<std_msgs::Bool>("/intersection_flag", 1);
}


void IntersectionMatching::intersection_matching_manager(void)
{
	//IntersectionMatching ism;

	ros::Rate r(100);
	while(ros::ok()){
		//if(peak_deg_callback_flag && estimated_pose_callback_flag && edge_callback_flag){
			//std::cout << "flags : true" << std::endl;
			//intersection_flag.data = ism.intersection_recognizer();
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
	edge_callback_flag = true;
	//std::cout << "edge_callback_flag = true" << std::endl;
}


//bool IntersectionMatching::intersection_detect_mode_manager(void)
void IntersectionMatching::intersection_detect_mode_manager(void)
{
	if(edge.progress > edge_progress_threshold_min && edge.progress < edge_progress_threshold_max){ 
		std::cout << "detect_mode" << std::endl;
		intersection_detect_mode_flag = true;
	}else{
		intersection_detect_mode_flag = false;
		std::cout << "not detect_mode" << std::endl;
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
