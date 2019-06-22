#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>


std_msgs::Int32MultiArray set_deg;
std_msgs::Float32MultiArray set_rad;

bool array_callback_flag;

float deg2pipi(int angle_num)
{
	float deg_angle = 0.5 * (float)angle_num;
	float pipi_angle = 0.0;
	if(deg_angle >= 0.0 && deg_angle < 180.0){
		pipi_angle = M_PI * deg_angle / 180.0;
	}else{
		pipi_angle = -M_PI * (180.0 - (deg_angle - 180.0)) / 180.0;
	}
	
	return pipi_angle;
}


void array_callback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
	std::cout << "subscribe" << std::endl;
	set_deg = *msg;
	set_rad.data.clear();
	size_t set_deg_size = set_deg.data.size();
	for(size_t i=0;i<set_deg_size;i++){
		int angle_id = set_deg.data[i];
		std::cout << "push_back data" << std::endl;
		set_rad.data.push_back(deg2pipi(angle_id));
	}
	array_callback_flag = true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "peak_deg2rad");
	ros::NodeHandle n;
	
	ros::Subscriber sub_array = n.subscribe("/intersection_recognition/peak_deg", 10, array_callback);
	ros::Publisher pub_array = n.advertise<std_msgs::Float32MultiArray>("/intersection_recognition/peak_rad", 10);

	array_callback_flag = false;

	ros::Rate r(10);
	while(ros::ok()){
		if(array_callback_flag){
			pub_array.publish(set_rad);
			std::cout << "publish /peak_rad" << std::endl;
			array_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}
