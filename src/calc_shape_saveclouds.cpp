//入ってきたdataに対して外縁を計算する
//savecloud では点群を貯めた状態において外縁の計算をする
//
//

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
//PCL

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
#include <omp.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool callback_flag = false;
bool odom_callback = false;

// int divide = 360;
int divide = 720;
/* int divide = 1080; */
const double PI = 3.141592;
double remove_distance = 0.1;
/* double remove_distance = 1.0; */

ros::Publisher shape_pub;
ros::Time current_time;

//publish pointcloud
inline void pubPointCloud2(ros::Publisher& pub, 
                            const CloudA& cloud,
                            const char* frame_id,
                            ros::Time& time)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id ="velodyne";
    // output.header.frame_id ="map";
    output.header.stamp = time;
    pub.publish (output);
}

//copy point
inline void Copy_point(PointA& input,PointA& output)
{
	output.x = input.x;
	output.y = input.y;
	// output.z = input.z;
	output.z = 0.00;
	output.normal_x = input.normal_x;
	output.normal_y = input.normal_y;
	output.normal_z = input.normal_z;
	// output.intensity = input.intensity;
	output.intensity = 1.0;
	output.curvature = input.curvature;
	/* cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"<<endl; */
}

//calculation distance && compare distance
inline bool Calc_distance(PointA& orig,PointA& min)
{
	double orig_d = orig.x*orig.x + orig.y*orig.y;
	double min_d  = min.x*min.x + min.y*min.y;

	/* cout<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"<<endl; */
	//check distance
	if(orig_d<remove_distance){
		orig_d = 100.0;
	}
	//end check
	if(orig_d<min_d){
		return true;
	}else{
		return false;
	}
}

inline double calc_d(PointA& input)
{
	double distance = sqrt(input.x*input.x + input.y*input.y);

	/* cout<<"cccccccccccccccccccccccccccccccccccccccccccccccccc"<<endl; */
	// cout<<distance<<endl;
	return distance;
}
//五点平均verCloudAPtr

// void Fifth_mean(CloudAPtr cloud)
// {
// 	double distance = 0.0;
// 	size_t point_size = cloud->points.size();
// 	CloudAPtr tmp_cloud(new CloudA);
//
// 	for(size_t i=0;i<point_size;i++){
// 		PointA mean;
// 		if(i<2){
// 			if(i==0){
// 				mean.x = (cloud->points[point_size-2].x + cloud->points[point_size-1].x + cloud->points[i].x + cloud->points[(i+1)].x + cloud->points[(i+2)].x)/5;
// 				mean.y = (cloud->points[point_size-2].y + cloud->points[point_size-1].y + cloud->points[i].y + cloud->points[(i+1)].y + cloud->points[(i+2)].y)/5;
// 				mean.z = (cloud->points[point_size-2].z + cloud->points[point_size-1].z + cloud->points[i].z + cloud->points[(i+1)].z + cloud->points[(i+2)].z)/5;
//
// 				tmp_cloud->push_back(mean);
// 				cout<<i<<","<<calc_d(tmp_cloud->points[i])<<endl;
// 			}else{
// 				mean.x = (cloud->points[point_size-1].x + cloud->points[(i-1)].x + cloud->points[i].x + cloud->points[(i+1)].x + cloud->points[(i+2)].x)/5;
// 				mean.y = (cloud->points[point_size-1].y + cloud->points[(i-1)].y + cloud->points[i].y + cloud->points[(i+1)].y + cloud->points[(i+2)].y)/5;
// 				mean.z = (cloud->points[point_size-1].z + cloud->points[(i-1)].z + cloud->points[i].z + cloud->points[(i+1)].z + cloud->points[(i+2)].z)/5;
// 				tmp_cloud->push_back(mean);
// 				cout<<i<<","<<calc_d(tmp_cloud->points[i])<<endl;
// 			}
// 		}else{
// 			mean.x = (cloud->points[(i-2)].x + cloud->points[(i-1)].x + cloud->points[i].x + cloud->points[(i+1)].x + cloud->points[(i+2)].x)/5;
// 			mean.y = (cloud->points[(i-2)].y + cloud->points[(i-1)].y + cloud->points[i].y + cloud->points[(i+1)].y + cloud->points[(i+2)].y)/5;
// 			mean.z = (cloud->points[(i-2)].z + cloud->points[(i-1)].z + cloud->points[i].z + cloud->points[(i+1)].z + cloud->points[(i+2)].z)/5;
// 			tmp_cloud->push_back(mean);
// 			cout<<i<<","<<calc_d(tmp_cloud->points[i])<<endl;
// 		}
// 	}
// }
//
//五点平均verCloudA
void Fifth_mean(CloudA& cloud)
{
	// double distance = 0.0;
	size_t point_size = cloud.points.size();
	CloudAPtr tmp_cloud(new CloudA);

	for(size_t i=0;i<point_size;i++){
		PointA mean;
		/* if(i<2){ */
		/* 	if(i==0){ */
		/* 		mean.x = (cloud.points[point_size-2].x + cloud.points[point_size-1].x + cloud.points[i].x + cloud.points[(i+1)].x + cloud.points[(i+2)].x)/5; */
		/* 		mean.y = (cloud.points[point_size-2].y + cloud.points[point_size-1].y + cloud.points[i].y + cloud.points[(i+1)].y + cloud.points[(i+2)].y)/5; */
		/* 		mean.z = (cloud.points[point_size-2].z + cloud.points[point_size-1].z + cloud.points[i].z + cloud.points[(i+1)].z + cloud.points[(i+2)].z)/5; */
        /*  */
		/* 		tmp_cloud->push_back(mean); */
		/* 		// cout<<i<<","<<calc_d(tmp_cloud->points[i])<<endl; */
		/* 	}else{ */
		/* 		mean.x = (cloud.points[point_size-1].x + cloud.points[(i-1)].x + cloud.points[i].x + cloud.points[(i+1)].x + cloud.points[(i+2)].x)/5; */
		/* 		mean.y = (cloud.points[point_size-1].y + cloud.points[(i-1)].y + cloud.points[i].y + cloud.points[(i+1)].y + cloud.points[(i+2)].y)/5; */
		/* 		mean.z = (cloud.points[point_size-1].z + cloud.points[(i-1)].z + cloud.points[i].z + cloud.points[(i+1)].z + cloud.points[(i+2)].z)/5; */
		/* 		tmp_cloud->push_back(mean); */
		/* 		// cout<<i<<","<<calc_d(tmp_cloud->points[i])<<endl; */
		/* 	} */
		/* }else{ */
		/* 	mean.x = (cloud.points[(i-2)].x + cloud.points[(i-1)].x + cloud.points[i].x + cloud.points[(i+1)%point_size].x + cloud.points[(i+2)%point_size].x)/5; */
		/* 	mean.y = (cloud.points[(i-2)].y + cloud.points[(i-1)].y + cloud.points[i].y + cloud.points[(i+1)%point_size].y + cloud.points[(i+2)%point_size].y)/5; */
		/* 	mean.z = (cloud.points[(i-2)].z + cloud.points[(i-1)].z + cloud.points[i].z + cloud.points[(i+1)%point_size].z + cloud.points[(i+2)%point_size].z)/5; */
		/* 	tmp_cloud->push_back(mean); */
		/* 	// cout<<i<<","<<calc_d(tmp_cloud->points[i])<<endl; */
		/* } */

		mean.x = (cloud.points[i%point_size].x + cloud.points[(i+1)%point_size].x + cloud.points[(i+2)%point_size].x + cloud.points[(i+3)%point_size].x + cloud.points[(i+4)%point_size].x)/5;
		mean.y = (cloud.points[i%point_size].y + cloud.points[(i+1)%point_size].y + cloud.points[(i+2)%point_size].y + cloud.points[(i+3)%point_size].y + cloud.points[(i+4)%point_size].y)/5;
		mean.z = (cloud.points[i%point_size].z + cloud.points[(i+1)%point_size].z + cloud.points[(i+2)%point_size].z + cloud.points[(i+3)%point_size].z + cloud.points[(i+4)%point_size].z)/5;
		tmp_cloud->push_back(mean);

		/* cout<<"dddddddddddddddddddddddddddddddddddddddddddddddddd"<<endl; */
		//publish_fifmean_data
		Copy_point(tmp_cloud->points[i],cloud.points[(i+2)%point_size]);
	}
}

//load_shape detection
void Calc_shape(CloudAPtr input_cloud,double& abs_x,double& abs_y,double& abs_yaw)
{
	size_t cloud_size = input_cloud->points.size();
	// cout<<cloud_size<<endl;
	int deg = 0;
	CloudAPtr shape_cloud(new CloudA);
	shape_cloud->points.resize(divide);
	double tmp_x=0,tmp_y=0;
	double sin_abs = sin(abs_yaw);
	double cos_abs = cos(abs_yaw);
	// #pragma omp parallel for
	for(size_t i=0;i<cloud_size;i++){
		// input_cloud->points[i].intensity = atan(input_cloud->points[i].y/input_cloud->points[i].x)/PI*180;
		// input_cloud->points[i].intensity = atan2(input_cloud->points[i].y,input_cloud->points[i].x)/PI*180.0;
		tmp_x = input_cloud->points[i].x - abs_x;
		tmp_y = input_cloud->points[i].y - abs_y;
		// input_cloud->points[i].intensity = atan2(input_cloud->points[i].y - abs_y ,input_cloud->points[i].x - abs_x )/PI*360.0;
		// input_cloud->points[i].intensity = atan2(- tmp_x*sin(abs_yaw) + tmp_y*cos(abs_yaw)  ,tmp_x*cos(abs_yaw) + tmp_y*sin(abs_yaw) )/PI*360.0;
		input_cloud->points[i].intensity = atan2(- tmp_x*sin_abs + tmp_y*cos_abs  ,tmp_x*cos_abs + tmp_y*sin_abs )/PI*360.0;
	}
	// #pragma omp parallel for
	for(size_t i=0;i<cloud_size;i++){
		deg = (int)input_cloud->points[i].intensity + 359;
		tmp_x = input_cloud->points[i].x - abs_x;
		tmp_y = input_cloud->points[i].y - abs_y;
		// input_cloud->points[i].x = tmp_x*cos(abs_yaw) + tmp_y*sin(abs_yaw);
		// input_cloud->points[i].y = - tmp_x*sin(abs_yaw) + tmp_y*cos(abs_yaw);
		input_cloud->points[i].x = tmp_x*cos_abs + tmp_y*sin_abs;
		input_cloud->points[i].y = - tmp_x*sin_abs + tmp_y*cos_abs;
		
		// cout<<deg<<endl;//for debag
		if(shape_cloud->points[deg].intensity!=1.0){
			Copy_point(input_cloud->points[i],shape_cloud->points[deg]);
		}else{
			if(Calc_distance(input_cloud->points[i],shape_cloud->points[deg]))
			{
				Copy_point(input_cloud->points[i],shape_cloud->points[deg]);
			}
		}
	}
	// #pragma omp parallel for
	for(int i=0;i<divide;i++){
		if(i!=0&&calc_d(shape_cloud->points[i])==0.0){
			// cout<<"ZERO!!!!"<<i<<endl;
			// if(calc_d(shape_cloud->points[i-1])>20.0){
			if(calc_d(shape_cloud->points[i-1])>13.0){
				shape_cloud->points[i].x = 40*sin((double)i/2.0/180.0*PI-PI/2.0);
				shape_cloud->points[i].y = -40*cos((double)i/2.0/180.0*PI-PI/2.0);
			}else{
				Copy_point(shape_cloud->points[i-1],shape_cloud->points[i]);
			}
		}
	}
	// cout<<"fin!!!!"<<endl;
	//output
	CloudA output_cloud;
	cloud_size = shape_cloud->points.size();
	/* cout<<"CLOUD_SIZE="<<cloud_size<<endl;//for debag */
	// output_cloud.points.resize(cloud_size);
	/* cout<<"111111111111111111111111111111111111111111111111"<<endl; */
	for(size_t i=0;i<cloud_size;i++){
	/* for(size_t i=0;i<divide;i++){ */
		/* if(calc_d(shape_cloud->points[i])<remove_distance && i!=0){ */
		PointA tmp;
		/* cout<<"staaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaart"<<endl; */
		if(calc_d(shape_cloud->points[i])<remove_distance){
			if(i==0){
				/* cout<<"2222222222222222222222222222222222222222222222222"<<endl; */
				tmp.x = 5.0;
				tmp.y = 5.0;
				tmp.z = 0.0;
				tmp.normal_x = 0.0;
				tmp.normal_y = 0.0;
				tmp.normal_z = 0.0;
				// output.intensity = input.intensity;
				tmp.intensity = 1.0;
				tmp.curvature = 0.0;
				/* cout<<"remooooooooooooooooooove!!!!!"<<endl; */
			}
			else{
				/* cout<<"333333333333333333333333333333333333333333333333"<<endl; */
				tmp.x = output_cloud.points[i-1].x;
				tmp.y = output_cloud.points[i-1].y;
				tmp.z = output_cloud.points[i-1].z;
				/* tmp.normal_x = output_cloud.points[i-1].normal_x; */
				/* tmp.normal_y = output_cloud.points[i-1].normal_y; */
				/* tmp.normal_z = output_cloud.points[i-1].normal_z; */
				/* tmp.intensity = output_cloud.points[i-1].intensity; */
				/* tmp.curvature = output_cloud.points[i-1].curvature; */
				tmp.normal_x = 0.0;
				tmp.normal_y = 0.0;
				tmp.normal_z = 0.0;
				tmp.intensity = 1.0;
				tmp.curvature = 0.0;
				/* Copy_point(output_cloud.points[i-1],output_cloud.points[i]); */
			}
		}
		else{
			/* cout<<"$4444444444444444444444444444444444444444444444"<<endl; */
			tmp.x = shape_cloud->points[i].x;
			tmp.y = shape_cloud->points[i].y;
			tmp.z = shape_cloud->points[i].z;
			/* tmp.normal_x = shape_cloud->points[i].normal_x; */
			/* tmp.normal_y = shape_cloud->points[i].normal_y; */
			/* tmp.normal_z = shape_cloud->points[i].normal_z; */
			/* tmp.intensity = shape_cloud->points[i].intensity; */
			/* tmp.curvature = shape_cloud->points[i].curvature; */
			tmp.normal_x = 0.0;
			tmp.normal_y = 0.0;
			tmp.normal_z = 0.0;
			tmp.intensity = 1.0;
			tmp.curvature = 0.0;
			// Copy_point(shape_cloud->points[i],output_cloud.points[i]);
		}
		// cout<<tmp<<endl;
		/* cout<<"---------pppppppppppppppppppp---------------"<<i<<endl; */
		output_cloud.points.push_back(tmp);
		/* cout<<"55555555555555555555555555555555555555555555555"<<endl;	 */
		//debag
		// cout<<i<<","<<calc_d(output_cloud.points[i])<<endl;
		//debag
	}
	/* cout<<"55555555555555555555555555555555555555555555555"<<endl;	 */
	// Fifth_mean(shape_cloud);
	// Fifth_mean(output_cloud);
	/* cout<<"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<endl; */

	// ros::Time time = ros::Time::now();
	// ros::Time time = cuurent_time;
	pubPointCloud2(shape_pub,output_cloud,"/detect_shape",current_time);
	/* cout<<"ffffffffffffffffffffffffffffffffffffffffffffff"<<endl; */
	output_cloud.points.clear();
	/* cout<<"ggggggggggggggggggggggggggggggggggggggggggg"<<endl; */
}

//callback
CloudAPtr tmp_cloud (new CloudA);
void static_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	/* cout << "static_callback "; */
	pcl::fromROSMsg(*msg,*tmp_cloud);
	callback_flag = true;
	/* cout<<"in"<<endl; */
}

double abs_x = 0.0,abs_y = 0.0, abs_z = 0.0, abs_w = 0.0, abs_yaw = 0.0;
double R = 0.0, P = 0.0, Y = 0.0;
void OdomCallback(const nav_msgs::Odometry input)
{
	/* cout << "odom_callback "; */
	current_time = input.header.stamp;
	abs_x = input.pose.pose.position.x;
	abs_y = input.pose.pose.position.y;
	double qx = input.pose.pose.orientation.x;
	double qy = input.pose.pose.orientation.y;
	double qz = input.pose.pose.orientation.z;
	double qw = input.pose.pose.orientation.w;

	tf::Quaternion tf_quaternion(qx, qy, qz, qw);
	tf::Matrix3x3 tf_matrix(tf_quaternion);
	tf_matrix.getRPY(R, P, Y);
	abs_yaw = Y;
	//abs_yaw = input.pose.pose.orientation.z;

	odom_callback = true;
	/* cout<<"in"<<endl; */
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "calc_shape_saveclouds");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    // ros::Subscriber sub = nh.subscribe ("/local_cloud", 1, static_callback);
    ros::Subscriber sub = nh.subscribe ("/intersection_recognition/save_cloud", 1, static_callback);
	//ros::Subscriber sub_lcl = n.subscribe("/odom",1,OdomCallback);
	ros::Subscriber sub_lcl = n.subscribe("/estimated_pose/pose",1,OdomCallback);
    // Create a ROS publisher for the output point cloud
    shape_pub = nh.advertise<sensor_msgs::PointCloud2> ("/intersection_recognition/detect_shape2", 1);
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
		/* cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl; */
		// if(callback_flag){
		if(callback_flag && odom_callback){
			/* cout << "bfr Calc_shape "; */
			Calc_shape(tmp_cloud,abs_x,abs_y,abs_yaw);
			/* cout << "aft" << endl; */
			callback_flag = false;
			odom_callback = false;
		}
		/* cout<<"#########################################"<<endl; */
        ros::spinOnce();
		/* cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<endl; */
        loop_rate.sleep();
    }
}
