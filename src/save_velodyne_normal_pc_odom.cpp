/*
 * velodyneの点群をGyro odometryに付加させ
 * 数秒間貯める
 * 
 * 
 * 
 *
*/

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>

#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <omp.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>


using namespace Eigen;
using namespace std;	

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool odom_callback = false;
bool points_callback = false;
bool grass_pc_callback_flag = false;
double d_x=0.0, d_y=0.0, d_z=0.0;
double angle_x_=0.0, angle_y_=0.0, angle_z_=0.0;
double old_yaw=0.0;
int loop = 11; //original
double odom_threshold = 0.3;
int SAVE_SIZE = 15000;

ros::Time current_time;
ros::Publisher shape_pub;
ros::Publisher debug_pub;

//publish pointcloud
/*
inline void pubPointCloud2(ros::Publisher& pub, 
                            const CloudA& cloud,
                            const char* frame_id,
                            ros::Time& time)
{
	std::cout << "publish!" << std::endl;
	
	cloud.width = cloud.points.size();
	cloud.height = 1;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id ="map";
    output.header.stamp = time;
    pub.publish (output);
}
*/

inline bool Odometry_threshold(double &s_x,double &s_y,double &x,double &y)
{
	double distance = sqrt( pow(s_x -x,2) + pow(s_y - y,2) );
	if(distance>odom_threshold){
		return true;
	}else{
		return false;
	}
}

/*
inline void Copy_point(PointA& org,PointA& save)
{
	if(fabs(angle_z_-old_yaw)<0.01){
		save.x = org.x;
		save.y = org.y;
		save.z = org.z;
		save.normal_x = org.normal_x;
		save.normal_y = org.normal_y;
		save.normal_z = org.normal_z;
		save.intensity = org.intensity;
		save.curvature = org.curvature;
	}
}
*/

void cnv(CloudAPtr org, CloudAPtr rsl, 
		 double dx, double dy, double dz, double angle_x, double angle_y, double angle_z, size_t SIZE_)
{
	// cout << "normal conv" << endl;
	pcl::PointNormal p;

	Vector3d point_in, point_out, axis_x, axis_y ,axis_z;
	Quaterniond quat_X, quat_Y, quat_Z;
	axis_x << 1.0, 0.0, 0.0;
	axis_y << 0.0, 1.0, 0.0;
	axis_z << 0.0, 0.0, 1.0;

	quat_X = AngleAxisd(angle_x, axis_x); // x軸を中心に回転するquat作成
	quat_Y = AngleAxisd(angle_y, axis_y); // y軸
	quat_Z = AngleAxisd(angle_z, axis_z); // z軸
	Quaterniond total_quat;
	total_quat = quat_Z * quat_Y * quat_X;
	
	size_t SIZE = org->points.size();
	//rsl->points.resize(SIZE);

	*rsl = *org;
	for(size_t i=0; i<SIZE; i++){	
		point_in << org->points[i].x, org->points[i].y, org->points[i].z;
        
		//回転
		point_out = total_quat * point_in;
        
		//並進
		rsl->points[i].x = point_out(0) + dx;
		rsl->points[i].y = point_out(1) + dy;
		rsl->points[i].z = point_out(2) + dz;
		
		rsl->points[i].curvature = org->points[i].curvature;
		if(org->points[i].x==0&&org->points[i].y==0){
			rsl->points[i].x = 100000.0;
			rsl->points[i].y = 100000.0;
		}
	}
	std::cout << "cnv" << std::endl;
}

CloudAPtr tmp_cloud (new CloudA);
void static_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	std::cout << "pc callback" << std::endl;
	pcl::fromROSMsg(*msg,*tmp_cloud);
	points_callback = true;
}


CloudAPtr grass_pc (new CloudA);
void grass_pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	std::cout << "grass callback" << std::endl;
	pcl::fromROSMsg(*msg, *grass_pc);
	grass_pc_callback_flag = true;
}


void OdomCallback(const nav_msgs::Odometry input){
	
	current_time = input.header.stamp;
	d_x = input.pose.pose.position.x;
	d_y = input.pose.pose.position.y;
	d_z = 0.0;
	old_yaw = angle_z_;
	
	double qx = input.pose.pose.orientation.x;
	double qy = input.pose.pose.orientation.y;
	double qz = input.pose.pose.orientation.z;
	double qw = input.pose.pose.orientation.w;
	double R=0.0, P=0.0, Y=0.0;
	tf::Quaternion tf_quaternion(qx, qy, qz, qw);
	tf::Matrix3x3 tf_matrix(tf_quaternion);
	tf_matrix.getRPY(R, P, Y);
	angle_z_ = Y;
	//angle_z_ = input.pose.pose.orientation.z;

	std::cout << "odom callback" << std::endl;
	odom_callback = true;
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "save_velodyne_normal_pc_odom");
  	ros::NodeHandle n;
    ros::NodeHandle nh("~");

	int cloud_size_threshold;
	nh.getParam("cloud_size_threshold", cloud_size_threshold);

    ros::Subscriber sub = nh.subscribe ("/intersection_recognition/EC_distribution_filtered_pc", 1, static_callback);
    ros::Subscriber grass_sub = nh.subscribe ("/grass_points", 1, grass_pc_callback);
	//ros::Subscriber sub_lcl = n.subscribe("/tinypower/odom",1,OdomCallback);
	ros::Subscriber sub_lcl = n.subscribe("/estimated_pose/pose",1,OdomCallback);
    
    shape_pub = nh.advertise<sensor_msgs::PointCloud2> ("/intersection_recognition/save_cloud", 1);

	CloudAPtr conv_cloud (new CloudA);
	CloudAPtr conv_cloud_grass (new CloudA);
	CloudAPtr save_cloud (new CloudA);
	//save_cloud->resize(SAVE_SIZE * loop);
	//save_cloud->resize(SAVE_SIZE);
	int cloud_size_diff = 0;
	int save_count = 0;
	double saveodom_x = 0.0;
	double saveodom_y = 0.0;
	
	ros::Rate loop_rate(10);
	while (ros::ok()){
		//std::cout << "while() start" << std::endl;
		if(odom_callback && points_callback){
			std::cout << "flags ok and next cnv" << std::endl;
			size_t cnv_size = tmp_cloud->points.size();
			
			cnv(tmp_cloud, conv_cloud, d_x, d_y, d_z, angle_x_, angle_y_, angle_z_, cnv_size);
			*save_cloud += *conv_cloud;
	



			
			
			
			if(grass_pc_callback_flag){
				std::cout << "aaaaaa" << std::endl;
				size_t grass_size = grass_pc->points.size();
				cnv(grass_pc, conv_cloud_grass, d_x, d_y, d_z, angle_x_, angle_y_, angle_z_, grass_size);
				*save_cloud += *conv_cloud_grass;
				/*
				for(size_t i=0;i<grass_size;i++){
					save_cloud->points.push_back(conv_cloud_grass->points[i]);
				}
				*/
			}
		





			
			
			// if(grass_pc_callback_flag){

						// }	
			/*
			for(size_t i=0;i<cloud_size;i++){
				Copy_point(conv_cloud->points[i],save_cloud->points[SAVE_SIZE*save_count+i]);
			}
			*/
			
			///////////////////save_cloud/////////////////////////
			// omp_set_num_threads(4);
			// #pragma omp parallel for
						
			int cloud_size = (int)save_cloud->points.size();
			cloud_size_diff = cloud_size - cloud_size_threshold;
			/////////////////////save_cloud/////////////////////////
			cout<<"conv cloud size : "<<cloud_size<<endl;
			
			if(cloud_size_diff > 0){
				save_cloud->points.erase(save_cloud->points.begin(), (save_cloud->points.begin() + (size_t)(cloud_size_diff)));
				std::cout << "resized save_cloud size : " << save_cloud->points.size() << std::endl;
			}

			//pubPointCloud2(shape_pub,*save_cloud,"/map",current_time);
			
			if(Odometry_threshold(saveodom_x,saveodom_y,d_x,d_y)){
				save_count++;
				save_count = save_count%loop;
				saveodom_x = d_x;
				saveodom_y = d_y;
			}
			
			save_cloud->width = save_cloud->points.size();
			save_cloud->height = 1;
		
			sensor_msgs::PointCloud2 output;
    		pcl::toROSMsg(*save_cloud, output);
    		output.header.frame_id ="map";
    		//output.header.stamp = time;
    		output.header.stamp = current_time;
    		shape_pub.publish (output);

			odom_callback = false;
			points_callback = false;
			grass_pc_callback_flag = false;
			

		}
        ros::spinOnce();
        loop_rate.sleep();
	}
	return (0);
}

