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
#include <infant_msgs/PointCloud2WithOdometry.h>
#include <pcl/filters/voxel_grid.h>

#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <omp.h>


using namespace Eigen;
using namespace std;	

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool odom_callback = false;
bool points_callback = false;
double d_x=0.0, d_y=0.0, d_z=0.0;
double angle_x_=0.0, angle_y_=0.0, angle_z_=0.0;
double old_yaw=0.0;
int loop = 11; //original
/* int loop = 15; */
double odom_threshold = 0.3;
/* double odom_threshold = 0.5; */
/* int SAVE_SIZE = 10000; */
/* int SAVE_SIZE = 17000; */
int SAVE_SIZE = 15000;

ros::Time current_time;
ros::Publisher shape_pub;
ros::Publisher debug_pub;

//publish pointcloud
inline void pubPointCloud2(ros::Publisher& pub, 
                            const CloudA& cloud,
                            const char* frame_id,
                            ros::Time& time)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id ="map";
    output.header.stamp = time;
    pub.publish (output);
}

inline bool Odometry_threshold(double &s_x,double &s_y,double &x,double &y)
{
	double distance = sqrt( pow(s_x -x,2) + pow(s_y - y,2) );
	if(distance>odom_threshold){
		return true;
	}else{
		return false;
	}
}

inline void Copy_point(PointA& org,PointA& save)
{
	/* if(fabs(d_x-org.x)<50 && fabs(d_y-org.y)<50 && fabs(angle_z_-old_yaw)<0.03){ */
	/* if(fabs(angle_z_-old_yaw)>0.01) cout<<"dyaw="<<fabs(angle_z_-old_yaw)<<endl; */
	if(fabs(angle_z_-old_yaw)<0.01){
		/* if(!(fabs(d_x-org.x)==0 && fabs(d_y-org.y)==0 && fabs(d_z-org.z)==0)){ */
		save.x = org.x;
		save.y = org.y;
		save.z = org.z;
		save.normal_x = org.normal_x;
		save.normal_y = org.normal_y;
		save.normal_z = org.normal_z;
		save.intensity = org.intensity;
		save.curvature = org.curvature;
	}
	// }
}

void cnv(CloudAPtr org, CloudAPtr rsl, 
		 double dx, double dy, double dz, double angle_x, double angle_y, double angle_z)
{
	// cout << "normal conv" << endl;
	pcl::PointNormal p;

	Vector3d point_in, point_out, axis_x, axis_y ,axis_z;
	Quaterniond quat_X, quat_Y, quat_Z;
	axis_x << 1.0, 0.0, 0.0;
	axis_y << 0.0, 1.0, 0.0;
	axis_z << 0.0, 0.0, 1.0;

	// quat_X = AngleAxisd(angle_x/180.0*M_PI, axis_x); // x軸を中心に回転するquat作成
	// quat_Y = AngleAxisd(angle_y/180.0*M_PI, axis_y); // y軸
	// quat_Z = AngleAxisd(angle_z/180.0*M_PI, axis_z); // z軸
	quat_X = AngleAxisd(angle_x, axis_x); // x軸を中心に回転するquat作成
	quat_Y = AngleAxisd(angle_y, axis_y); // y軸
	quat_Z = AngleAxisd(angle_z, axis_z); // z軸
	Quaterniond total_quat;
	total_quat = quat_Z * quat_Y * quat_X;
	
	size_t SIZE = org->points.size();
	rsl->points.resize(SIZE);

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
	// cout<<"moved!!\n";
}

//callback
CloudAPtr tmp_cloud (new CloudA);
void static_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,*tmp_cloud);
	points_callback = true;
}

double Distance_zed(PointA zed){
	double x = zed.x;
	double y = zed.y;
	double distance = sqrt(pow(x,2)+pow(y,2));

	return distance;
}

bool zed_flag = false;
// nav_msgs::Odometry save_odom;
// sensor_msgs::PointCloud zed_in;
CloudAPtr zed_cloud (new CloudA);
// CloudA zed_debug ;

/* void ZedCallback(infant_msgs::PointCloud2WithOdometry input){ */
void ZedCallback(sensor_msgs::PointCloud2 input){
		sensor_msgs::PointCloud2 pc2;
		// pc2 = input.point_cloud2;
		// sensor_msgs::convertPointCloud2ToPointCloud(pc2,zed_in);
		CloudAPtr zed_in (new CloudA);
		CloudAPtr zed_in2 (new CloudA);
		// pcl::fromROSMsg(pc2,*zed_in);
		pcl::fromROSMsg(input,*zed_in);
	// if(zed_in.points.size()>10000){
		// cout<<"zed_points"<<zed_in->points.size()<<endl;
		zed_flag = true;

		for(size_t i = 0;i<zed_in->points.size();i++){
			if(zed_in->points[i].z<-0.15){
				if(Distance_zed(zed_in->points[i])<10.0){
					/* cout<<"zed callback!!!!!!!!!!!!!!!!!!"<<endl; */
					zed_in2->push_back(zed_in->points[i]);
				}
			}
		}
	
		//save_odom
		// save_odom.header.frame_id = "map";
		// save_odom.child_frame_id = "matching_base_link";
		// save_odom.header.stamp = ros::Time::now();
		// save_odom.pose.pose.position.x = input.pose.pose.position.x;
		// save_odom.pose.pose.position.y = input.pose.pose.position.y;
		// save_odom.pose.pose.position.z = 0.0;
		// save_odom.pose.pose.orientation.z = input.pose.pose.orientation.z;
		// save_odom.pose.pose.position.x = zed_odom.pose.pose.position.x;
		// save_odom.pose.pose.position.y = zed_odom.pose.pose.position.y;
		// save_odom.pose.pose.position.z = 0.0;
		// save_odom.pose.pose.orientation.z = zed_odom.pose.pose.orientation.z;
	// }
		pcl::VoxelGrid<pcl::PointXYZINormal> vg;  
		// CloudAPtr ds_cloud (new CloudA);  
		vg.setInputCloud (zed_in2);  
		vg.setLeafSize (0.015f, 0.015f, 0.015f);
		/* vg.setLeafSize (0.15f, 0.15f, 0.15f); */
		vg.filter (*zed_cloud);
		// vg.filter (zed_debug);
		// cout<<"zed_points_downsample"<<zed_cloud->points.size()<<endl;
		// cout<<"zed_points_downsample"<<zed_debug.points.size()<<endl;
		// ros::Time time = ros::Time::now();
		// pubPointCloud2(debug_pub,zed_debug,"/aaaa",time);
}

bool intensity_flag = false;
CloudAPtr intensity_cloud(new CloudA);
void IntensityCallback(sensor_msgs::PointCloud2 msg)
{
	CloudAPtr intensity_tmp(new CloudA);
	pcl::fromROSMsg(msg,*intensity_tmp);
	pcl::VoxelGrid<pcl::PointXYZINormal> vg;  
	vg.setInputCloud (intensity_tmp);  
	/* vg.setLeafSize (0.05f, 0.05f, 0.05f); */
	vg.setLeafSize (0.1f, 0.1f, 0.1f);
	vg.filter (*intensity_cloud);
	intensity_flag = true;
	// cout<<"intensity cloud size:"<<intensity_cloud->points.size()<<endl;
}

void OdomCallback(const nav_msgs::Odometry input){
	current_time = input.header.stamp;
	d_x = input.pose.pose.position.x;
	d_y = input.pose.pose.position.y;
	d_z = 0.0;
	old_yaw = angle_z_;
	angle_z_ = input.pose.pose.orientation.z;
	/* cout<<"old:"<<old_yaw<<", now:"<<angle_z_<<endl; */

	odom_callback = true;
}
CloudAPtr save_cloud (new CloudA);
double diff_yaw = 0.0;
void MnckCallback(const std_msgs::Float64 msg){
	// diff_yaw = msg.data;
	save_cloud->points.clear();
	save_cloud->resize(SAVE_SIZE * loop);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "save_velodyne_points");
  	ros::NodeHandle n;
    ros::NodeHandle nh;
	
	size_t max_size = 0;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/local_cloud", 1, static_callback);
	ros::Subscriber sub_lcl = n.subscribe("/lcl",1,OdomCallback);
	/* ros::Subscriber sub_zed = n.subscribe("/movable_image/points",1,ZedCallback); */
	ros::Subscriber sub_zed = n.subscribe("/zed_grasspoints",1,ZedCallback);
	/* ros::Subscriber sub_intensity = n.subscribe("/intensity/remove",1,IntensityCallback); */
	ros::Subscriber sub_intensity = n.subscribe("/velodyne_points/inte",1,IntensityCallback);
	ros::Subscriber sub_mnck = n.subscribe("/flag/diff",1,MnckCallback);
    // Create a ROS publisher for the output point cloud
    shape_pub = nh.advertise<sensor_msgs::PointCloud2> ("/save_cloud", 1);
    debug_pub = nh.advertise<sensor_msgs::PointCloud2> ("/zed_debug", 1);

	// CloudAPtr conv_cloud (new CloudA);
	CloudAPtr conv_cloud (new CloudA);
	CloudAPtr conv_zed_cloud (new CloudA);
	CloudAPtr conv_intensity_cloud (new CloudA);
	// CloudAPtr save_cloud (new CloudA);
	save_cloud->resize(SAVE_SIZE * loop);
	int save_count = 0;
	double saveodom_x = 0.0;
	double saveodom_y = 0.0;
	// ros::Rate loop_rate(5);
	ros::Rate loop_rate(10);
	/* ros::Rate loop_rate(20); // 20 */
	// ros::Rate loop_rate(30); // 20
	while (ros::ok()){
		if(odom_callback && points_callback){
			// cout<<"in!!!!"<<endl;
			odom_callback = false;
			points_callback = false;
			cnv(tmp_cloud, conv_cloud, d_x, d_y, d_z, angle_x_, angle_y_, angle_z_);
			if(zed_flag){
				// cout<<"zed_convert"<<endl;
				cnv(zed_cloud, conv_zed_cloud, d_x, d_y, d_z, angle_x_, angle_y_, angle_z_);
			}
			if(intensity_flag){
				// cout<<"intensity_points_convert"<<endl;
				cnv(intensity_cloud, conv_intensity_cloud, d_x, d_y, d_z, angle_x_, angle_y_, angle_z_);
			}
			CloudA pub_cloud;
			size_t cloud_size = conv_cloud->points.size();
			size_t cloud_zed_size = conv_zed_cloud->points.size();
			size_t cloud_intensity_size = conv_intensity_cloud->points.size();
			if(max_size<cloud_size){
				// cout<<cloud_size<<endl;
				max_size = cloud_size;
			}
			// for(size_t i=0;i<cloud_size;i++){
			// 	pub_cloud.push_back(conv_cloud->points[i]);
			// }
			/////////////////////save_cloud/////////////////////////
			// cout<<"minmax_point save!!!!!!"<<cloud_size<<endl;
			for(size_t i=0;i<cloud_size;i++){
				Copy_point(conv_cloud->points[i],save_cloud->points[SAVE_SIZE*save_count+i]);
			}
			if(zed_flag){
				// cout<<"zed_point save!!!!!!"<<cloud_zed_size<<endl;
				for(size_t i = 0;i<cloud_zed_size;i++){
					Copy_point(conv_zed_cloud->points[i],save_cloud->points[SAVE_SIZE*save_count+i+cloud_size]);
				}
				zed_flag = false;
			}
			if(intensity_flag){
				// cout<<"intensity_point save!!!!!!"<<cloud_intensity_size<<endl;
				for(size_t i = 0;i<cloud_intensity_size;i++){
					Copy_point(conv_intensity_cloud->points[i],save_cloud->points[SAVE_SIZE*save_count+i+cloud_size+cloud_zed_size]);
				}
			}
			PointA surplus;
			surplus.x = 0;
			surplus.y = 0;
			surplus.z = 0;
			omp_set_num_threads(4);
			#pragma omp parallel for
			for(size_t i = SAVE_SIZE*save_count+cloud_size+cloud_zed_size+cloud_intensity_size;i<SAVE_SIZE*(save_count+1);i++){
				Copy_point(surplus,save_cloud->points[i]);
			}
			// cout<<"======"<<SAVE_SIZE*save_count+cloud_size+cloud_zed_size+cloud_intensity_size<<endl;
			// cout<<"!!!!!!!!!"<<save_count<<endl;
			///////////////////save_cloud/////////////////////////
			// omp_set_num_threads(4);
			// #pragma omp parallel for
			for(size_t i=0;i<SAVE_SIZE*loop;i++){
				pub_cloud.push_back(save_cloud->points[i]);
			}
			if(Odometry_threshold(saveodom_x,saveodom_y,d_x,d_y)){
				save_count++;
				save_count = save_count%loop;
				saveodom_x = d_x;
				saveodom_y = d_y;
				intensity_flag = false;
			}
			// ros::Time time = ros::Time::now();
			pubPointCloud2(shape_pub,pub_cloud,"/map",current_time);
		}
        ros::spinOnce();
        loop_rate.sleep();
	}
	return (0);
}

