#include <ros/ros.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/SVD>
//PCL
//ransac
#include <pcl/ModelCoefficients.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  

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

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

typedef pcl::PointXYZI PointX;
typedef pcl::PointCloud<PointX> CloudX;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;

bool velodyne_flag = false;
double height_cut = 0.5;

ros::Time current_time;
ros::Publisher gauss_pub;
ros::Publisher curv_pub;

//  output 
inline void pubPointCloud2 (ros::Publisher& pub, 
                            const CloudA& cloud,
                            const char* frame_id,
                            ros::Time& time)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id ="velodyne";
    output.header.stamp = time;
    pub.publish (output);
}
//

inline void Copy_cloud(PointA& point_in,
					   PointA& point_out)
{
	// points_with_normal
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
	point_out.normal_x = point_in.normal_x;
	point_out.normal_y = point_in.normal_y;
	point_out.normal_z = point_in.normal_z;
	point_out.curvature = point_in.curvature;
	point_out.intensity = point_in.intensity;
	
	// gauss_shere
	// point_out.x = point_in.normal_x;
	// point_out.y = point_in.normal_y;
	// point_out.z = point_in.normal_z;
	// point_out.normal_x = point_in.normal_x;
	// point_out.normal_y = point_in.normal_y;
	// point_out.normal_z = point_in.normal_z;
	// point_out.curvature = point_in.curvature;
	// point_out.intensity = point_in.intensity;
}
inline void Visualize(CloudAPtr cloud)
{
	size_t cloud_size = cloud->points.size();
	size_t count = 0;
	// double para = 0.985;//tsukuba glass
	double para = 0.96;//ikuta sunny
	// double para = 0.95;
	// double para = 0.94;//ikuta rain
	// double para = 0.90;
	for(size_t i=0;i<cloud_size;i++){
        // if (fabs(cloud->points[i].normal_z) >  para){
        if (fabs(cloud->points[i].normal_z) <  para){
			if(cloud->points[i].z<height_cut){
				count++;
			}
		}
	}

	CloudAPtr output_cloud(new CloudA);
	output_cloud->points.resize(count);
	int output = 0;
	for(size_t i=0;i<cloud_size;i++){
        // if (fabs(cloud->points[i].normal_z) >  para){
        if (fabs(cloud->points[i].normal_z) <  para){
			if(cloud->points[i].z<height_cut){
				Copy_cloud(cloud->points[i],output_cloud->points[output]);
				// cout<<cloud->points[i].x<<endl;
				// cout<<output_cloud->points[output].normal_z<<endl;
				// cout<<"aa"<<endl;
				output++;
			}
		}
	}
    // output
    // ros::Time time = ros::Time::now();
    pubPointCloud2 (gauss_pub, *output_cloud, "/normal_cloud", current_time);
}

//RANSAC
// inline void Visualize(CloudAPtr cloud)
// {
// 	size_t cloud_size = cloud->points.size();
// 	size_t count = 0;
// 	double para = 0.99;
// 	for(size_t i=0;i<cloud_size;i++){
//         if (fabs(cloud->points[i].normal_z) >  para){
// 				count++;
// 		}
// 	}
//
// 	CloudA output_cloud;
// 	output_cloud.points.resize(count);
// 	int output = 0;
// 	for(size_t i=0;i<cloud_size;i++){
//         if (fabs(cloud->points[i].normal_z) >  para){
// 				Copy_cloud(cloud->points[i],output_cloud.points[output]);
// 				// cout<<cloud->points[i].x<<endl;
// 				// cout<<output_cloud->points[output].normal_z<<endl;
// 				// cout<<"aa"<<endl;
// 				output++;
// 		}
// 	}
// 	// ransac
// 	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
// 	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
// 	// Create the segmentation object  
// 	pcl::SACSegmentation<PointA> seg;  
// 	// Optional  
// 	seg.setOptimizeCoefficients (true);  
// 	// Mandatory  
// 	seg.setModelType (pcl::SACMODEL_PLANE);  
// 	seg.setMethodType (pcl::SAC_RANSAC);  
// 	seg.setDistanceThreshold (0.05);  
// 	      
// 	seg.setInputCloud (output_cloud.makeShared ());  
// 	seg.segment (*inliers, *coefficients);  
//     // output
//     ros::Time time = ros::Time::now();
//     pubPointCloud2 (gauss_pub, output_cloud, "/normal_cloud", time);
// }

inline void Visualize_curv(CloudAPtr cloud)
{
	size_t cloud_size = cloud->points.size();
	size_t count = 0;
	// double para = 0.10;
	// double para = 0.17;
	// double para = 0.20;
	double para = 0.23;//ikuta sunny
	// double para = 0.25;
	// double para = 0.30;
	// double para = 0.37;//ikuta rain
	// double para = 0.80;
	for(size_t i=0;i<cloud_size;i++){
        if (fabs(cloud->points[i].curvature) >  para){
			if(cloud->points[i].z<height_cut){
				count++;
			}
		}
	}

	CloudA output_cloud;
	output_cloud.points.resize(count);
	int output = 0;
	for(size_t i=0;i<cloud_size;i++){
        if (fabs(cloud->points[i].curvature) >  para){
			if(cloud->points[i].z<height_cut){
				Copy_cloud(cloud->points[i],output_cloud.points[output]);
				// cout<<cloud->points[i].x<<endl;
				// cout<<output_cloud->points[output].normal_z<<endl;
				// cout<<"aa"<<endl;
				output++;
			}
		}
	}
    // output
    // ros::Time time = ros::Time::now();
    pubPointCloud2 (curv_pub, output_cloud, "/curvature_cloud", current_time);
}

// Downsample 
inline void Downsample (CloudXPtr input_cloud, 
                        CloudXPtr downsampled,
                        float size)
{
    pcl::VoxelGrid<PointX> sor;
    sor.setInputCloud (input_cloud);
    sor.setLeafSize (size, size, size);
    sor.filter (*downsampled);
}//


//callback
CloudAPtr tmp_cloud (new CloudA);
void normal_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	current_time = msg->header.stamp;
	pcl::fromROSMsg(*msg,*tmp_cloud);
	velodyne_flag = true;
}
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "normal_curvature_pc_bottom");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/perfect_velodyne/normal", 1, normal_cb);
    // Create a ROS publisher for the output point cloud
    // gauss_pub = nh.advertise<sensor_msgs::PointCloud2> ("/normal_cloud", 1);
    // gauss_pub = nh.advertise<sensor_msgs::PointCloud2> ("/static_cloud2", 1);
    gauss_pub = nh.advertise<sensor_msgs::PointCloud2> ("/static_cloud", 1);
    // curv_pub = nh.advertise<sensor_msgs::PointCloud2> ("/curvature_cloud2", 1);
    curv_pub = nh.advertise<sensor_msgs::PointCloud2> ("/curvature_cloud", 1);
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
        if (velodyne_flag){
			velodyne_flag = false;
			// visualize_points
			Visualize(tmp_cloud);
			Visualize_curv(tmp_cloud);
        } 
        ros::spinOnce();
        loop_rate.sleep();
    }
}
