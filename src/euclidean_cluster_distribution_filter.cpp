//euclidean_cluster_distribution_filter.cpp


#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/PCLHeader.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;
CloudIPtr input_pc_ (new CloudI);
CloudIPtr cloud_f_ (new CloudI);
CloudIPtr cloud_filtered_ (new CloudI);
CloudIPtr cloud_plane_ (new CloudI);
CloudIPtr distribution_filtered_pc_ (new CloudI);

typedef pcl::PointIndices PointIndices;
typedef pcl::PointIndices::Ptr PointIndicesPtr;
PointIndicesPtr inliers_ (new PointIndices);

typedef pcl::ModelCoefficients Mdl_Coefficients;
typedef pcl::ModelCoefficients::Ptr MdlcoefficientsPtr;
MdlcoefficientsPtr coefficients_ (new Mdl_Coefficients);

typedef pcl::search::KdTree<PointI> searchKdTree;
typedef pcl::search::KdTree<PointI>::Ptr searchKdTreePtr;
searchKdTreePtr tree_ (new searchKdTree);

pcl::VoxelGrid<PointI> vg;

pcl::SACSegmentation<PointI> seg;

pcl::ExtractIndices<PointI> extract;

pcl::EuclideanClusterExtraction<PointI> ec;

std::vector<PointIndices> cluster_indices;

std::vector<CloudIPtr> Clustered_PC_List;



class EuclideanCluster
{
	public:
		EuclideanCluster();
		
		void filter(void);
		void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void pc_downsample(void);
		void pc_extract(void);
		std::vector<CloudIPtr> clustered_pc_represent(void);
		Eigen::Vector3d calc_euclidean_cluster_size(CloudIPtr);
		CloudIPtr distribution_filter(std::vector<CloudIPtr>);

	private:
		bool pc_callback_flag = false;
		bool downsample_flag = false;

		int x = 0, y = 1, z = 2;
		
		int max_itr;
		int Hz = 100;
		int min_cluster_size;
		int max_cluster_size;
		
		double leaf_size_x;
		double leaf_size_y;
		double leaf_size_z;
		double dist_threshold;
		double downsample_rate;
		double cluster_tolerance;
		double pt_dist_threshold;

		ros::Subscriber sub_pc;
		ros::Publisher pub_pc;

		sensor_msgs::PointCloud2 pc_for_pub;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "euclidean_cluster_distribution_filter");
	
	EuclideanCluster euclidean_cluster;
	euclidean_cluster.filter();
}


EuclideanCluster::EuclideanCluster()
{
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	sub_pc = n.subscribe("/rm_cluster/removed_points", 10, &EuclideanCluster::pc_callback, this);
	pub_pc = n.advertise<sensor_msgs::PointCloud2>("/intersection_recognition/EC_distribution_filtered_pc", 10);
	
	nh.getParam("max_itr", max_itr);
	nh.getParam("min_cluster_size", min_cluster_size);
	nh.getParam("max_cluster_size", max_cluster_size);
	nh.getParam("leaf_size_x", leaf_size_x);
	nh.getParam("leaf_size_y", leaf_size_y);
	nh.getParam("leaf_size_z", leaf_size_z);
	nh.getParam("dist_threshold", dist_threshold);	
	nh.getParam("downsample_rate", downsample_rate);
	nh.getParam("cluster_tolerance", cluster_tolerance);
	nh.getParam("pt_dist_threshold", pt_dist_threshold);
		
	//set parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(max_itr);
	seg.setDistanceThreshold(dist_threshold);
}


void EuclideanCluster::filter(void)
{
	ros::Rate r(Hz);
	while(ros::ok()){
		if(pc_callback_flag){
			pc_downsample();
			pc_extract();
			Clustered_PC_List = clustered_pc_represent();
			std::cout << "Clustered_PC_List size : " << Clustered_PC_List.size() << std::endl;
			*distribution_filtered_pc_ = *distribution_filter(Clustered_PC_List);
			distribution_filtered_pc_->header.stamp = input_pc_->header.stamp;
			distribution_filtered_pc_->header.frame_id = input_pc_->header.frame_id;
			pcl::toROSMsg(*distribution_filtered_pc_, pc_for_pub);
			//pc_for_pub.header.stamp = input_pc_->header.stamp;
			//pc_for_pub.header.stamp = ros::Time::now();
			//pc_for_pub.header.frame_id = input_pc_->header.frame_id;
			//pc_for_pub.header.frame_id = "map";
			pub_pc.publish(pc_for_pub);
			std::cout << "published" << std::endl;
		}
		r.sleep();
		ros::spinOnce();
	}
}


void EuclideanCluster::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "=====================================" <<std::endl;
	pcl::fromROSMsg(*msg, *input_pc_);
	
	pc_callback_flag = true;
}


void EuclideanCluster::pc_downsample(void)
{
	// Create the filtering object: downsample the dataset using each leaf size
	vg.setInputCloud(input_pc_);
	vg.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
	vg.filter(*cloud_filtered_);
	std::cout << cloud_filtered_->points.size() <<std::endl;
}


void EuclideanCluster::pc_extract(void)
{
	int nr_points = (int)cloud_filtered_->points.size();

	/* while(cloud_filtered_->points.size() > downsample_rate * nr_points){ */
	/* 	// Segment the largest planar component from the remaining cloud */
	/* 	seg.setInputCloud(cloud_filtered_); */
	/* 	seg.segment(*inliers_, *coefficients_); */
	/* 	if(inliers_->indices.size() == 0){ */
	/* 		std::cout << "Could not estimate a planar model for the given dataset." << std::endl; */
	/* 		break; */
	/* 	} */

		// Extract the planar inliers from the input cloud
		/*
		extract.setInputCloud(cloud_filtered_);
		extract.setIndices(inliers_);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane_);

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f_);
		*cloud_filtered_ = *cloud_f_;
		*/	
		//std::cout << "cloud_filtered size : " << cloud_filtered_->points.size() << std::endl;


		// Creating the KdTree object for the search method of the extraction
		tree_->setInputCloud(cloud_filtered_);
		ec.setClusterTolerance(cluster_tolerance);
		ec.setMinClusterSize(min_cluster_size);
		ec.setMaxClusterSize(max_cluster_size);
		ec.setSearchMethod(tree_);
		ec.setInputCloud(cloud_filtered_);
		ec.extract(cluster_indices);
		std::cout << "cluster_indices size : " << cluster_indices.size() << std::endl;
		//std::cout << "cluster_indices[0] = " << cluster_indices[0] << std::endl;
	// }
}


std::vector<CloudIPtr> EuclideanCluster::clustered_pc_represent(void)
{
	std::vector<PointIndices>::const_iterator it;
	std::vector<CloudIPtr> clustered_pc_list;
	for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
		CloudIPtr cloud_cluster_ (new CloudI);
		for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			cloud_cluster_->points.push_back(cloud_filtered_->points[*pit]);
		}
		cloud_cluster_->width = cloud_cluster_->points.size();
		std::cout << "cloud_cluster_ size : " << cloud_cluster_->points.size() << std::endl;
		cloud_cluster_->height = 1;
		cloud_cluster_->is_dense = true;
		
		clustered_pc_list.push_back(cloud_cluster_);
	}
	
	return clustered_pc_list;
}


CloudIPtr EuclideanCluster::distribution_filter(std::vector<CloudIPtr> clustered_PC_list)
{
	CloudIPtr clustered_points_ (new CloudI);
	CloudIPtr distribution_filtered_points_ (new CloudI);
	size_t list_size = clustered_PC_list.size();

	std::cout << "list_size : " << list_size << std::endl;

	for(size_t i = 0; i < list_size; i++){
		//calculate center of the clustered point cloud
		bool distribution_x_flag = false;
		bool distribution_y_flag = false;
		bool distribution_z_flag = false;
		*clustered_points_ = *clustered_PC_list.at(i);

		std::cout << "clustered_points_ size : " << clustered_points_->points.size() << std::endl;

		Eigen::Vector3d EC_size = calc_euclidean_cluster_size(clustered_points_);
		
		//check distribution
		if(EC_size[x] > pt_dist_threshold){
			distribution_x_flag = true;
		}
		if(EC_size[y] > pt_dist_threshold){
			distribution_y_flag = true;
		}
		if(EC_size[z] > pt_dist_threshold){
			distribution_z_flag = true;
		}

		//input filtered points
		if(distribution_x_flag || distribution_y_flag || distribution_z_flag){
			*distribution_filtered_points_ += *clustered_points_;
		}
	}
	
	std::cout << "distribution_filtered_points size : " << distribution_filtered_points_->points.size() << std::endl;

	return distribution_filtered_points_;
}

Eigen::Vector3d EuclideanCluster::calc_euclidean_cluster_size(CloudIPtr clst_pc_)
{
	Eigen::Vector3d euclidean_cluster_size(0, 0, 0);
	Eigen::Vector3d min_coordinate(0, 0, 0);
	Eigen::Vector3d max_coordinate(0, 0, 0);

	for(auto& pt : clst_pc_->points){
		if(min_coordinate[x] > pt.x){
			min_coordinate[x] = pt.x;
		}
		if(max_coordinate[x] < pt.x){
			max_coordinate[x] = pt.x;
		}
		if(min_coordinate[y] > pt.y){
			min_coordinate[y] = pt.y;
		}
		if(max_coordinate[y] < pt.y){
			max_coordinate[y] = pt.y;
		}
		if(min_coordinate[z] > pt.z){
			min_coordinate[z] = pt.z;
		}
		if(max_coordinate[z] < pt.z){
			max_coordinate[z] = pt.z;
		}
	}
	euclidean_cluster_size = max_coordinate - min_coordinate;

	return euclidean_cluster_size;
}
