/*

Clustering using PCL Libraties

Publish
    pub_bbox        : jsk_visualize
    pub_cluster     : cluster result
    pub_centroid    : cluster centroid
    pub_point       : cluster pointcloud
Subscribe
    sub : pointcloud

Causion:
    using original msg (amsl_recog_msgs, jsk_recognition_msgs)


*/

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

#include <Eigen/Core>

using namespace Eigen;
using namespace sensor_msgs;
using std::cout;
using std::endl;
using std::vector;

typedef pcl::PointXYZI PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub_cluster;
ros::Publisher pub_centroid;
ros::Publisher pub_points;

struct Cluster{
    float x; 
    float y; 
    float z;
    float width;
    float height;
    float depth;
    float curvature;
    Vector3f min_p;
    Vector3f max_p;
};

template<typename T_c>
struct Clusters{
    Cluster data;
    T_c centroid;
    T_c points;
};

template<typename T_c>
void getClusterInfo(T_c pt, Cluster& cluster)
{
    Vector3f centroid;
    centroid[0]=pt.points[0].x;
    centroid[1]=pt.points[0].y;
    centroid[2]=pt.points[0].z;

    Vector3f min_p;
    min_p[0]=pt.points[0].x;
    min_p[1]=pt.points[0].y;
    min_p[2]=pt.points[0].z;

    Vector3f max_p;
    max_p[0]=pt.points[0].x;
    max_p[1]=pt.points[0].y;
    max_p[2]=pt.points[0].z;

    for(size_t i=1;i<pt.points.size();i++){
        centroid[0]+=pt.points[i].x;
        centroid[1]+=pt.points[i].y;
        centroid[2]+=pt.points[i].z;
        if (pt.points[i].x<min_p[0]) min_p[0]=pt.points[i].x;
        if (pt.points[i].y<min_p[1]) min_p[1]=pt.points[i].y;
        if (pt.points[i].z<min_p[2]) min_p[2]=pt.points[i].z;

        if (pt.points[i].x>max_p[0]) max_p[0]=pt.points[i].x;
        if (pt.points[i].y>max_p[1]) max_p[1]=pt.points[i].y;
        if (pt.points[i].z>max_p[2]) max_p[2]=pt.points[i].z;
    }

    cluster.x=centroid[0]/(float)pt.points.size();
    cluster.y=centroid[1]/(float)pt.points.size();
    cluster.z=centroid[2]/(float)pt.points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.min_p = min_p;
    cluster.max_p = max_p;
}

// Clustering
template<typename T_p, typename T_c, typename T_ptr>
void clustering(T_ptr cloud_in, vector<Clusters<T_c> >& cluster_array)
{
    //Downsample//
    pcl::VoxelGrid<T_p> vg;
    T_ptr ds_cloud (new T_c);
    vg.setInputCloud (cloud_in);  
    vg.setLeafSize (0.08f, 0.08f, 0.08f);
    vg.filter (*ds_cloud);

    //downsampled point's z =>0
    vector<float> tmp_z;
    tmp_z.resize(ds_cloud->points.size());
	for(int i=0;i<(int)ds_cloud->points.size();i++){
        tmp_z[i]=ds_cloud->points[i].z;
		ds_cloud->points[i].z  = 0.0;
    }

    //Clustering//
    typename pcl::search::KdTree<T_p>::Ptr tree (new pcl::search::KdTree<T_p>);
    tree->setInputCloud (ds_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<T_p> ec;
    ec.setClusterTolerance (0.20); // 15cm
    ec.setMinClusterSize (50);//50
    ec.setMaxClusterSize (1200);//1600
    ec.setSearchMethod (tree);
    ec.setInputCloud(ds_cloud);
    ec.extract (cluster_indices);
    //reset z value
	for(int i=0;i<(int)ds_cloud->points.size();i++)
        ds_cloud->points[i].z=tmp_z[i];

    for(int iii=0;iii<(int)cluster_indices.size();iii++)
    {
        // cluster points
        T_ptr cloud_cluster (new T_c);
        cloud_cluster->points.resize(cluster_indices[iii].indices.size());
        // cluster data
        Cluster data;
        for(int jjj=0;jjj<int(cluster_indices[iii].indices.size());jjj++){
            int p_num = cluster_indices[iii].indices[jjj];
            cloud_cluster->points[jjj] = ds_cloud->points[p_num];
        }
        getClusterInfo(*cloud_cluster, data);
		
		T_p center;
		center.x = data.x;
		center.y = data.y;
		center.z = data.z;
		
        Clusters<T_c> cluster;
        cluster.data = data;
        cluster.centroid.points.push_back(center);
        for(size_t i=0;i<cloud_cluster->points.size();i++){
            cluster.points.points.push_back(cloud_cluster->points[i]);
		}cluster_array.push_back(cluster);
    }
}

// Callback
template<typename T_p, typename T_c, typename T_ptr>
void pcCallback(const PointCloud2ConstPtr msg)
{   
    T_ptr cloud(new T_c);
    pcl::fromROSMsg(*msg, *cloud);

    vector<Clusters<T_c> > cluster_array;
    if(0<cloud->points.size())
        clustering<T_p, T_c, T_ptr>(cloud, cluster_array);

    amsl_recog_msgs::ObjectInfoArray object_array;   
 
    std::string target_frame = msg->header.frame_id;

    // for amsl recognition msgs   
    ros::Time t_ = ros::Time::now();
    object_array.header.frame_id = target_frame;
    object_array.header.stamp = msg->header.stamp;
    // object_array.header.stamp = t_;
    for(size_t i=0;i<cluster_array.size();i++)
    {
        PointCloud2 pc2_cloud;
        pcl::toROSMsg(cluster_array[i].points, pc2_cloud);
        pc2_cloud.header.frame_id = target_frame;
        pc2_cloud.header.stamp = t_;

        amsl_recog_msgs::ObjectInfoWithROI data;
        data.header.frame_id    = target_frame;
        data.header.stamp = t_;
        data.pose.position.x = cluster_array[i].data.x;
        data.pose.position.y = cluster_array[i].data.y;
        data.pose.position.z = cluster_array[i].data.z;
        data.pose.orientation.x = 0;
        data.pose.orientation.y = 0;
        data.pose.orientation.z = 0;
        data.pose.orientation.w = 1;
        data.width  = cluster_array[i].data.width;
        data.height = cluster_array[i].data.height;
        data.depth  = cluster_array[i].data.depth;
        data.points = pc2_cloud;
        object_array.object_array.push_back(data);
    }


	cloud->clear();
    for(int i=0;i<int(cluster_array.size());i++){
		for(int j=0;j<(int(cluster_array[i].points.points.size()));j++){
			cloud->push_back(cluster_array[i].points.points[j]);
		}
	}
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = msg->header.stamp;
    
	pub_points.publish(output);
    pub_cluster.publish(object_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/cloud", 10, pcCallback<PointA, CloudA, CloudAPtr>);

    pub_cluster  = nh.advertise<amsl_recog_msgs::ObjectInfoArray>("/cluster/objectinfo", 10);
    pub_centroid = nh.advertise<PointCloud2>("/cluster/centroid", 1);
    pub_points   = nh.advertise<PointCloud2>("/cluster/points", 1);
    ros::spin();

    return 0;
}
