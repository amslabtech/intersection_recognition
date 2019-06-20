/*

PickUp Cluster ( Human and Car)

Publish
    pub_bbox        : jsk_visualize
    pub_cluster     : cluster
Subscribe
    sub : amsl_recog_msgs

Causion:
    using original msg(amsl_recog_msgs, jsk_recognition_msgs)

*/

# include <ros/ros.h>
# include <pcl_ros/point_cloud.h>
# include <pcl/point_types.h>
# include <pcl/point_cloud.h>
# include <pcl/point_types.h>

# include <amsl_recog_msgs/ObjectInfoWithROI.h>
# include <amsl_recog_msgs/ObjectInfoArray.h>

using namespace std;

template<typename T_p>
class PickUp{
    public:
        PickUp();

    private:
        ros::NodeHandle nh;

        ros::Publisher pub_cluster;
        ros::Publisher pub_points;

        ros::Subscriber sub_cluster;

        void cluster_callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr& msg);

        double max_width, max_height, max_depth;
        double min_width, min_height, min_depth;

};

template<typename T_p>
PickUp<T_p>::PickUp()
    : nh("~")
{
    sub_cluster = nh.subscribe("/cluster/objectinfo", 10, &PickUp::cluster_callback, this);

    pub_cluster  = nh.advertise<amsl_recog_msgs::ObjectInfoArray>("/cluster/objectinfo/pickup", 10);
    pub_points   = nh.advertise<sensor_msgs::PointCloud2>("/cluster/points/pickup", 1);

    nh.param<double>("max_width" , max_width,  1.5);//1.5
    nh.param<double>("max_height", max_height, 1.9);//2.0
    nh.param<double>("max_depth",  max_depth,  1.5);//1.5
    nh.param<double>("min_width" , min_width,  0.2);//0.15
    nh.param<double>("min_height", min_height, 1.0);//0.8
    nh.param<double>("min_depth",  min_depth,  0.2);//0.15
}

template<typename T_p>
void PickUp<T_p>::cluster_callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr& msg)
{
    std::cout<<"size:"<<msg->object_array.size()<<std::endl;

    ros::Time time = ros::Time::now();
    amsl_recog_msgs::ObjectInfoArray object_array;

    object_array.header.frame_id = msg->header.frame_id;
    // object_array.header.stamp = time;
    object_array.header.stamp = msg->header.stamp;


    typename pcl::PointCloud<T_p>::Ptr cloud(new pcl::PointCloud<T_p>);
    typename pcl::PointCloud<T_p>::Ptr clouds(new pcl::PointCloud<T_p>);
    for(size_t i=0;i<msg->object_array.size();i++)
    {
        amsl_recog_msgs::ObjectInfoWithROI cluster = msg->object_array[i];

        if(min_width  < cluster.width   && cluster.width  < max_width &&
           min_height < cluster.height  && cluster.height < max_height &&
           min_depth  < cluster.depth   && cluster.depth  < max_depth)
        {
            amsl_recog_msgs::ObjectInfoWithROI data;
            data.header.frame_id = cluster.header.frame_id;
            data.header.stamp = time;
            data.pose   = cluster.pose;
            data.width  = cluster.width;
            data.height = cluster.height;
            data.depth  = cluster.depth;
            data.points = cluster.points;
            object_array.object_array.push_back(data);

			
    		pcl::fromROSMsg(cluster.points, *cloud);
			for(int j=0;j<(int(cloud->points.size()));j++){
				clouds->points.push_back(cloud->points[j]);
			}
        }
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*clouds, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = time;

    pub_cluster.publish(object_array);
    pub_points.publish(output);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pickup_cluster");

    PickUp<pcl::PointXYZI> pu;

    ros::spin();

    return 0;
}
