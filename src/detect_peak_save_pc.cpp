//入ってきたdataに対してpeakを計算する
//savecloud では点群を貯めた状態でのpeakを計算する
//
//

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
//PCL

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
//vizualize
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
//publish_degree
#include <std_msgs/Int32MultiArray.h>
#include <omp.h>

#include <vector>
#include <boost/range/algorithm.hpp>


using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool callback_flag = false;
int devide = 720;
//d-kan
// double threshold = 7.0;
// ikuta
//double threshold = 25.0;
double threshold = 999.9;

ros::Publisher peak_pub;
ros::Publisher marker_pub;
ros::Publisher marker2_pub;
ros::Publisher deg_pub;
ros::Publisher road_pub;

//publish pointcloud
inline void pubPointCloud2(ros::Publisher& pub,
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

inline double calc_distance(PointA& input)
{
    double dist = sqrt(input.x*input.x + input.y*input.y);
    // cout<<"distance:"<<dist<<endl;//for debug

    return dist;
}

void Calc_peak(CloudAPtr peak)
{
    cout<<"peak_detection"<<endl;
    std_msgs::Int32MultiArray set_deg;
    set_deg.data.clear();
    int array_size = 0;

    size_t cloud_size = peak->points.size()-devide/18; //devide/18の存在意義？？？
    size_t certification = 40; //何者？
    int count = 0;
    bool publish_flag = false;


    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/velodyne";
    // line_list.header.stamp = ros::Time::now();
    line_list.header.stamp = ros::Time(0);
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for(size_t i=0;i<cloud_size;i++){ //全点探索
        if(peak->points[i].intensity >=threshold){
            count=0;
            size_t tmp_j = i;
            for(size_t j = tmp_j;j<tmp_j+certification;j++){
                if(peak->points[j].intensity >=threshold){
                    count++;
                    if(count==10){
                        double tmp_i = i;
                        double fif_mean = 0;
                        while(tmp_i<devide){
                            //五点平均が閾値に近い部分を一つのピークとする
                            for(size_t k=tmp_i;k<tmp_i+5;k++){
                                fif_mean += peak->points[k].intensity;
                            }
                            fif_mean = fif_mean/5.0; //5点平均
                            if(fif_mean<=(threshold-3.0)){//detect end of load
                                fif_mean = 0.0;
                                if(tmp_i - i>=10){
                                    cout<<"its a road!!!!:"<<(double)i+(tmp_i-i)/2.0<<endl;
                                    set_deg.data.push_back( i + (int)((tmp_i-i)/2.0) );
                                    array_size++;
                                    geometry_msgs::Point p;
                                    p.x = 0.0;
                                    p.y = 0.0;
                                    p.z = 0.0;
                                    line_list.points.push_back(p);
                                    int p_i = (int)(i + (tmp_i - i)/2.0);
                                    p.x  = peak->points[p_i].x;
                                    p.y  = peak->points[p_i].y;
                                    p.z  = peak->points[p_i].z;
                                    line_list.points.push_back(p);
                                    // marker_pub.publish(line_list);
                                    publish_flag = true;
                                }
                                i = tmp_i+5;
                                j = tmp_i+5;
                                // break;
                            }
                            tmp_i += 1;
                            fif_mean = 0.0;

                            if(tmp_i==devide){
                                if(tmp_i - i>=10){
                                    cout<<"its a road!!!!:"<<(double)i+(tmp_i-i)/2.0<<endl;
                                    set_deg.data.push_back( i + (int)((tmp_i-i)/2.0) );
                                    array_size++;
                                    geometry_msgs::Point p;
                                    p.x = 0.0;
                                    p.y = 0.0;
                                    p.z = 0.0;
                                    line_list.points.push_back(p);
                                    int p_i = (int)(i + (tmp_i - i)/2.0);
                                    p.x  = peak->points[p_i].x;
                                    p.y  = peak->points[p_i].y;
                                    p.z  = peak->points[p_i].z;
                                    line_list.points.push_back(p);
                                    // marker_pub.publish(line_list);
                                    publish_flag = true;
                                }
                                i= tmp_i;
                                j= tmp_i;
                                // break;
                            }
                        }
                    }
                }
            }
        }
    }
    //publish vizualize&load_deg
    if(publish_flag){
        set_deg.layout.data_offset = array_size;
        marker_pub.publish(line_list);
        deg_pub.publish(set_deg);
    }else{
        set_deg.layout.data_offset = 100;
        set_deg.data.push_back(0);
        deg_pub.publish(set_deg);
    }
}

void Calc_road(CloudAPtr peak)
{
    cout<<"road_detection"<<endl;

    std_msgs::Int32MultiArray set_deg;
    set_deg.data.clear();
    int array_size = 0;

    size_t cloud_size = peak->points.size()-devide/18;
    size_t certification = 40;
    int count = 0;
    bool publish_flag = false;


    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/velodyne";
    // line_list.header.stamp = ros::Time::now();
    line_list.header.stamp = ros::Time(0);
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    // line_list.color.r = 0.5;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    for(size_t i=0;i<cloud_size;i++){
        if(peak->points[i].intensity >=threshold){
            count=0;
            size_t tmp_j = i;
            for(size_t j = tmp_j;j<tmp_j+certification;j++){
                if(peak->points[j].intensity >=threshold){
                    count++;
                    // if(count==20){
                    if(count==10){
                    // if(count==15){
                        size_t tmp_i = i;
                        double fif_mean = 0;
                        while(tmp_i<devide){
                            //五点平均が閾値に近い部分を一つのピークとする
                            for(size_t k=tmp_i;k<tmp_i+5;k++){
                                fif_mean += peak->points[k].intensity;
                            }
                            fif_mean = fif_mean/5.0;
                            // cout<<"五点平均"<<fif_mean<<endl;
                            if(fif_mean<=threshold){//detect end of load
                            // if(fif_mean<=(threshold-1.0)){//detect end of load
                                fif_mean = 0.0;
                                // cout<<"start:"<<i<<endl;
                                // cout<<"end:"<<tmp_i<<endl;
                                int road_deg = tmp_i - i;
                                // cout<<road_deg<<endl;
                                // cout<<tmp_i<<endl;
                                // cout<<i<<endl;
                                if(road_deg>=10){
                                    cout<<"road_deg over 10"<<endl;
                                    cout<<road_deg<<endl;
                                    int cnt = 0;
                                    int calc_result = road_deg;
                                    while(calc_result>=5){
                                        calc_result -= 5;
                                        cnt++;
                                    }
                                    calc_result = (int)(road_deg/cnt);
                                    cout<<"calc_result value:"<<calc_result<<endl;
                                    geometry_msgs::Point p;
                                    for(int iii = 1;iii<(cnt+1);iii++){
                                        cout<<"its a road!!!!:"<<(double)i+(iii-1)*calc_result+(calc_result)/2.0<<endl;
                                        set_deg.data.push_back( i + (int)((iii-1)*calc_result+(calc_result)/2.0) );
                                        array_size++;
                                        p.x = 0.0;
                                        p.y = 0.0;
                                        p.z = 0.0;
                                        line_list.points.push_back(p);
                                        int p_i = (int)(i + (iii-1)*calc_result +(calc_result)/2.0);
                                        p.x  = peak->points[p_i].x;
                                        p.y  = peak->points[p_i].y;
                                        p.z  = peak->points[p_i].z;
                                        line_list.points.push_back(p);
                                        // marker_pub.publish(line_list);
                                        publish_flag = true;
                                    }
                                }else if(road_deg>=1000){
                                    cout<<"its a road!!!!:"<<(double)i+(road_deg)/2.0<<endl;
                                    set_deg.data.push_back( i + (int)((road_deg)/2.0) );
                                    array_size++;
                                    geometry_msgs::Point p;
                                    p.x = 0.0;
                                    p.y = 0.0;
                                    p.z = 0.0;
                                    line_list.points.push_back(p);
                                    int p_i = (int)(i + (road_deg)/2.0);
                                    p.x  = peak->points[p_i].x;
                                    p.y  = peak->points[p_i].y;
                                    p.z  = peak->points[p_i].z;
                                    line_list.points.push_back(p);
                                    // marker_pub.publish(line_list);
                                    publish_flag = true;
                                }
                                i = tmp_i+5;
                                j = tmp_i+5;
                                cout<<"tmp_i:"<<tmp_i<<endl;
                                break;
                            }
                            tmp_i += 1;
                            fif_mean = 0.0;

                            if(tmp_i==devide){
                                // cout<<"start:"<<i<<endl;
                                // cout<<"end:"<<tmp_i<<endl;
                                int road_deg = tmp_i - i;
                                // cout<<road_deg<<endl;
                                // cout<<tmp_i<<endl;
                                // cout<<i<<endl;
                                if(road_deg>=10){
                                    cout<<"road_deg over 10"<<endl;
                                    cout<<road_deg<<endl;
                                    int cnt = 0;
                                    int calc_result = road_deg;
                                    while(calc_result>=5){
                                        calc_result -= 5;
                                        cnt++;
                                    }
                                    calc_result = (int)(road_deg/cnt);
                                    geometry_msgs::Point p;
                                    // cout<<"cnt value"<<cnt<<endl;
                                    for(int iii = 1;iii<(cnt+1);iii++){
                                        cout<<"its a road!!!!:"<<(double)i+(iii-1)*calc_result+(calc_result)/2.0<<endl;
                                        set_deg.data.push_back( i + (int)((iii-1)*calc_result+(calc_result)/2.0) );
                                        array_size++;
                                        p.x = 0.0;
                                        p.y = 0.0;
                                        p.z = 0.0;
                                        line_list.points.push_back(p);
                                        int p_i = (int)(i + (iii-1)*calc_result + (calc_result)/2.0);
                                        p.x  = peak->points[p_i].x;
                                        p.y  = peak->points[p_i].y;
                                        p.z  = peak->points[p_i].z;
                                        line_list.points.push_back(p);
                                        // marker_pub.publish(line_list);
                                        publish_flag = true;
                                    }
                                }else if(road_deg>=1000){
                                    cout<<"its a road!!!!:"<<(double)i+(road_deg)/2.0<<endl;
                                    set_deg.data.push_back( i + (int)((road_deg)/2.0) );
                                    array_size++;
                                    geometry_msgs::Point p;
                                    p.x = 0.0;
                                    p.y = 0.0;
                                    p.z = 0.0;
                                    line_list.points.push_back(p);
                                    int p_i = (int)(i + (road_deg)/2.0);
                                    // p.x  = peak->points[(int)(i + (tmp_i - i)/2.0)].x;
                                    // p.y  = peak->points[(int)(i + (tmp_i - i)/2.0)].y;
                                    // p.z  = peak->points[(int)(i + (tmp_i - i)/2.0)].z;
                                    p.x  = peak->points[p_i].x;
                                    p.y  = peak->points[p_i].y;
                                    p.z  = peak->points[p_i].z;
                                    line_list.points.push_back(p);
                                    // marker_pub.publish(line_list);
                                    publish_flag = true;
                                }
                                i= tmp_i;
                                j= tmp_i;
                                // break;
                            }
                        }
                    }
                }
            }
        }
    }
    //publish vizualize&load_deg
    if(publish_flag){
        set_deg.layout.data_offset = array_size;
        marker2_pub.publish(line_list);
        road_pub.publish(set_deg);
    }else{
        set_deg.layout.data_offset = 100;
        set_deg.data.push_back(0);
        road_pub.publish(set_deg);
    }
}
void Calc_threshold(CloudAPtr shape_cloud,double& threshold)
{
    vector<double> v;
    size_t shape_size = shape_cloud->points.size();
    for(size_t i=0;i<shape_size;i++){
        v.push_back(calc_distance(shape_cloud->points[i]));
    }
    boost::sort(v);

    cout<<v[(size_t)shape_size*3/4]<<endl;
    if(v[(size_t)shape_size*3/4]>15.0){
        // threshold = 1.7*v[(size_t)shape_size*3/4];
        threshold = 35.0;
    }else if(v[(size_t)shape_size*3/4]>7.0){
        threshold = 25.0;
    }else{
        threshold = 15.0;
    }

}

void Detect_peak(CloudAPtr shape_cloud)
{
    size_t cloud_size = shape_cloud->points.size();
    size_t tmp_size = cloud_size + devide/18; //tm_sizeの存在意義？

    //閾値を四分位法から推定
    Calc_threshold(shape_cloud,threshold);
    double distance = 0.0;
    CloudAPtr extend_shape(new CloudA);
    //peak detectする前にデータの繰り返しを行いデータの不連続面に対してもpeak detectを行う
    //更にデータに対して最大値を儲ける
    for(size_t i = 0;i<tmp_size;i++){
        extend_shape->push_back(shape_cloud->points[i%cloud_size]); //i>=cloud_sizeとは？
        distance = calc_distance(extend_shape->points[i]);
        if(distance > threshold+3.0){
            // A
            // extend_shape->points[i].intensity = threshold;
            extend_shape->points[i].intensity = threshold+3.0;
        }else{
            extend_shape->points[i].intensity = distance;
        }
    }
    //Peak を検出する
    Calc_peak(extend_shape);
    //road を検出
    threshold = 10.0;
    Calc_road(extend_shape);

}


//callback
CloudAPtr tmp_cloud (new CloudA);
void Shape_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg,*tmp_cloud);
    callback_flag = true;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "detect_peak_save_pc");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/intersection_recognition/detect_shape2", 1, Shape_callback);
    // Create a ROS publisher for the output point cloud
    peak_pub = nh.advertise<sensor_msgs::PointCloud2> ("/intersection_recognition/detect_peak2", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker/long", 10);
    marker2_pub = n.advertise<visualization_msgs::Marker>("visualization_marker/short", 10);
    deg_pub = n.advertise<std_msgs::Int32MultiArray>("/intersection_recognition/peak_deg", 10);
    road_pub = n.advertise<std_msgs::Int32MultiArray>("/intersection_recognition/road_deg", 10);
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
        if(callback_flag){
            Detect_peak(tmp_cloud);
            callback_flag = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
