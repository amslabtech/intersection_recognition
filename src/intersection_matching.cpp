#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
//PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
//
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include "lf/node_edge_manager.h"
#include "mmath/binarion.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

typedef pcl::PointXYZI PointX;
typedef pcl::PointCloud<PointX> CloudX;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;

ros::Publisher flag_pub;
ros::Publisher branch_pub;
ros::Publisher yaw_pub;
ros::Publisher mode_pub;

visualization_msgs::MarkerArray arrows;
std_msgs::Bool inter_flag;

bool callback_flag = false;
bool odom_callback = false;
bool update_node_flag = false;
bool isfirst = true;
bool start_flag = false;
bool turn_flag = false;

const double rad2deg = 180/M_PI;
const int loop_count = 10;
const size_t accum_num = 30;
// int loop_count = 20;
int degree_count = 720;

int save_number = loop_count*degree_count;
int save_count = 0;
int target_node = 0;
int turn_cnt = 0;

double node_x = -0.995471;
double node_y = -14.6942;
double node_len = sqrt(pow(node_x,2)+pow(node_y,2));
double x_init = 0.0;
double y_init = 0.0;
int allow_error = 0;

NodeEdgeManager* nem;
// int save_peak[10*720];
int save_peak[loop_count*4];
vector<int> ignore_node;

inline void INIT()
{
	cout<<"INIT"<<endl;
	for(int i=0;i<loop_count*4;i++){
		save_peak[i] = 0;
	}
}

inline bool intersection_traj_flag(geometry_msgs::Pose &odom,geometry_msgs::Pose &intersec_odom)
{
	// double difference = abs( sqrt( pow(odom.position.x,2)+pow(odom.position.y,2) ) - sqrt( pow(intersec_odom.position.x,2)+pow(intersec_odom.position.y,2) )  );
	double difference = fabs(sqrt(pow(odom.position.x-intersec_odom.position.x,2)+pow(odom.position.y-intersec_odom.position.y,2)));

	/* cout<<"difference="<<difference<<endl; */
	// if(difference>1.0){
	if(difference>3.0){
	/* if(isfirst || difference>3.0 || target_node==7 || target_node==58 || target_node==70){ */
	/* if(isfirst || difference>6.0){ */
		cout<<"True"<<endl;
		return true;
	}else{
		cout<<"False"<<endl;
		return false;
	}
}

int cnt_degree[360];
int loop_tra = 0;
bool unliner_flag = false;

void Init_deg()
{
	cout<<"Init_degree"<<endl;
	for(int i=0;i<360;i++){
		cnt_degree[i]=0;
	}
}


void Intersec_init()
{
	INIT();
	loop_tra=0;
	Init_deg();
}

//callback
std_msgs::Int32MultiArray peak_in;
void Peak_deg(const std_msgs::Int32MultiArray::Ptr &msg)
{
	peak_in.data.clear();
	peak_in.layout.data_offset = msg->layout.data_offset;
	if(peak_in.layout.data_offset!=100){
		for(unsigned int i=0;i<peak_in.layout.data_offset;i++){
			peak_in.data.push_back(msg->data[i]);
		}
		callback_flag = true;
		// cout<<"peak_callback"<<endl;
	}
}

geometry_msgs::Pose odom_l;
geometry_msgs::Pose intersec_odom;
Binarion yaw_ave;
int cnt = 0;
/* double yaw_ave = 0.0; */
void OdomCallback(const nav_msgs::OdometryConstPtr& input){
	odom_l.position.x = input->pose.pose.position.x;
	odom_l.position.y = input->pose.pose.position.y;
	odom_l.position.z = input->pose.pose.position.z;
	odom_l.orientation.z = input->pose.pose.orientation.z;
	// cout<<"odom_callback"<<endl;
	/* yaw_list.push_back(odom.orientation.z); */
	/* if(yaw_list.size() > accum_num){ */
	/* 	yaw_list.erase(yaw_list.begin()); */
	/* 	double sum = accumulate(yaw_list.begin(), yaw_list.end(), 0.0); */
	/* 	yaw_ave = sum/yaw_list.size(); */
	/* } */
	Binarion tmp(odom_l.orientation.z);
	yaw_ave = yaw_ave.slerp(tmp, cnt/(1.0+cnt));
	cnt++;
	cnt %= accum_num;

	if(intersec_odom.position.x==0&&intersec_odom.position.y==0){
		intersec_odom.position.x = odom_l.position.x;
		intersec_odom.position.y = odom_l.position.y;
		intersec_odom.position.z = odom_l.position.z;
	}
	odom_callback = true;
}

geometry_msgs::Pose gps_odom;
bool gps_flag = false;
void GpsCallback(const nav_msgs::OdometryConstPtr& msg)
{
	if(x_init == 0.0 && y_init == 0.0){
		x_init = odom_l.position.x;
		y_init = odom_l.position.y;
	}
	gps_odom.position.x = msg->pose.pose.position.x + x_init;
	gps_odom.position.y = msg->pose.pose.position.y + y_init;
	gps_odom.orientation.z = yaw_ave.getYaw();
	/* cout<<"gps_odom:"<<gps_odom<<endl; */

	gps_flag = true;
}

Node node[3];
Edge_c edge_new;
int edge_now = 0;
int node_next = 0;
bool edge_flag = false;
void divCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	node[0] = nem->nodeGetter(msg->data[0]);
	node[1] = nem->nodeGetter(msg->data[1]);
	node_next = msg->data[1];
	edge_now = nem->edgeGetter(msg->data[0],msg->data[1]);
	/* edge_len = nem->distGetter(msg->data[0],msg->data[1]); */
	/* cout<<"NEXT=>"<<node_next<<endl; */
	double ratio =  0.01 * msg->data[2];
	node[2].x = node[0].x + ratio * (node[1].x - node[0].x);
	node[2].y = node[0].y + ratio * (node[1].y - node[0].y);
	/* if(isfirst || next_flag){ //初回または前回の交差点を10m以上過ぎたら */
	/* 	edge_new = nem->edgeGetter(edge_now,odom.orientation.z); //2018/09/12 */
	/* } */
	/* if(!isfirst){ */
	edge_flag = true;
	/* } */
}

int edge_next = 0;
bool local_flag = false;
Node node_l[2];
void Next_node(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	/* pre_target = msg->data[0]; */
	target_node = msg->data[1];
	node_l[0] = nem->nodeGetter(msg->data[0]);
	node_l[1] = nem->nodeGetter(msg->data[1]);
	edge_next = nem->edgeGetter(msg->data[0],msg->data[1]);
	local_flag = true;
}

void startCallback(const std_msgs::Bool::ConstPtr& msg)
{
	start_flag = msg->data;
}

void turnCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if((target_node == 63) && (turn_cnt < 1000)){
		turn_flag = msg->data;
		if(turn_flag) turn_cnt++;
		else if(turn_cnt != 0) turn_cnt = 2000;
	}
}


/* const float thresh = 0.9; */
const float thresh = 0.85;
double edge_len = 0;
double n_len = 0;
inline bool detect_flag()
{
	double nx = 0.0, ny = 0.0;
	if(local_flag){
		nx = node_l[0].x;
		ny = node_l[0].y;
	}
	else{
		nx = node[0].x;
		ny = node[0].y;
	}
	/* double nx = intersec_odom.position.x; */
	/* double ny = intersec_odom.position.y; */
	/* edge_len = sqrt(pow(node[1].x-nx,2)+pow(node[1].y-ny,2)); */
	if(local_flag) edge_len = sqrt(pow(node_l[1].x-node_l[0].x,2) + pow(node_l[1].y-node_l[0].y,2));
	else edge_len = sqrt(pow(node[1].x-node[0].x,2) + pow(node[1].y-node[0].y,2));
	/* if(local_flag) edge_len = nem->distGetter(node_l[0], node_l[1]);	 */
	/* else edge_len = nem->distGetter(node[0], node[1]);; */

	n_len = sqrt(pow(odom_l.position.x-nx, 2) + pow(odom_l.position.y-ny, 2));

	/* if(gps_flag){ */
	/* 	n_len = sqrt(pow(gps_odom.position.x-nx,2)+pow(gps_odom.position.y-ny,2)); */
	/* } */
	/* else{ */
	/* 	n_len = sqrt(pow(odom_l.position.x-nx,2)+pow(odom_l.position.y-ny,2)); */
	/* } */

	/* cout<<"n_len:"<<n_len<<" edge_len:"<<edge_len<<endl; */
	if(isfirst || n_len>edge_len*thresh){
		cout<<"detect mode!!!!!"<<endl;
		return true;
	}
	else{
		cout<<"not detect"<<endl;
		return false;
	}
}

void setIgnoreNode(string file)
{
	ifstream ifs(file);
	string str;

	if(ifs.fail()){
		cerr << "can't read " << file << endl;
		return;
	}

	while(getline(ifs, str)){
		if(!str.empty()){
			ignore_node.push_back(atoi(str.c_str()));
		}
	}
}


// bool inter_flag = false;
int inter_cnt = 0;
int gps_cnt = 0;
int bfr_num = 0;
// bool goal_flag = false;
/* double dist = 0.0, n_x = 0.0, n_y = 0.0; */
double n_x = 0.0, n_y = 0.0;
void shape_matching(Edge_c new_e, geometry_msgs::Pose odom, std_msgs::Int32MultiArray peak)
{
	inter_flag.data = false;
	int cnt0=0, cnt1=0;
	double tmp_rad = 0.0;
	/* for(size_t i = 0; i<ignore_node.size(); i++){ */
	/* 	if(ignore_node[i] == node_next || ignore_node[i] == target_node){ */
	/* 		return; */
	/* 	} */
	/* } */

	/* cout<<"shape matching"<<endl; */
	if(yaw_ave.getYaw() < 0) tmp_rad = yaw_ave.getYaw() + M_PI;
	else tmp_rad = yaw_ave.getYaw() - M_PI;
	if(callback_flag){
		for(size_t i=0; i<new_e.orientation.size(); i++){
			if(fabs(Binarion::deviation(tmp_rad, new_e.orientation[i])*rad2deg) < 15){
				cnt0++;
			}
			else{
				double p_rad = 0.0;
				double e_rad = 0.0;
				for(size_t j=0; j<peak.data.size(); j++){
					p_rad = M_PI * peak.data[j]/360 + yaw_ave.getYaw() - M_PI;
					e_rad = new_e.orientation[i];
					if(p_rad> M_PI) p_rad = p_rad - 2*M_PI;
					if(p_rad<-M_PI) p_rad = p_rad + 2*M_PI;
					/* cout<<"Edge="<<e_rad*rad2deg<<",Peak="<<p_rad*rad2deg<<",j="<<j<<endl; */

					if(fabs(Binarion::deviation(e_rad, p_rad)*rad2deg) < 12){
					/* if(fabs(Binarion::deviation(e_rad, p_rad)*rad2deg) < 10){ */
						cnt1++;
						break;
					}
				}
			}
		}
	}
	/* if(callback_flag && peak.data.size()<=new_e.orientation.size() && (cnt0+cnt1)==(int)new_e.orientation.size()){ */
	if(callback_flag && (cnt0+cnt1)==(int)new_e.orientation.size()){
		inter_cnt++;
		bool ignore_flag = false;
		switch(target_node){
			case 66:
			case 67:
			case 68:
			case 69:
				ignore_flag = true;
				break;
		}

		if(inter_cnt >= 3 && !ignore_flag){
			  /*bfr_num = node_next;
			  cout<<"bfr:"<<bfr_num<<"now:"<<new_e.num<<endl; */
			inter_flag.data = true;
			isfirst = false;
			// goal_flag = true;
			inter_cnt = 0;
			// intersec_odom.position.x = odom.position.x;
			// intersec_odom.position.y = odom.position.y;
			// intersec_odom.position.z = odom.position.z;
			if(local_flag){
				intersec_odom.position.x = node_l[1].x;
				intersec_odom.position.y = node_l[1].y;
				intersec_odom.position.z = odom_l.position.z;
			}
			else{
				intersec_odom.position.x = node[1].x;
				intersec_odom.position.y = node[1].y;
				intersec_odom.position.z = odom_l.position.z;
			}
			cout<<"-------------!!!INTERSECTION!!!----------------"<<endl;
		}
	}
	else if(gps_flag){
		if(local_flag){
			// n_x = nem->nodeGetter(target_node).x;
			// n_y = nem->nodeGetter(target_node).y;
			n_x = node_l[1].x;
			n_y = node_l[1].y;
		}
		else if(edge_flag){
			n_x = node[1].x;
			n_y = node[1].y;
		}
		/* dist = sqrt(pow(gps_odom.position.x - n_x, 2) + pow(gps_odom.position.y - n_y, 2)); */
		/* int ugps_nodes[4] = {68, 15, 69, 28}; */
		bool ugps_flag = false;
		Eigen::Vector2d v, edge_v;
		/* for(int i=0; i<4; i++){ */
		/* 	if(target_node == ugps_nodes[i]){ */
		/* 		ugps_flag = true; */
		/* 		break; */
		/* 	} */
		/* } */
		switch(target_node){
			case 68:
			case 15:
			case 69:
			case 28:
				ugps_flag = true;
				break;
		}

		if(ugps_flag){
			v(0) = odom.position.x - node_l[0].x;
			v(1) = odom.position.y - node_l[0].y;
			cout<<"unuse GPS!!!!!"<<endl;
		}
		else{
			v(0) = gps_odom.position.x - node_l[0].x;
			v(1) = gps_odom.position.y - node_l[0].y;
		}
		edge_v(0) = node_l[1].x - node_l[0].x;
		edge_v(1) = node_l[1].y - node_l[0].y;
		float dist = edge_v.norm() - fabs(v.dot(edge_v) / edge_v.norm());

		cout<<"difference="<<dist<<endl;
		if(dist < 1.3){
			gps_cnt++;
			if(gps_cnt > 5){
				// bfr_num = node_next;
				/* cout<<"bfr:"<<bfr_num<<"now:"<<new_e.num<<endl; */
				inter_flag.data = true;
				isfirst = false;
				// goal_flag = true;
				inter_cnt = 0;
				gps_cnt = 0;
				intersec_odom.position.x = gps_odom.position.x;
				intersec_odom.position.y = gps_odom.position.y;
				// intersec_odom.position.x = n_x;
				// intersec_odom.position.y = n_y;
				intersec_odom.position.z = odom_l.position.z;
				cout<<"-------------!!!GPS INTERSECTION!!!----------------"<<endl;
			}
		}
		/* else if(sqrt(pow(odom.position.x - n_x, 2) + pow(odom.position.y - n_y, 2)) < 1.5){ */
		/* 	// bfr_num = node_next; */
		/* 	#<{(| cout<<"bfr:"<<bfr_num<<"now:"<<new_e.num<<endl; |)}># */
		/* 	inter_flag.data = true; */
		/* 	isfirst = false; */
		/* 	goal_flag = true; */
		/* 	inter_cnt = 0; */
		/* 	intersec_odom.position.x = gps_odom.position.x; */
		/* 	intersec_odom.position.y = gps_odom.position.y; */
		/* 	// intersec_odom.position.x = n_x; */
		/* 	// intersec_odom.position.y = n_y; */
		/* 	intersec_odom.position.z = odom.position.z; */
		/* 	cout<<"-------------!!!Around INTERSECTION!!!----------------"<<endl; */
		/* } */
	}
}


int old = 0;
void show_branch_direction(Edge_c new_e, geometry_msgs::Pose odom)
{
	visualization_msgs::Marker vec;
	
	vec.header.stamp = ros::Time::now();
	vec.header.frame_id = "/map";
	vec.ns = "vec";
	vec.type = visualization_msgs::Marker::ARROW;
	vec.action = visualization_msgs::Marker::ADD;
	vec.scale.x = 2.0;
	vec.scale.y = 0.3;
	vec.scale.z = 0.2;
	vec.color.r = 1.0;
	vec.color.g = 1.0;
	vec.color.a = 1.0;
	vec.lifetime = ros::Duration(0.5);
	
	if(inter_flag.data){
		vec.color.r = 1.0;
		vec.color.g = 0.0;
		vec.color.b = 0.3;
	}

	arrows.markers.clear();
	for(size_t i=0; i< new_e.orientation.size(); i++){
		vec.pose.position.x = odom.position.x;
		vec.pose.position.y = odom.position.y;
		// geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(new_e.orientation[i]+odom.orientation.z);
		double tmp_yaw = new_e.orientation[i];
		/* if(tmp_yaw> M_PI) tmp_yaw = tmp_yaw - 2*M_PI; */
		/* if(tmp_yaw<-M_PI) tmp_yaw = tmp_yaw + 2*M_PI; */

		geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(tmp_yaw);
		vec.pose.orientation.x = quat.x;
		vec.pose.orientation.y = quat.y;
		vec.pose.orientation.z = quat.z;
		vec.pose.orientation.w = quat.w;
		vec.id = i;
		arrows.markers.push_back(vec);
		/* if(new_e.orientation[i]<0){ */
		/* 	new_e.orientation[i] += 2*M_PI; */
		/* } */
		/* cout<<"next_yaw ="<<new_e.orientation[i]*rad2deg<<endl; */
	}
}

double target_setter(int nextedge, double yaw, std_msgs::Int32MultiArray peak)
{
	double target = nem->yawGetter(nextedge);
	double diff_min = 100;
	double tmp = 0;

	if(callback_flag && target_node != 13){
		for(unsigned int i=0;i<peak_in.layout.data_offset;i++){
			double p_rad = M_PI * peak.data[i]/360 + yaw - M_PI;
			/* cout<<"Peak:"<<p_rad<<endl; */
			if(p_rad> M_PI) p_rad = p_rad - 2*M_PI;
			if(p_rad<-M_PI) p_rad = p_rad + 2*M_PI;
			double diff = fabs(Binarion::deviation(target, p_rad)*rad2deg);
			/* cout<<"diff:"<<diff<<endl; */
			if(diff < 20 && diff < diff_min){
			// if(diff < diff_min){
				diff_min = diff;
				tmp = p_rad;
			}		
		}
		if(tmp != 0) target = tmp;
	}
	/* cout<<"TARGET:"<<target<<endl; */
	return target;
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "intersection_shape_matching");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/peak/deg", 1, Peak_deg);
    ros::Subscriber sub_local = nh.subscribe ("/local_node", 1, Next_node);
    // ros::Subscriber sub = nh.subscribe ("/peak/deg2", 1, Peak_deg);
	ros::Subscriber sub_div = n.subscribe<std_msgs::Int16MultiArray>("/edge/certain", 1, divCallback);
	ros::Subscriber sub_lcl = n.subscribe("/lcl",1,OdomCallback);
	ros::Subscriber sub_gps = n.subscribe("/odom/gps",1,GpsCallback);
	ros::Subscriber sub_start = n.subscribe("/start",1,startCallback);
	ros::Subscriber sub_turn = n.subscribe("/turn",1,turnCallback);
    // Create a ROS publisher for the output point cloud
    flag_pub = nh.advertise<std_msgs::Bool> ("/intersection_flag", 1);
    yaw_pub = nh.advertise<std_msgs::Float64> ("/target_yaw", 1);
    branch_pub = nh.advertise<visualization_msgs::MarkerArray> ("/next_branch", 10);
	mode_pub = nh.advertise<std_msgs::Int16>("/mode",1);


	//init_param
	cout<<"init_params"<<endl;
	string filename;
	int begin_node;
	int end_node;
	double div;
	n.getParam("/node_edge", filename);
	n.getParam("/init/node/begin", begin_node);
	n.getParam("/init/node/end", end_node);
	n.getParam("/init/node/div", div);
	nem = new NodeEdgeManager(filename);

	string ignore_file;
	n.getParam("/ignore_node", ignore_file);
	setIgnoreNode(ignore_file);

	/* cout<<"begin_node"<<begin_node<<endl; */
	/* cout<<"end_node"<<end_node<<endl; */
	//end init_param

	int edge_num = nem->edgeGetter(begin_node,end_node);
	node_len = sqrt(nem->distGetter(edge_num)) * (1-div);
	Node node = nem->nodeGetter(end_node);
	edge_new = nem->edgeGetter(edge_num, odom_l.orientation.z);
	node_x = node.x;
	node_y = node.y;

	intersec_odom.position.x = 0.0;
	intersec_odom.position.y = 0.0;
	intersec_odom.position.z = 0.0;
	//init
	// for(int i=0;i<loop_count*4;i++){
	// 	save_peak[i] = 0;
	// }
	INIT();
	Init_deg();
	cout<<"init"<<endl;
	//
	// geometry_msgs::Pose intersec_odom;
	std_msgs::Int32MultiArray peak_global;
	std_msgs::Float64 target;
	std_msgs::Int16 mode;
	geometry_msgs::Pose odom;

    // main handle
    /* ros::Rate loop_rate(20); */
    ros::Rate loop_rate(10);
	mode.data = 0;

    while (ros::ok()){
		if(gps_flag) odom = gps_odom;
		else odom = odom_l;
		/* branch_pub.publish(arrows); */
		/* show_branch_direction(edge_new,odom);//2018/09/12 */
		// callback_flag = true;
		if(odom_callback){
			/* if(odom_callback){ //2018/09/13 */
			update_node_flag = detect_flag();

			if(local_flag) edge_new = nem->edgeGetter(edge_next,odom_l.orientation.z); //2018/09/12
			else  edge_new = nem->edgeGetter(edge_now,odom_l.orientation.z); //2018/09/12
			/* else if(edge_flag) edge_new = nem->edgeGetter(edge_now,odom.orientation.z); //2018/09/12 */
			// Edge_c edge_new = nem->edgeGetter(node_next,odom.orientation.z); //2018/09/12
			//
			show_branch_direction(edge_new,odom_l);//2018/09/12
			branch_pub.publish(arrows);
			if(local_flag){
				/* target.data = nem->yawGetter(edge_next); */
				/* // yaw_pub.publish(target); */
				/* if(local_flag && edge_now != edge_next && goal_flag){ */
				// if( goal_flag){
				target.data = target_setter(edge_next, yaw_ave.getYaw(), peak_in);
				// goal_flag = false;
					// yaw_pub.publish(target);
				// }
			}

			inter_flag.data = false;
			if(intersection_traj_flag(odom,intersec_odom) && update_node_flag){/////change here
				shape_matching(edge_new,odom_l,peak_in);
			}


			if(inter_flag.data){
				// mode.data = 1;
				switch(target_node){
					case 13:
					case 66:
					case 67:
					case 68:
					case 69:
					case 38:
					case 70:
					case 65:
						mode.data = 3;
						break;

					default:
						mode.data = 1;
						break;
				}
				/* if(target_node==13){ */
				/* 	mode.data = 3; */
				/* } */
				/* else{ */
				/* 	mode.data = 1; */
				/* } */
			}
			else{
				if(mode.data == 3){
					mode.data = 3;
				}
				else mode.data = 0;
			}

			if(turn_flag){
				mode.data = 1;
			}
			else if(turn_cnt == 2000){
				mode.data = 3;
				turn_cnt += 100;
			}

			if(start_flag) mode.data = 0;

			// flag_pub.publish(inter_flag);
		}
		callback_flag = false;
		odom_callback = false;
		local_flag = false;
		edge_flag = false;
		start_flag = false;
		turn_flag = false;
		/* cout<<"inter_flag:"<<inter_flag.data<<",mode:"<<mode.data<<endl; //debug */
		yaw_pub.publish(target);
		mode_pub.publish(mode);
		flag_pub.publish(inter_flag);
		ros::spinOnce();
		loop_rate.sleep();
    }
}
