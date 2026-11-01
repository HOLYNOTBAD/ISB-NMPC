#include "ros/ros.h"
//#include "geometry_msgs/TransformStamped.h"
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
using namespace std;
//using namespace Eigen;

//uwb stands for mocap here
float pos_uwb[3]={0,0,0};
float q_uwb[4]={0,0,0,1};
float mocap_origin[3]={0,0,0};
float position_offset[3]={0,0,0};

string mocap_unit;
string local_origin_topic;
bool zero_position_on_first_pose = true;
bool has_uwb_data = false;
bool has_mocap_origin = false;
double max_abs_position_m = 900.0;

ros::Publisher vision_pub;
ros::Publisher local_origin_pub;

void uwb_to_fcu();
void publish_local_origin();

void cb_uwb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
	geometry_msgs::PoseStamped uwb_data;
	uwb_data = *msg;
	float raw_pos[3]={0,0,0};
	if(mocap_unit == "m")
	{
		raw_pos[0] =  uwb_data.pose.position.x;//m to m
		raw_pos[1] =  uwb_data.pose.position.y;
		raw_pos[2] =  uwb_data.pose.position.z;
	}
	else if(mocap_unit == "mm")
	{
		raw_pos[0] =  uwb_data.pose.position.x/1000;//mm to m
		raw_pos[1] =  uwb_data.pose.position.y/1000;
		raw_pos[2] =  uwb_data.pose.position.z/1000;
	}
	else
	{
		ROS_WARN("[px4mocap] invalid mocap_unit for px4mocap!");
		return;
	}
	if(zero_position_on_first_pose && !has_mocap_origin)
	{
		mocap_origin[0] = raw_pos[0];
		mocap_origin[1] = raw_pos[1];
		mocap_origin[2] = raw_pos[2];
		has_mocap_origin = true;
		ROS_INFO("[px4mocap] captured mocap origin: [%.3f %.3f %.3f] m",
		         mocap_origin[0], mocap_origin[1], mocap_origin[2]);
		publish_local_origin();
	}
	pos_uwb[0] = raw_pos[0] - (zero_position_on_first_pose ? mocap_origin[0] : 0.0f) + position_offset[0];
	pos_uwb[1] = raw_pos[1] - (zero_position_on_first_pose ? mocap_origin[1] : 0.0f) + position_offset[1];
	pos_uwb[2] = raw_pos[2] - (zero_position_on_first_pose ? mocap_origin[2] : 0.0f) + position_offset[2];
	q_uwb[0] = uwb_data.pose.orientation.x;
	q_uwb[1] = uwb_data.pose.orientation.y;
	q_uwb[2] = uwb_data.pose.orientation.z;
	q_uwb[3] = uwb_data.pose.orientation.w;
	has_uwb_data = true;
}

void publish_local_origin()
{
	geometry_msgs::PointStamped origin;
	origin.header.stamp = ros::Time::now();
	origin.header.frame_id = "map";
	origin.point.x = mocap_origin[0];
	origin.point.y = mocap_origin[1];
	origin.point.z = mocap_origin[2];
	local_origin_pub.publish(origin);
}


void uwb_to_fcu()
{
	geometry_msgs::PoseStamped vision;

        vision.pose.position.x = pos_uwb[0];
        vision.pose.position.y = pos_uwb[1];
        vision.pose.position.z = pos_uwb[2];

        vision.pose.orientation.x = q_uwb[0];
        vision.pose.orientation.y = q_uwb[1];
        vision.pose.orientation.z = q_uwb[2];
        vision.pose.orientation.w = q_uwb[3];

	vision.header.stamp = ros::Time::now();
	vision.header.frame_id = "map";
	vision_pub.publish(vision);
	
}



int main(int argc,char **argv)
{
	ros::init(argc,argv,"px4mocap0");
	ros::NodeHandle n("");
	ros::NodeHandle nh("~");
		
	nh.param<string>("mocap_unit", mocap_unit, "m");
	nh.param<string>("local_origin_topic", local_origin_topic, "/local_frame/origin");
	nh.param<bool>("zero_position_on_first_pose", zero_position_on_first_pose, true);
	nh.param<double>("max_abs_position_m", max_abs_position_m, 900.0);
	nh.param<float>("position_offset_x", position_offset[0], 0.0f);
	nh.param<float>("position_offset_y", position_offset[1], 0.0f);
	nh.param<float>("position_offset_z", position_offset[2], 0.0f);
	ROS_INFO("[px4mocap] mocap_unit: %s",mocap_unit.c_str());
	ROS_INFO("[px4mocap] zero_position_on_first_pose: %s, position_offset=[%.3f %.3f %.3f] m",
	         zero_position_on_first_pose ? "true" : "false",
	         position_offset[0], position_offset[1], position_offset[2]);
	
	ros::Subscriber uwb_sub = n.subscribe("rigidpose_topic",10,cb_uwb);
	vision_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
	local_origin_pub = n.advertise<geometry_msgs::PointStamped>(local_origin_topic, 1, true);

	ros::Rate loop_rate(30); //50Hz 30-50Hz is suitable

	int count = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		if(!has_uwb_data)
		{	if(count == 1) {ROS_INFO("[px4mocap] no data from mocap!");}
		}
		else if( std::fabs(pos_uwb[0])>max_abs_position_m ||
		         std::fabs(pos_uwb[1])>max_abs_position_m ||
		         std::fabs(pos_uwb[2])>max_abs_position_m )
		{
			if(count == 1) {ROS_INFO("[px4mocap] mocap pos too large! Maybe lost the track.");}
		}
		else
		{
			uwb_to_fcu();
		}
		loop_rate.sleep();
		
		count = count + 1;
		if(count==180) {count = 0;}
	
	}  // Cannot publish from inside the callback -- vel_pub is only in scope in main().（无法在回调函数里发布话题，vel_pub作用域仅在main内）

	return 0;
}
