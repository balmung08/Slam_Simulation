#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <common/public.h>
#include "nav_msgs/Path.h"

class SpeedCtr
{
private:
	ros::NodeHandle n;
	ros::Publisher velcmd_pub, path_pub, marker_pub;
	ros::Subscriber surface_sub, gps_sub, posture_sub, actturnangle_sub;

	float car_length, front_distance_max, front_distance_min, rear_distance_max, rear_distance_min;
	float angle_threshold, max_speed, min_speed;
	double surface_value;

	TDataFilter *df_cmd;
	TTimer spd_tmr; //  车速计时器

	vector<float> vel_cmd_buf;
	geometry_msgs::Twist vel_cmd;
	float surface_speed, posture_speed, turn_speed;
	bool IsMoving;
	double car_distance;

	geometry_msgs::Point last_p_map;

	vector<geometry_msgs::Point> surface_points;

	void SurfacePointCallback(const nav_msgs::Path::ConstPtr &msg); // baselink坐标系
	void GPSDataCallback(const visualization_msgs::Marker::ConstPtr &msg);
	// void GPSDataCallback(const geometry_msgs::Pose::ConstPtr &msg);
	void PostureCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
	void ActTurnAngleCallback(const std_msgs::Float32::ConstPtr &msg);

	void AddSurfacePoint(geometry_msgs::Point);
	void ShowPath();
	void ShowSpeed(double v);
	void CalCurVelCmd();
	float GetSpdBySurface(const double surface);
	void SpeedContrl();

public:
	SpeedCtr();
};

SpeedCtr::SpeedCtr()
{
	ros::NodeHandle nh("~");
	nh.param<float>("car_length", car_length, 2.0);
	nh.param<float>("front_distance_max", front_distance_max, 1.5);
	nh.param<float>("front_distance_min", front_distance_min, 0.0);
	nh.param<float>("rear_distance_max", rear_distance_max, -1);
	nh.param<float>("rear_distance_min", rear_distance_min, -2);
	nh.param<float>("angle_threshold", angle_threshold, 4);
	nh.param<float>("max_speed", max_speed, 2.0);
	nh.param<float>("min_speed", min_speed, 0.5);
	nh.param<double>("surface_value", surface_value, 0.001);
	// surface_sub = n.subscribe<geometry_msgs::Point>("surface_point", 10, &SpeedCtr::SurfacePointCallback, this);
	surface_sub = n.subscribe<nav_msgs::Path>("surface_point", 10, &SpeedCtr::SurfacePointCallback, this);
	gps_sub = n.subscribe<visualization_msgs::Marker>("/car_marker", 10, &SpeedCtr::GPSDataCallback, this);
	// gps_sub = n.subscribe<geometry_msgs::Pose>("car_pose", 10, &SpeedCtr::GPSDataCallback, this);
	posture_sub = n.subscribe<std_msgs::Float32MultiArray>("posture", 10, &SpeedCtr::PostureCallback, this);
	actturnangle_sub = n.subscribe<std_msgs::Float32>("act_turn_angle", 10, &SpeedCtr::ActTurnAngleCallback, this);

	velcmd_pub = n.advertise<geometry_msgs::Twist>("vel_cmd", 10);

	path_pub = n.advertise<nav_msgs::Path>("myPath", 1, true);
	marker_pub = n.advertise<visualization_msgs::Marker>("textSpeed", 10);

	df_cmd = new TDataFilter(10);

	IsMoving = false;

	last_p_map.x = last_p_map.y = last_p_map.z = 0;
	surface_speed = posture_speed = turn_speed = max_speed;
}

void SpeedCtr::PostureCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	float angle_x = msg->data.at(0);
	float angle_y = msg->data.at(1);
	// cout << angle_x << endl;
	if (fabs(angle_x) >= angle_threshold)
		posture_speed = min_speed;
	else
		posture_speed = max_speed;
}

void SpeedCtr::ActTurnAngleCallback(const std_msgs::Float32::ConstPtr &msg)
{
	float act_turn_angle = msg->data;
}

void SpeedCtr::GPSDataCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
	geometry_msgs::Point px;
	px.x = msg->pose.position.x;
	px.y = msg->pose.position.y;
	px.z = 0;

	// IsMoving = (msg->pose.position.z > 0.002);
	// if (!IsMoving)
	//     return;

	// if (fabs(last_p_map.x) + fabs(last_p_map.y) < 0.001) //  第一次
	// {
	//     last_p_map = px;
	//     return;
	// }

	float ds = P2P(px, last_p_map);
	if (ds < 0.10)
		return;
	last_p_map = px;

	if (surface_points.empty())
		return;
	for (auto it = surface_points.begin(); it != surface_points.end(); ++it)
	{	
		it->x -= ds;
		if (it->x < -car_length)
			surface_points.erase(it);
	}
	// cout << "-----------ds = " <<  ds << endl;
}

void SpeedCtr::ShowPath()
{
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "base_link";
	for (auto it = surface_points.begin(); it != surface_points.end(); ++it)
	{
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.pose.position.x = it->x;
		pose_stamped.pose.position.z = it->z / 100;
		pose_stamped.pose.position.y = 0;

		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.header.frame_id = path.header.frame_id;
		path.poses.push_back(pose_stamped);
		// if(it->z>0.1) printf("(x=%.2f z=%.1f) ", it->x, it->z);
	}
	path_pub.publish(path);
}

void SpeedCtr::AddSurfacePoint(geometry_msgs::Point p)
{
	if (surface_points.empty())
		surface_points.push_back(p);
	else
	{
		bool find = false;
		for (auto it = surface_points.begin(); it != surface_points.end(); ++it)
			if (fabs(it->x - p.x) < 0.05)
			{
				*it = p;
				find = true;
				break;
			}

		if (!find)
			surface_points.push_back(p);

		if (surface_points.size() > 10000)
			surface_points.erase(surface_points.begin());

		sort(surface_points.begin(), surface_points.end(), [](geometry_msgs::Point p1, geometry_msgs::Point p2)
			 { return p1.x < p2.x; });
	}
	// ROS_INFO("%d", surface_points.size());
}

void SpeedCtr::SurfacePointCallback(const nav_msgs::Path::ConstPtr &msg)
{
	// geometry_msgs::Point p;
	//  p.x = msg->x;                  //  前向位置
	//  p.y = GetSpdBySurface(msg->z); //  根据崎岖度获取速度
	//  p.z = msg->z;                  //  该位置的崎岖度
	for (int i = 0; i < msg->poses.size(); ++i)
	{
		geometry_msgs::Point p;
		p.x = msg->poses[i].pose.position.x;
		p.z = msg->poses[i].pose.position.z;
		p.y = GetSpdBySurface(p.z);
		// cout << " ****p.z = " <<p.z << endl;
		if(surface_points.empty()) surface_points.push_back(p);
		else {
			surface_points.push_back(p);
		}
		// cout << surface_points.size() << endl;

		// ShowSpeed(p.z);
		// if (i == 0)
		// 	surface_points.push_back(p);
		// else
		// 	AddSurfacePoint(p);
		// cout << "p.x = " << p.x << " p.z = " << p.z << endl;
	}
	if (surface_points.size() > 10000)
			surface_points.erase(surface_points.begin());
	// if (surface_points.empty()) return;
	sort(surface_points.begin(), surface_points.end(), [](geometry_msgs::Point p1, geometry_msgs::Point p2)
			 { return p1.x < p2.x; });

	// AddSurfacePoint(p);
	ShowPath();
	SpeedContrl();
	

	// if (surface_points.empty())  surface_points.push_back(p);
	// else
	// {
	//     if (IsMoving) surface_points.push_back(p);
	//     if (surface_points.size() > 1000)
	//         surface_points.erase(surface_points.begin());

	//     sort(surface_points.begin(), surface_points.end(), mysort);
	//     SpeedContrl();
	//     ShowPath();
	// }
}

float SpeedCtr::GetSpdBySurface(const double surface)
{
	float speed;
	if (surface > surface_value)
		speed = min_speed;
	else
		speed = max_speed;
	return speed;
}

void SpeedCtr::ShowSpeed(double v)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

	marker.scale.z = 0.2;
	marker.color.b = 0.0f;
	marker.color.g = 1.0f;
	marker.color.r = 0.0f;
	marker.color.a = 1;

	char buf[200];
	sprintf(buf, "%.2f", v);
	marker.text = buf;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0.6;
	marker_pub.publish(marker);
}

void SpeedCtr::SpeedContrl()
{
	float speed_ctr = max_speed;
	float front_speed = 0, rear_speed = 0;
	int front_count = 0, rear_count = 0;
	double speed = max_speed;
	for (auto it = surface_points.begin(); it != surface_points.end(); ++it)
	{
		if (it->x < front_distance_max && it->x > front_distance_min)
		{
			front_speed += it->y, ++front_count;
			// cout << it->z << " ";
			speed = min(speed, it->y);
		}

		if (it->x < rear_distance_max && it->x > rear_distance_min)
		{
			rear_speed += it->y, ++rear_count;
		}
	}
	// cout << endl;
	// cout << endl;
	// printf("%d\n",count);
	front_speed /= front_count;
	// cout << "front_speed = " << front_speed << " ";
	rear_speed /= rear_count;
	// cout << "rear_speed = " << rear_speed << endl;
	if (front_count > 1 && rear_count > 1)
	{
		speed_ctr = min(front_speed, rear_speed);
	}
	else if(front_count > 1 && rear_count < 1)
	{
		speed_ctr = front_speed;
	}
	else if(front_count < 1 && rear_count > 1)
	{
		speed_ctr = rear_speed;
	}
	else speed_ctr = max_speed;

	surface_speed = speed;
	CalCurVelCmd();
}
void SpeedCtr::CalCurVelCmd()
{
	float speed_cmd = min(surface_speed, posture_speed);
	ShowSpeed(speed_cmd);
	geometry_msgs::Twist msg;
	msg.linear.x = speed_cmd;
	velcmd_pub.publish(msg);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "speed_ctr");
	ROS_INFO_STREAM("speed ctr start");
	SpeedCtr sc;
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		// ROS_INFO("");
		// printf("speed ctr start\n");
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
