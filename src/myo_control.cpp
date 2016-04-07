#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Twist.h"

bool flying = false;
bool control_mode = false;

void myo_callback(const std_msgs::UInt8::ConstPtr& command, ros::Publisher takeoff_pub, ros::Publisher land_pub, ros::Publisher steer_pub) {
	/*Messages for publishing: */
	std_msgs::Empty emptyMsg;
	geometry_msgs::Twist twist;
	
	twist.linear.x = twist.linear.y = twist.linear.z = 0;
	twist.angular.x = twist.angular.y = twist.angular.z = 0;
	
	switch(command->data) {
		case 1:
			if(flying) {
				if(control_mode) {
					printf("Fly front\n");
					twist.linear.x = 2;
					steer_pub.publish(twist);
				} else {
					printf("Fly up\n");
					twist.linear.z = 3;
					steer_pub.publish(twist);
				}
			} else {
				printf("Take off\n");
				takeoff_pub.publish(emptyMsg);
				flying = true;
			}
			break;
		case 4:
			if(flying) {
				if(control_mode) {
					printf("Fly back\n");
					twist.linear.x = -2;
					steer_pub.publish(twist);
				} else {
					printf("Fly down\n");
					twist.linear.z = -3;
					steer_pub.publish(twist);
				}
			}
			break;
		case 2:
			if(flying) {
				if(control_mode) {
					printf("Fly left\n");
					twist.linear.y = 2;
					steer_pub.publish(twist);
				} else {
					printf("Rotate counterclockwise\n");
					twist.angular.z = 2;
					steer_pub.publish(twist);
				}
			}
			break;
		case 3:
			if(flying) {
				if(control_mode) {
					printf("Fly right\n");
					twist.linear.y = -2;
					steer_pub.publish(twist);
				} else {
					printf("Rotate clockwise\n");
					twist.angular.z = -2;
					steer_pub.publish(twist);
				}
			}
			break;
		case 5:
			if(control_mode) {
				printf("Switch control mode true\n");
				control_mode = false;
			} else {
				printf("Switch control mode false\n");
				control_mode = true;
			}
			break;
		default:
			twist.linear.x = twist.linear.y = twist.linear.z = 0;
			twist.angular.x = twist.angular.y = twist.angular.z = 0;
			steer_pub.publish(twist);
			break;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "myo_pose");

	ros::NodeHandle n;

	ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1, true);
	ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1, true);
	ros::Publisher steer_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber myo_sub = n.subscribe<std_msgs::UInt8>("/myo/data/thalmic_gesture", 1000, boost::bind(myo_callback, _1, takeoff_pub, land_pub, steer_pub));
	
	/*rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'*/
	
	ros::Rate loop_rate(50);
	
	ros::spin();
	
	return 0;
}