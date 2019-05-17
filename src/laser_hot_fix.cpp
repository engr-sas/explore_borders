#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"

ros::Publisher s_pub;
sensor_msgs::LaserScan scans_out;
std::string sub_scan;
std::string pub_scan;
double max_r;

//take in laser scan and truncate after max_r for gmappiung
void laserCallback(const sensor_msgs::LaserScan msg){
	scans_out = msg;

	for(int i = 0; i < msg.ranges.size(); i++){ //scan 45
		if(msg.ranges[i] > max_r){
			scans_out.ranges[i] = max_r;
		}
	}
	s_pub.publish(scans_out);
}

int main(int argc, char** argv) {
	std::cout<<"laser_hot_fix \n";
	ros::init(argc, argv, "laser_hot_fix");
	ros::NodeHandle	n;
	
	if(!n.getParam("/laser_hot_fix/max_range", max_r)){
		max_r = 6;
		ROS_INFO("max range set to default 6 '/max_range'");
	}
	
	if(!n.getParam("/laser_hot_fix/sub_scan_topic", sub_scan)){
		sub_scan = "scan";
		ROS_INFO("subscribing to 'scan' as default or set param");
	}
	
	if(!n.getParam("/laser_hot_fix/pub_scan_topic", pub_scan)){
		pub_scan = "scan_fix";
		ROS_INFO("publishing to '/scan_fix' default or set param");
	}
	
	
	ros::Subscriber laser_sub= n.subscribe(sub_scan, 200 ,laserCallback);
	s_pub = n.advertise<sensor_msgs::LaserScan>(pub_scan, 200);
	ros::spin();
}