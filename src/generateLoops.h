/* Author: Shamsudeen A Sodangi */
#ifndef global_exp_H
#define global_exp_H
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Path.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>


class GenerateLoops{
	public:
	GenerateLoops(double, std::string);
	
	//variables
	double del_points, rob_lin_vel, rob_rot_vel, del_goal_points, pi ,sensor_range;
	double *sensor_range_ ;
	geometry_msgs::Pose curr_pose;
	geometry_msgs::PointStamped clickedpoint;
	geometry_msgs::PolygonStamped exp_boundary;
	nav_msgs::Path global_plan;
	nav_msgs::Path *global_plan_;
	std::string map_frame;
	std::vector<int> num_vertices;
	std::vector<geometry_msgs::Point> p_boundary, global_loops, primary_points, inner_bound;
	std::vector<geometry_msgs::Pose> cur_goal_poses;
	std::vector<std::vector<geometry_msgs::Point> > all_walls, all_sections;
	std::vector<int> goal_edge_rec, loop_sn;

	//functions
	float angleBetween(float a1, float a2);
	std::vector<std::vector<geometry_msgs::Point> > allInnerEdges(std::vector<geometry_msgs::Point> vertices, float range);
	std::vector<geometry_msgs::Point> buildLoop(std::vector<geometry_msgs::Point> wallEdges, float laser_range);
	std::vector<geometry_msgs::Point> bestPath(std::vector<geometry_msgs::Point> plan);
	geometry_msgs::Point closestPoint(std::vector<geometry_msgs::Point> targets, geometry_msgs::Pose start, float robLinSpeed, float robRotSpeed);
	std::vector<geometry_msgs::Point> createInnerPointsAlt(std::vector<geometry_msgs::Point> bounds, double off_dist);
	geometry_msgs::PolygonStamped createPoly(std::vector<geometry_msgs::Point> pv);
	geometry_msgs::Point createPoint(float x, float y, float z);
	bool createInnerEdges(std::vector<geometry_msgs::Point> vertices, float range, std::vector<geometry_msgs::Point> *vert_out);
	
	bool equalPoints(geometry_msgs::Point p1, geometry_msgs::Point p2, float margin);
	std::vector<geometry_msgs::Point> extractWall(std::vector<geometry_msgs::Point> g_plan, std::vector<int> nodes, int wall);
	std::vector<int> findReflex(std::vector<float> angles);
	
	void getLoops(std::vector<geometry_msgs::Point> p);
	double getMaxDiagonal(std::vector<geometry_msgs::Point> vertices);
	geometry_msgs::Pose getCenter(std::vector<geometry_msgs::Point> wall);
	float getLength(geometry_msgs::Point p1, geometry_msgs::Point p2);
	float grad(float x1, float y1, float x2, float y2);
	float gradPoints(geometry_msgs::Point p1, geometry_msgs::Point p2);
	float getRotationTime(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2, float rotSpeed);
	std::vector<geometry_msgs::Pose> generatePath(std::vector<geometry_msgs::Point> g_plan);
	float getDistTime(geometry_msgs::Point p1, geometry_msgs::Point p2, float speed);
	float getPerimeter(std::vector<geometry_msgs::Point> vertices);
	geometry_msgs::Quaternion getQuatFromP(geometry_msgs::Point p1, geometry_msgs::Point p2);
	std::vector<geometry_msgs::Point> getPoints(geometry_msgs::Point p1, geometry_msgs::Point p2, float del_dist);
	std::vector<geometry_msgs::Pose> getPosesBtw(geometry_msgs::Point p1, geometry_msgs::Point p2, float del);
	

	std::pair<std::vector<geometry_msgs::Point>, float> innerEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, float range);
	std::pair<geometry_msgs::Point, float> innerEdgeWall(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, float range);
	std::vector<geometry_msgs::Pose> orderPoses(std::vector<geometry_msgs::Pose> loop, geometry_msgs::Point start);	
	geometry_msgs::Point midPoint(geometry_msgs::Point p1, geometry_msgs::Point p2);
	float minSide(std::vector<geometry_msgs::Point> vertices);
	bool offsetTrue(std::vector<geometry_msgs::Point> outer, std::vector<geometry_msgs::Point> inner);
	
	geometry_msgs::Point pointOnLine(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point ref, double dist, bool reverse);
	void planThread(std::vector<geometry_msgs::Point> rl, std::vector<int> nv);
	float posAngle(float ang);
	std::vector<geometry_msgs::PoseStamped> poseToStamped(std::vector<geometry_msgs::Pose> pose);
	char quad(geometry_msgs::Point origin, geometry_msgs::Point p);
		
	std::vector<geometry_msgs::PoseStamped> rotateAtPoint(geometry_msgs::PoseStamped ref);
	std::vector<geometry_msgs::Pose>  rotateAtPoint(geometry_msgs::Pose ref);
	geometry_msgs::Pose rotatePose(geometry_msgs::Pose origin, double z);	
	void recievePoint(geometry_msgs::Point p);
	float treatAngle(float ang, char quad);
};

#endif