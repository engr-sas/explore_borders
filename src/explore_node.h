/* Author: Shamsudeen A Sodangi */
#ifndef explor_node_H
#define explore_node_H
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
//#include <actionlib_msgs/GoalID.h>
#include <cmath>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/Bool.h"

bool view;
int cm_count = 0;

actionlib_msgs::GoalStatusArray *goal_status_;
int steps, p_size, active_feature, laser_align;
bool other_side_md, edge_switch, exploring, use_odom;
bool read_points, start_inside, first_run, laser_clockwise;
geometry_msgs::Pose mb_robot_pose, robot_pose, goal_record;
geometry_msgs::Point obstacle, edge_one_point;
GenerateLoops* gl_;
double scan_segment, inflation_angle;
double robot_length, robot_width, pi, hold_distance, plan_tol, wg_rate;
double resolution, origin_x, origin_y, longest_line, vir_max_range;
double rob_lin_vel, rob_rot_vel, costmap_length, sensor_range, robot_span;
double left_average, right_average, front_average, g_inflation, clear_dist;
double xy_tol, orien_tol, t_now, last_time, rp_time, stuck_time, g_margin;
float safe_speed;
int expected_edges, num_nodes;
nav_msgs::OccupancyGrid o_grid, map_grid;
nav_msgs::Odometry odom_data;
nav_msgs::Odometry *odom_data_ = &odom_data;
ros::Duration *g_rest_p;
ros::Publisher  measure_pub, ac_poly_pub;
ros::Publisher  goal_publisher, plan_publisher, boundary_publisher;
sensor_msgs::LaserScan scans_;
sensor_msgs::LaserScan *scans = &scans_;
std::vector<geometry_msgs::Point> global_vertices, obstacles, half_line, edge_obstacles, ori_plan;
std::vector<int> checked_features, to_be_explored, plan_points;
std::vector<std::vector<int> > border_goals;
std::vector<std::vector<geometry_msgs::Point> > features, feature_edges;
std::string map_frame, odom_frame, odom_topic, scan_topic, clicked_point, map_topic;
tf::TransformListener *tl_;

//functions  A 
void addToFeature(int feat_index, geometry_msgs::Point point);
bool borderHijack();
bool borderCheck(int feature_index);
void closenessCheck(double margin);
bool closeToPlan(int gwf);
geometry_msgs::Point closestPointOnLine(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point ref, double margin);
geometry_msgs::Point closestPointOnFeat(int feat_id);
bool closePoints(geometry_msgs::Point p1, geometry_msgs::Point p2, float margin);
void continueOnPlan();
void createHalfLine(std::string dir);
bool crossedEdges(std::string* dir_p, bool *other_side,std::vector<geometry_msgs::Point> *traj, geometry_msgs::Point* start, int feature_index);
void erasePlanPoint(int index);
void erasePlanBefore(int index);
bool equalPose(geometry_msgs::Pose p1, geometry_msgs::Pose p2, double xy_tol, double orien_tol);
geometry_msgs::Pose escapeWall(std::string direction, double del_lin);
bool featureIsInsideArea(int feat_id);
geometry_msgs::Pose findFree(geometry_msgs::Pose target);
bool featureChecked(int id);
bool featureClose(int feat_id);
bool featureCleared(int feat_id);
geometry_msgs::Pose firstWallGoal(std::string direction, geometry_msgs::Point obs, double hold_at);

//G - I
void generateEdges();
int getIndexFromMap(geometry_msgs::Point point);
double getFrontScan();
bool getOdom();
double getLongestLine(std::vector<geometry_msgs::Point> bounds);
double getZFromQuat(geometry_msgs::Quaternion quat);
bool goToAction(geometry_msgs::Pose goal);
bool goToAction(geometry_msgs::Pose goal, int repeats);
bool holdGoalObserve(geometry_msgs::Pose goal);
bool intersection(geometry_msgs::Point* point_out, geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point a1, geometry_msgs::Point a2);
void insertToFeature(int feat_index, geometry_msgs::Point point, int min_sn, int max_sn, int mid_sn);
int isInFeature(geometry_msgs::Point);
bool isInside(geometry_msgs::Point point, std::vector<geometry_msgs::Point> enclosure);
bool isPrimary(geometry_msgs::Point point);
bool initializeFollow(std::string* dir, double hold_at, geometry_msgs::Point obs);

//M -Z
void matchFeatures();
void managePlan();
void mergeFeatures(int feat_1, int feat_2);
geometry_msgs::Point midPoint(geometry_msgs::Point p1, geometry_msgs::Point p2);
geometry_msgs::Pose midPose(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
bool moveCloser(geometry_msgs::Pose goal, double target_margin);
bool obstacleOnPoint(geometry_msgs::Point p1);
bool obstacleOnPoint(geometry_msgs::Point p1, double margin);
bool obstacleOnLine(geometry_msgs::Point p1, geometry_msgs::Point p2);
bool obstacleOnLineMap(geometry_msgs::Point p1, geometry_msgs::Point p2, double inflation);
bool obstacleOnMap(geometry_msgs::Point point, double inflation);
bool onLine(geometry_msgs::Point p1, geometry_msgs::Point p2);
void orderToBeExplored();
std::vector<int>  possiblyInFeature(geometry_msgs::Point point, std::vector<std::vector<int> >* pos_vec, bool *sorted_p);
geometry_msgs::Point pointOnLine(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point ref, double dist, bool reverse);
bool repeating(std::vector<geometry_msgs::Point> *steps_rec);
void removeAroundFeatures(int feature_index);
bool stuck(geometry_msgs::Point* last_pos, double rec_rate);
bool setBlindGoal(geometry_msgs::Pose goal_in, double rate);
void setExpParams(ros::NodeHandle n);
geometry_msgs::Pose translateLinear(geometry_msgs::Pose pose, double del_lin);
void updateCostmap(nav_msgs::OccupancyGrid og);
void updateCheckedFeatures(int id);
void updateLists(int index, int moved_to);
void updateToBeExplored(int index);
bool withinBounds(geometry_msgs::Point point, std::vector<geometry_msgs::Point> enclosure, double margin);

#endif