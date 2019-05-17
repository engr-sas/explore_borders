/* Author: Shamsudeen A Sodangi */
#include "generateLoops.cpp"
//#include "objects.cpp"
#include "explore_node.h"

void costmapUpCallback(const map_msgs::OccupancyGridUpdate msg){
	//do nothing
	//ros::spinOnce();
}

void statusCallback(const actionlib_msgs::GoalStatusArray msg){
	*goal_status_ = msg;
}

void costmapCallback(const nav_msgs::OccupancyGrid msg){
	o_grid = msg;
	//updateCostmap(o_grid);
}

void mapCallback(const nav_msgs::OccupancyGrid msg){
	map_grid = msg;
	/*if(!objects_map_p->initialized){//todo make internal
		std::cout<<"subscriber \n";
		objects_map_p->initialize(map_grid, 0.1);
		//map_ini = true;
	}*/
}

void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){ 
	if(global_vertices.size() < num_nodes){
		ROS_INFO("Marked Point \n");
		global_vertices.push_back(msg->point);
		boundary_publisher.publish(gl_->createPoly(global_vertices));
	}else ROS_WARN("Number of defined Vertices in param set \n");
}

void odomCallback(const nav_msgs::Odometry msg){
	*odom_data_ = msg;
}

void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback msg){
	mb_robot_pose = msg.feedback.base_position.pose;
	if(!use_odom) robot_pose = mb_robot_pose;
}

void laserCallback(const sensor_msgs::LaserScan msg){
	*scans = msg;
}

int main(int argc, char** argv) {
	std::cout<<"Explore node started \n";
	ros::init(argc, argv, "explore node");
	ros::NodeHandle	n;
	setExpParams(n);
	view = false;
	first_run = true;
	//map_ini = false;
	
	g_margin = 0.2; //ratios used for costmap, addToFeature(1), half line(0.5), obstacle on point	
	vir_max_range = sensor_range + 1;
	pi = 3.14159;
	edge_switch = true;
	//edge_one = true; 
	read_points = true;
	half_line.resize(2);
	plan_tol = 1; //param max min  based on sensor etc
	inflation_angle = 0.1;
	wg_rate = 0.5; //TODO param rate goals are set when following wall. Tuned ish
	safe_speed = 0.1;

	actionlib_msgs::GoalStatusArray goal_status; //instantiate
	goal_status_ = &goal_status;
	checked_features.push_back(-1);
	active_feature = -1;
	
	ros::Duration g_rest(0.1);
	g_rest_p = &g_rest;
	
	//Objects objects_map;
	//objects_map_p = &objects_map;
	
	GenerateLoops gl(sensor_range, map_frame);
	gl_ = &gl;
	gl.rob_lin_vel = rob_lin_vel; 
	gl.rob_rot_vel = rob_rot_vel;
	gl.del_goal_points = 1; //TODO set param based on sensor range
			
	ros::Subscriber odom_sub= n.subscribe(odom_topic, 20 ,odomCallback);
	ros::Subscriber rviz_sub = n.subscribe(clicked_point, 10 ,rvizCallBack);
	ros::Subscriber laser_sub= n.subscribe(scan_topic, 20 , laserCallback);
	ros::Subscriber map_sub= n.subscribe(map_topic, 20 , mapCallback);
	ros::Subscriber costmap_sub	= n.subscribe("/move_base/local_costmap/costmap", 20, costmapCallback);
	ros::Subscriber costmap_updates_sub	= n.subscribe("/move_base/local_costmap/costmap_updates", 10, costmapUpCallback);
	ros::Subscriber goal_status_sub	= n.subscribe("/move_base/status", 50, statusCallback);
	ros::Subscriber feedback_sub = n.subscribe("/move_base/feedback", 200, feedbackCallback);
	
	plan_publisher = n.advertise<nav_msgs::Path>("/global_plan_path", 50);
	goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 200);
	boundary_publisher = n.advertise<geometry_msgs::PolygonStamped>("/exp_boundary", 20);
	ac_poly_pub = n.advertise<geometry_msgs::PolygonStamped>("/active_polygon", 20);
	cmd_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 200);
	measure_pub = n.advertise<std_msgs::Bool>("odom_total", 50);
	
	ros::Rate loop_rate(5);
	bool run = true;
	while(ros::ok){
		if(run && (num_nodes == global_vertices.size())){
			goal_record = robot_pose;	
			if(getOdom()){
				double start_time =ros::Time::now().toSec();			
				longest_line = getLongestLine(global_vertices);
				
				gl.curr_pose = robot_pose;
				gl.getLoops(global_vertices);
				gl.planThread(gl.global_loops, gl.num_vertices);
				start_inside = isInside(robot_pose.position, global_vertices);
				
				plan_publisher.publish(gl.global_plan);
				boundary_publisher.publish(gl.exp_boundary);
				
				goToAction(translateLinear(robot_pose, 1)); //move_base hotfix;
				goToAction(gl_->rotatePose(robot_pose, pi/2));
				managePlan();
				
				double fini_time =ros::Time::now().toSec();
				std::cout<<"Exploration time: "<<fini_time - start_time<<"\n";
				
				std_msgs::Bool c_true; c_true.data = true;
				measure_pub.publish(c_true);	
				system("rosrun map_server map_saver -f my_map");
				loop_rate.sleep();
				run = false;
				//ros::shutdown();
			}else ROS_INFO("trying to get robot position");	
		}
		ros::spinOnce();
		loop_rate.sleep();	
	}
}

double getLongestLine(std::vector<geometry_msgs::Point> bounds){
	std::vector<geometry_msgs::Point> temp_bounds;
	temp_bounds = bounds;
	double curr_max = 0;
	
	for(int i = 0; i < temp_bounds.size(); i ++){
		for(int j = 0; j < temp_bounds.size(); j ++){
			if(gl_->getLength(temp_bounds[i], temp_bounds[j]) > curr_max){
				curr_max = gl_->getLength(temp_bounds[i], temp_bounds[j]);
			}
		}
		temp_bounds.erase(temp_bounds.begin());
	}
	return curr_max;
}

void setExpParams(ros::NodeHandle n){
	if(!n.getParam("/explore_node/laser_clockwise", laser_clockwise)){
		laser_clockwise = true;
		laser_align = -1;
		ROS_INFO("laser_msg assumed to be clockwise");
	}else{
		if(laser_clockwise){
			ROS_INFO("laser is clockwise");
			laser_align = -1;
		}else laser_align = 1;
	}
	if(!n.getParam("/explore_node/clear_dist", clear_dist)){
		clear_dist = 1;
		ROS_INFO("clearance distance set as one");
	}
	if(!n.getParam("/explore_node/scan_segment", scan_segment)){
		scan_segment = pi/8;
		ROS_INFO("clearance distance set as one");
	}
	if(!n.getParam("/explore_node/map_frame", map_frame)){
		map_frame = "/map";
		ROS_INFO("Map frame set to default '/map'");
	}
	if(!n.getParam("/explore_node/map_topic", map_topic)){
		map_topic = "/map_topic";
		ROS_INFO("map_topic set to default '/map_topic'");
	}
	if(!n.getParam("/explore_node/odom_frame", odom_frame)){
		odom_frame = "/odom";
		ROS_INFO("odom frame set to default '/odom'");
	}
	if(!n.getParam("/explore_node/scan_topic", scan_topic)){
		scan_topic = "/scan";
		ROS_INFO("scan_topic set to default '/scan'");
	}
	if(!n.getParam("/explore_node/odom_topic", odom_topic)){
		odom_topic = "/odom_topic";
		ROS_INFO("odom topic set to default '/odom_topic'");
	}
	if(!n.getParam("/explore_node/use_odom", use_odom)){
		use_odom = false;
		ROS_INFO("using move_base/feedback for localization");
	}
	if(!n.getParam("/explore_node/clicked_point", clicked_point)){
		clicked_point = "/clicked_point";
		ROS_INFO("clicked_point set to default '/clicked_point'");
	}
	if(!n.getParam("/explore_node/resolution", g_margin)){
		g_margin = 0.2;
		ROS_INFO("resolution set to default 0.2 '/resolution'");
	}
	if(!n.getParam("/explore_node/vertices", num_nodes)){
		num_nodes = 4;
		ROS_INFO("Number of vertices set to default '4'");
	}else{
		if(num_nodes < 3){
			num_nodes = 3;
			ROS_INFO("Number of vertices in params cant be less than 3. Reverted to default '4'");
		}
	}
	if(!n.getParam("/explore_node/inflation", g_inflation)){
		g_inflation = 1;
		ROS_INFO("Inflation of explore map set as '1'");
	}
	
	
	if(!n.getParam("/explore_node/max_rot_vel", rob_rot_vel)){
		if(!n.getParam("/move_base/DWAPlannerROS/max_rot_vel", rob_rot_vel) &&
		  !n.getParam("/move_base/TrajectoryPlannerROS/max_vel_theta", rob_rot_vel)){
			rob_rot_vel = 1; // 57 deg per sec
			ROS_INFO("Max rotational Velocity set to default '0.5'");
		}else {
			rob_rot_vel = rob_rot_vel/2;
			ROS_INFO("Inherited max rotational velocity %e from move_base", rob_rot_vel);
		}
	}else rob_rot_vel = rob_rot_vel/2;
	
	if(!n.getParam("/explore_node/max_lin_vel", rob_lin_vel)){
		if(!n.getParam("/move_base/DWAPlannerROS/max_trans_vel", rob_lin_vel) &&
		  !n.getParam("move_base/TrajectoryPlannerROS/max_vel_x", rob_lin_vel)){
			rob_lin_vel = 2.0;
			ROS_INFO("Max Linear Velocity set to default '1.0'");
		}else {
			rob_lin_vel = rob_lin_vel/2;
			ROS_INFO("Inherited max linear velocity %e from move_base",rob_lin_vel );
		}
	}else rob_lin_vel = rob_lin_vel/2;
	
	if(!n.getParam("/explore_node/sensor_range", sensor_range)){
		if(!n.getParam("/move_base/global_costmap/raytrace_range", sensor_range)){
			sensor_range = 5;
			ROS_INFO("Sensor Range set to default '5'");
		}else ROS_INFO("Inherited sensor range %e from move_base", sensor_range);
	}
	sensor_range = sensor_range * 0.9;
	
	if(!n.getParam("/move_base/TrajectoryPlannerROS/xy_goal_tolerance", xy_tol) &&
	   !n.getParam("/move_base/DWAPlannerROS/xy_goal_tolerance", xy_tol)){
		xy_tol = 0.7;
		ROS_WARN("Could not catch xy_goal_tolerance from move_base");
	}else{
		xy_tol = xy_tol*2;
	}
	
	if(!n.getParam("/move_base/TrajectoryPlannerROS/yaw_goal_tolerance", orien_tol) &&
	   !n.getParam("/move_base/DWAPlannerROS/yaw_goal_tolerance", orien_tol)){
		orien_tol = 0.3; //11 deg
		ROS_WARN("Could not get yaw goal_tolerance from move_base, 0.1 assumed");
	}else{
		orien_tol = orien_tol*2;
	}
	
	if(!n.getParam("/move_base/local_costmap/height", costmap_length)){
		costmap_length = 5;
		ROS_WARN("Could not get costmap_length from move_base");
	}else{
		costmap_length = costmap_length/2;
	}
		
	if(!n.getParam("/explore_node/robot_span", robot_span)){
		robot_width = 1;
		robot_length = 1;
		robot_span = 1;
		ROS_WARN("Could not get footprint from move_base, 1x1 set");
	}else{
		robot_width = robot_span;
		robot_length = robot_span;
	}
}

bool getOdom(){
	ros::spinOnce();
		double start_time =ros::Time::now().toSec();
		tf::TransformListener tom;
		tom.waitForTransform(odom_frame, map_frame, ros::Time(0), ros::Duration(2.0));
		geometry_msgs::PoseStamped p_in;
		geometry_msgs::PoseStamped p_out;
		p_in.pose = odom_data_->pose.pose;
		p_in.header.frame_id = odom_frame;
		p_out.header.stamp = ros::Time::now();
		p_out.header.frame_id = map_frame;

		try{		
			tom.transformPose(map_frame, p_in, p_out);
		}
		catch (tf::TransformException ex){
			ROS_WARN("%s",ex.what());
			ROS_WARN("Could not get odometry");
			return false;
		}
		robot_pose = p_out.pose;
		double finish_time =ros::Time::now().toSec();
		if((finish_time - start_time) > 0.05){
			//std::cout<<"OdomTime: "<<finish_time - start_time<<" \n";
		}
		return true;
}

void managePlan(){
	exploring = true;
	std::cout<<"start "<<gl_->global_plan.poses.size()<<"\n";
	p_size = gl_->global_plan.poses.size();
	int p_count = 0;
	for(int i = 0; i < gl_->global_plan.poses.size(); i++){
		plan_points.push_back(p_count);
		p_count ++;
	}
	int max_steps = 0.9 * costmap_length / gl_->del_goal_points;
	bool node_goal;
	bool skipped = false;
	ros::Rate loop_rate(1);
	geometry_msgs::Pose goal;

	for(int w = 0; w < gl_->global_plan.poses.size(); w++) ori_plan.push_back(gl_->global_plan.poses[w].pose.position);
	
	while(!getOdom()) {
		loop_rate.sleep();
	}

	while(exploring){	
		//g_rest_p->sleep();
		//ROS_INFO("Exploring \n");
		if(!start_inside) start_inside = isInside(robot_pose.position, global_vertices);
		steps = max_steps;
		getOdom();
		borderHijack();
			
		if(gl_->global_plan.poses.size() == 0){
			exploring = false;
			borderHijack();
			ROS_INFO("Exploration complete");
			break;
		}else if(gl_->global_plan.poses.size() < max_steps){
			int last_index = gl_->global_plan.poses.size()-1;
			if(!obstacleOnLine(robot_pose.position, gl_->global_plan.poses[last_index].pose.position)){
				holdGoalObserve(gl_->global_plan.poses[gl_->global_plan.poses.size()-1].pose);
			}else{ 
				int ftr = isInFeature(obstacle);
				borderCheck(ftr);
			}
			exploring = false;
			borderHijack();
			ROS_INFO("Exploration complete");
			break;
		}else if(gl_->global_plan.poses.size() >= steps){		
			goal = gl_->global_plan.poses[steps].pose;
			while(gl_->getLength(goal.position, robot_pose.position) > sensor_range){
	
				if(steps == 0){
					if(gl_->getLength(goal.position, robot_pose.position) > costmap_length){
						goal = findFree(goal);
					}else goal = gl_->global_plan.poses[steps].pose;
					break;
				}
				steps--;
				goal = gl_->global_plan.poses[steps].pose;
			}
			
		}

		node_goal =false;
		//if any of next steps goals is primary goTo 	
		for(int i = 0; i <= steps; i++){
				if(isPrimary(gl_->global_plan.poses[i].pose.position)){
					std::cout<<"prim ";
					steps = i;
					node_goal = true;
					if(!obstacleOnLine(robot_pose.position, gl_->global_plan.poses[i].pose.position)){
						//setblind goal, check reach, check side or whatever else is relevant
						goal = gl_->global_plan.poses[i].pose;
						if(holdGoalObserve(goal)){
							erasePlanBefore(i);	
						}
					}else if((i > 0) && (gl_->getLength(robot_pose.position, obstacle) > sensor_range)
							 && (gl_->getLength(robot_pose.position, gl_->global_plan.poses[0].pose.position)
								 > (gl_->del_goal_points + (plan_tol*1.5))) 
							 && (moveCloser(gl_->global_plan.poses[0].pose, (plan_tol))) ){ //if not on path
						//if cant moveCloser 
							setBlindGoal(gl_->global_plan.poses[i].pose, 1);
						std::cout<<"mc didnt ng";
					}else{
						
						int ftr = isInFeature(obstacle);
						if(!featureChecked(ftr)){	
							//if(featureIsInsideArea(ftr)){
								if(!borderHijack()){
									erasePlanBefore(i);	
								}
								borderCheck(ftr);
							/*}else{
								//redundancy to avoid getting stuck (map, odom conflict)
									std::cout<<"mp 24 \n";
								goal = findFree(gl_->global_plan.poses[i].pose);
								setBlindGoal(goal, 2);
								erasePlanPoint(i);
							}*/
						}else{//there is still an obs but not properly id'd
							removeAroundFeatures(ftr);
							goal = gl_->global_plan.poses[i].pose;
							/*if(!obstacleOnMap(goal.position, g_inflation)){
									if(holdGoalObserve(goal)){
										erasePlanBefore(i);	
									}
							}else */if(!moveCloser(goal, (plan_tol))){
								if((i+2) > gl_->global_plan.poses.size()){
									setBlindGoal(gl_->global_plan.poses[i+1].pose, 1);
								}
							}
						}
					}
				} 
		}

		if(!node_goal ){
			std::cout<<"notNode ";
			if(obstacleOnLine(robot_pose.position, goal.position)){
				int ftr = isInFeature(obstacle);
				if((steps > 0) && (gl_->getLength(robot_pose.position, obstacle) > sensor_range) 
				   && (gl_->getLength(robot_pose.position, gl_->global_plan.poses[0].pose.position) > 
					(gl_->del_goal_points + (plan_tol*1.5))) 
				   && (!moveCloser(gl_->global_plan.poses[0].pose, (plan_tol))) ){ //if not on path
						//if move closer cant
					setBlindGoal(gl_->global_plan.poses[steps].pose, 1);
					std::cout<<"mc didnt !ng";
				}else if(!featureChecked(ftr)){
					//if(featureIsInsideArea(ftr)){
						if(!borderHijack()){
							erasePlanBefore(steps);
						}
						borderCheck(ftr);
					/*}else{
						goal = findFree(goal);
						setBlindGoal(goal, 2);
					}*/

				}else{
					removeAroundFeatures(ftr);
					goal = gl_->global_plan.poses[steps].pose;
					if(!obstacleOnMap(goal.position, g_inflation)){
						setBlindGoal(goal, 2);
					}else if(!moveCloser(goal, (plan_tol))){
						if((steps + 1) < gl_->global_plan.poses.size()){
							setBlindGoal(gl_->global_plan.poses[steps+1].pose, 1);
						}
					}
				}
				
			}else{
				setBlindGoal(goal, 2);
			}
		}
	
		closenessCheck(plan_tol);
	}
}

void erasePlanBefore(int index){
	//inclusive
	for(int i = index; i >=0 ; i--){
		//std::cout<<"erase PB gl["<<i<<"]: "<<gl_->global_plan.poses[i].pose.position<<"\n";
		erasePlanPoint(i);
	}
}

void erasePlanPoint(int index){
	//too small to bother with sort
	
	if(plan_points.size() > index){
			gl_->global_plan.poses.erase(gl_->global_plan.poses.begin() + index);
			plan_points.erase(plan_points.begin() + index);
			plan_publisher.publish(gl_->global_plan);
	}
}

bool moveCloser(geometry_msgs::Pose goal, double target_margin){
	std::cout<<"moveCloser \n";
	geometry_msgs::Pose temp_goal = goal;
	target_margin = target_margin * 0.8;
	//straight line
	while(gl_->getLength(robot_pose.position, temp_goal.position) > target_margin){
		temp_goal.position = midPoint(robot_pose.position, temp_goal.position);
		if(!obstacleOnMap(temp_goal.position, g_inflation)){
			setBlindGoal(temp_goal, 2);
			std::cout<<"original goal to mC: "<<goal.position<<"\n";
			std::cout<<"set goal from mC: "<<temp_goal.position<<"\n";
			return true;
		}
	}
	
	/*double arr_z[] = {-0.2, 0.2, -0.4, 0.4, -0.6, 0.6};
	int n = sizeof(arr_z)/sizeof(arr_z[0]);
	std::vector<double> del_z(arr_z, arr_z + n);
	
	double goal_len = gl_->getLength(robot_pose.position, goal.position);
	geometry_msgs::Quaternion straight = gl_->getQuatFromP(robot_pose.position, goal.position);
	double z_s = getZFromQuat(straight);
	for(int i = 0; i < del_z.size(); i++){
		std::cout<<"del_z["<<i<<"]: "<<del_z[i]<<"\n";
		geometry_msgs::Pose new_goal = robot_pose;
		new_goal.orientation = straight;
		new_goal.position.x += goal_len * cos(z_s + del_z[i]); 
		new_goal.position.y += goal_len * sin(z_s + del_z[i]); 
		std::cout<<"new_goal: "<<new_goal.position<<"\n";
		if(!obstacleOnPoint(new_goal.position)){
			setBlindGoal(new_goal, 1);
			return true;
		}
	}*/
	
	return false;
}

bool isPrimary(geometry_msgs::Point point){
	for(int i = 0; i < gl_->primary_points.size(); i++){
		if(gl_->equalPoints(point, gl_->primary_points[i], 0.1)){
			return true;		
		}
	}
	return false;
}

bool featureIsInsideArea(int feat_id){
	for(int y = 0; y < feature_edges[feat_id].size(); y++){
		if(!isInside(feature_edges[feat_id][y], global_vertices)) return false;	
	}
	return true;
}

bool isInside(geometry_msgs::Point point, std::vector<geometry_msgs::Point> enclosure){
	
	//std::cout<<"\n check point: "<<point<<"\n";
	//std::cout<<"\n robot_pose.position: "<<robot_pose.position<<"\n";
	geometry_msgs::Point *dummy;
	if(enclosure.size() > 2){
		if(withinBounds(point, enclosure, 0)){
			//if intersection within given bounds then its outside
			//interest point comes in second set of lines
			for(int i2 = 0; i2 < enclosure.size(); i2++){//draw line to each edge
				for(int i3 = 0; i3 < enclosure.size() - 1; i3++){// draw lines btwn edges
					if((i2 != i3) && (i2 != (i3 + 1))){//skip same starts
						if(intersection(dummy, enclosure[i3], enclosure[i3 +1], enclosure[i2], point)){
							//std::cout<<"when going to :"<<i2<<"intersects :"<<i3<<" to"<<i3+1<<"\n";
							//for(int i = 0; i < enclosure.size(); i++) std::cout<<"enclosure["<<i<<"]: "<<enclosure[i]<<"\n";
							return false;
						}
					}
				}
				if((i2 != 0) && (i2 != (enclosure.size() -1))){
					if(intersection(dummy, enclosure[0], enclosure[enclosure.size() -1], enclosure[i2], point)) {
						//std::cout<<"when going to :"<<i2<<"intersects :"<<0<<" to"<<enclosure.size() -1<<"\n";
						//for(int i = 0; i < enclosure.size(); i++) std::cout<<"enclosure["<<i<<"]: "<<enclosure[i]<<"\n";
						return false;
					}
				}
			}
		}else return false;
	}else ROS_WARN("enclosure has less than 3 vertices");
	
	//std::cout<<"is inside \n \n";
	return true;
}

bool withinBounds(geometry_msgs::Point point, std::vector<geometry_msgs::Point> enclosure, double margin){
	double x_min, x_max, y_min, y_max;
	if(enclosure.size() > 2){
		x_max = enclosure[0].x;
		x_min = enclosure[0].x;
		y_max = enclosure[0].y;
		y_min = enclosure[0].y;
	}else {
		ROS_WARN("enclosure has less than 3 vertices");
		return false;
	}
	for(int i = 0; i < enclosure.size(); i++){
		if(enclosure[i].x < x_min) x_min = enclosure[i].x; 
		if(enclosure[i].x > x_max) x_max = enclosure[i].x; 
		if(enclosure[i].y > y_max) y_max = enclosure[i].y; 
		if(enclosure[i].y < y_min) y_min = enclosure[i].y; 
	}
	x_min -= margin;
	x_max += margin;
	y_min -= margin;
	y_max += margin;
	
	if((point.x > x_min) && (point.x < x_max) && (point.y > y_min) && (point.y < y_max)){
		return true;
	}else return false;
}

geometry_msgs::Pose findFree(geometry_msgs::Pose target){

	//getOdom();
	std::vector<geometry_msgs::Point> options = gl_->getPoints(target.position, robot_pose.position, 0.3);
	for(int i = 2; i < options.size(); i++){
		if(!obstacleOnMap(options[i], 0.2)){ 
			target.position = options[i];
			return target;
		}
	}
	
	geometry_msgs::Point far_away = 
		pointOnLine(robot_pose.position, target.position, target.position, longest_line, false);
	options = gl_->getPoints(robot_pose.position, far_away, 0.5);
	for(int i = 0; i < options.size(); i++){
		if(!obstacleOnMap(options[i], 0.2)) {
			target.position = options[i];
			return target;
		}
	}
	return target;
}

void updateCheckedFeatures(int id){
	bool add_to = true;
	
	for(int i =0; i < checked_features.size(); i++){
		if(checked_features[i] == id) add_to = false; 
	}
	if(add_to) checked_features.push_back(id);
	
	removeAroundFeatures(id);
}

bool featureChecked(int id){
	for(int i =0; i < checked_features.size(); i++){
		if(checked_features[i] == id) return true; 
	}
	return false;
}

double getZFromQuat(geometry_msgs::Quaternion quat){
	double r, p, y;
	tf::Quaternion tf_quat;
	quaternionMsgToTF(quat, tf_quat);
	tf_quat.normalize();
	tf::Matrix3x3 m(tf_quat);
	m.getRPY(r, p, y);
	return y;
	
}

bool onLine(geometry_msgs::Point p1, geometry_msgs::Point p2){
	//getOdom();
	
	geometry_msgs::Point rob_p = robot_pose.position;
	geometry_msgs::Point cl_p = closestPointOnLine(p1, p2, rob_p, g_margin/3);

	if(gl_->equalPoints(robot_pose.position, cl_p, g_margin*2)){
		return true;
	}
		
	return false;
}

bool initializeFollow(std::string *dir, double hold_at, geometry_msgs::Point obs){
	double front_clearance = hold_at * 2;
	getOdom();
	closenessCheck(xy_tol*2);
	std::cout<<"obs 4 initialize: "<<obs<"\n";
	//if(dir != "left"  && dir != "right") ROS_WARN("Direction in wall follow not set, set 'left' or 'right'");
	double del_dist = 0.5;
	
	geometry_msgs::Pose right_g = firstWallGoal("right", obs, hold_at);
	double len_r = del_dist;
	while(len_r < sensor_range){
		geometry_msgs::Pose inch_f = translateLinear(right_g, len_r);
		if(obstacleOnMap(inch_f.position, 0.1)){
			//std::cout<<"right break at: "<<inch_f.position<<"\n";
			break;
		}
		len_r += del_dist;
	}
	
	geometry_msgs::Pose left_g = firstWallGoal("left", obs, hold_at);
	double len_l = del_dist;
	while(len_l < sensor_range){
		geometry_msgs::Pose inch_f = translateLinear(left_g, len_l);
		if(obstacleOnMap(inch_f.position, 0.1)){
			//std::cout<<"left break at: "<<inch_f.position<<"\n";
			break;
		}
		len_l += del_dist;
	}
	
	//pick dir with more obs in front, likely more info round d corner 
	if(len_l < len_r){
		std::cout<<"left follow \n";
		*dir = "left";
	}else{*dir = "right"; std::cout<<"right follow \n";}
	
	//TODO set side blocked
	for(int i = 0; i < feature_edges[active_feature].size(); i++){
		if(!isInside(feature_edges[active_feature][i], global_vertices)){
			std::cout<<"reset direction /n";
			double lo_dist  = gl_->getLength(feature_edges[active_feature][i], 
									translateLinear(left_g, 1).position);
			double ro_dist  = gl_->getLength(feature_edges[active_feature][i], 
									translateLinear(right_g, 1).position);
			if(lo_dist < ro_dist){
				*dir = "right";
				std::cout<<"right follow \n";
			}else{*dir = "left"; std::cout<<"left follow \n";}
			edge_switch = false;	
			break;
		}
	}
		
	geometry_msgs::Pose goal;
	goal = firstWallGoal(*dir, obs, hold_at); 
	
	int count = 0;
	stuck_time = ros::Time::now().toSec();
	geometry_msgs::Point last_pos  = robot_pose.position;
	geometry_msgs::Point *last_pos_p  = &last_pos;
	
	
	goToAction(goal, 3);
	
	double fc = getFrontScan();
	if(fc < front_clearance){
		if(*dir == "left") goal = gl_->rotatePose(goal, -pi/6); //30 deg CW
		else goal = gl_->rotatePose(goal, pi/6);
		
		goToAction(goal, 2);
	}
	obstacle = obs;
	createHalfLine(*dir);
	return true;
}

geometry_msgs::Point closestPointOnLine(
	geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point ref, double margin){
	int count = 0;
	double l1 = gl_->getLength(p1, ref);
	double l2 = gl_->getLength(p2, ref);
	while(gl_->getLength(p1, p2) > margin || count < 50){
		if(l1 < l2){
			p2 = midPoint(p1, p2);
		}else if(l1 > l2){
			p1 = midPoint(p1,p2);
		}else break;
		l1 = gl_->getLength(p1, ref);
		l2 = gl_->getLength(p2, ref);
		count++;
	}
	return midPoint(p1,p2);
}

bool crossedEdges(std::string* dir_p, bool *other_side, 
	std::vector<geometry_msgs::Point> *traj, geometry_msgs::Point* start, int feature_index){
	
	geometry_msgs::Point wall_ref;
	bool trigger = false;
	double margin = 2; //move away b4 trigger
		
		//if inside exploration area
		bool feat_inside = featureIsInsideArea(feature_index);
		
		//start_inside only changes once
		if(start_inside && !feat_inside && (!isInside(robot_pose.position, global_vertices)) ){
			trigger = true;
		}
	
	
	if(edge_switch){
		if(start_inside && !feat_inside){
			std::cout<<"feature outside \n";
			geometry_msgs::Point p1, p2;
			int gv_size = global_vertices.size();
		
			p1 = global_vertices[gv_size - 1]; p2 = global_vertices[0];
			geometry_msgs::Point wall_exit = 
			closestPointOnLine(global_vertices[gv_size - 1], global_vertices[0], robot_pose.position, 0.5);
		
			for(int i = 0; i < global_vertices.size() - 1; i++){
				geometry_msgs::Point interim_sh = 
					closestPointOnLine(global_vertices[i], global_vertices[i + 1], robot_pose.position, 0.5);
				if(gl_->getLength(interim_sh, robot_pose.position) < gl_->getLength(wall_exit, robot_pose.position)){
					wall_exit = interim_sh;
					p1 = global_vertices[i]; p2 = global_vertices[i + 1];
				}
			}
			
			int cl_edge = 0; // closest feature edege to exit
			double cl_dist = gl_->getLength(wall_exit, feature_edges[feature_index][cl_edge]);
			for(int i = 1; i < feature_edges[feature_index].size(); i++){
				if(gl_->getLength(wall_exit, feature_edges[feature_index][i]) < cl_dist){
					cl_edge = i;
				}
			}
			
			//for when the object does not extend outside on closest side
			if(!isInside(feature_edges[feature_index][cl_edge], global_vertices)){
				//if feat is outside, get closest point on wall, if obs is concave, if obs is continuos
				double rob_z = getZFromQuat(robot_pose.orientation);
				double mp_z = getZFromQuat(gl_->getQuatFromP(robot_pose.position, wall_exit));
				double dist_to_acw = gl_->getLength(robot_pose.position, wall_exit);
				double angle_tol = 0.2; // +/- 10sh deg
				if(std::abs(rob_z - mp_z) < angle_tol){ 
					if((getFrontScan()) >= dist_to_acw){
						 trigger = true;
					}
				} 
			}
		}
	}else{
		if(!isInside(robot_pose.position, global_vertices)){
			return true;
		}
	}
			
		if(trigger){
			std::cout<<"Triggered \n";
			if(edge_switch){			
				if(goToAction(gl_->rotatePose(robot_pose, pi), 3)){
					if(*dir_p == "left") *dir_p = "right";
					else if(*dir_p == "right") *dir_p = "left";	
					std::cout<<"switched \n";
					*other_side = false;
					*start = robot_pose.position;
					traj->erase(traj->begin(), traj->end());
					edge_switch = false;
					//edge_one = true;
					obstacle = half_line[0];
					createHalfLine(*dir_p);
					edge_one_point = robot_pose.position;				
				}	
				trigger = false;
			}	
		}		
	return false;
} 

/*bool continuousObject(std::string dir){
	ros::spinOnce();
	std::cout<<"cOb \n";
	if(dir != "left"  && dir != "right"){
		ROS_WARN("Direction in wall follow not set.Use 'left' or 'right'");
		return false;
	}else{
		int num_scans = std::floor((scans->angle_max - scans->angle_min) / scans->angle_increment);
		int start = num_scans / 2; //mid scan
		//if side is has a continuos obs
		double group_scans = pi/180; //group every 1 degree
		int no_in_group = group_scans / scans->angle_increment;
		std::cout<<"group_scans: "<<group_scans<<"\n";
		std::cout<<"no_in_group: "<<no_in_group<<"\n";
		double prev_scan = scans->ranges[start];
		if(prev_scan > scans->range_max) prev_scan = scans->range_max; //if infinte
		double ang_margin = 0.77; //45 degree change 
		double range_diff = (1 - cos(ang_margin + scans->angle_increment)); //range*range_diff = obs changes > ang_margin 
		std::cout<<"range_diff: "<<range_diff<<"\n";
		if(num_scans > (no_in_group * 2)){
			while((start > no_in_group) && (start < (num_scans- no_in_group))){ //scan 45			
				if(dir == "left") start --;
				else if(dir == "right") start ++;
				else break;
				
				double curr_scan = 0;
				for(int i = 0; i < no_in_group; i ++){
					if(scans->ranges[start + i]  > scans->range_max){
						curr_scan += scans->range_max; //if infinte
					}else curr_scan += scans->ranges[start + i];
					
				}
				curr_scan = curr_scan / no_in_group;
				//some sort of gap/average
				//maybe averga in 3s, if average > max stop; if sharp change stop
				if((curr_scan > (1 + range_diff)*prev_scan) || 
				   (curr_scan < (prev_scan*range_diff)) ){
					std::cout<<"sc["<<start<<"]: "<<curr_scan<<"\n";
					std::cout<<"prev_scan: "<<prev_scan<<"\n";
					return false;
				}
				
				prev_scan = curr_scan;
			}
		}
		
		return true;
	}
}*/

bool holdGoalObserve(geometry_msgs::Pose goal){

	//ros::Rate snooze(0.5);
	setBlindGoal(goal, 0);
	//snooze.sleep();
	int mb_status;
	float t_start = ros::Time::now().toSec();
	
	getOdom();
	//geometry_msgs::Point last_pos  = robot_pose.position;
	//geometry_msgs::Point *last_pos_p  = &last_pos;
	
	while(gl_->getLength(goal.position, robot_pose.position) > plan_tol){	
		closenessCheck(plan_tol);
		ros::spinOnce();
		getOdom();
		updateCostmap(o_grid);
		//if(stuck(last_pos_p, 10)) unStick(1, 1);

		if(obstacleOnPoint(goal.position)){//Redundancy
			goal = findFree(goal);
			setBlindGoal(goal, 0);
			//snooze.sleep();
			if(obstacleOnPoint(goal.position)) return false;
		}

		if(borderHijack()) return false;

		if(goal_status_->status_list.size() > 0){
			mb_status = goal_status_->status_list[goal_status_->status_list.size() - 1].status;
		}

		if(mb_status == 4 || mb_status == 5 || mb_status == 9){
			goal = findFree(goal);
			setBlindGoal(goal, 0);	
			return false;
		}else if((mb_status == 3) && ((ros::Time::now().toSec() - t_start) > 30)){//Hot Fix
			break; 
		}

		setBlindGoal(goal, 10); //refresh goal every 10 secs
	}

	return true;
}

bool closeToPlan(std::vector<int> bdg){
	
	for(int v = 0; v < bdg.size(); v++){
		int gwf = bdg[v];
		//check goal and everything b4 it
		if(plan_points.size() > 0){
			if(plan_points[0] >= gwf){
				std::cout<<"popped by ori_plan["<<gwf<<"]: "<<ori_plan[gwf]<<"\n";
				return true;
			}
		}	
	}
	return false;
}

geometry_msgs::Point closestPointOnFeat(int feat_id){
	double obs_dist = gl_->getLength(robot_pose.position, features[feat_id][0]);
	geometry_msgs::Point obs_edge = features[feat_id][0];
	
	//TODO use sorted format
	for(int i2 = 0; i2 < features[feat_id].size(); i2 ++){
		if(gl_->getLength(robot_pose.position, features[feat_id][i2]) < obs_dist){
			obs_dist = gl_->getLength(robot_pose.position, features[feat_id][i2]);
			obs_edge = features[feat_id][i2];
		}
	}
	std::cout<<"obs_edge: "<<obs_edge<<"\n";
	return obs_edge;
}

bool borderHijack(){

	getOdom();
	int c_index;

	if(to_be_explored.size() > 0) orderToBeExplored();
	for(int i = 0; i< to_be_explored.size(); i++){
		int feat_id = to_be_explored[i];
		
		if(!featureChecked(feat_id) ){  
			if(closeToPlan(border_goals[i])){
				borderCheck(feat_id);
				i = -1; //to_be_e might have changed
				return true;
			}
		}
	}
	return false;
}

bool borderCheck(int feature_index){
	std::cout<<"borderCheck :"<<feature_index<<"\n";
	active_feature = feature_index;
	if(featureCleared(feature_index)){
		
		for(int i = 0; i < to_be_explored.size(); i++){
			if(to_be_explored[i] == feature_index){
				updateCheckedFeatures(feature_index);
				continueOnPlan();
				return false;
			}
		}	
	}
		
	geometry_msgs::Pose goal;

	std::string dir = "right";
	std::string *dir_p = &dir;
	//edge_one = false;
	edge_switch = true;
	
	bool not_moving = false;
	bool other_side = false;
	bool *other_side_p = &other_side;
	bool finished = false;
	double g_rate = 0.5;
	double hold_at_ini = clear_dist * 0.5;
	double line_inflation = 0.1;
	
	geometry_msgs::Point obs_edge = closestPointOnFeat(feature_index);
	if(!initializeFollow(dir_p, hold_at_ini, obs_edge)) return false;
	
	
	geometry_msgs::Point start_inv  = robot_pose.position;
	geometry_msgs::Point *start_inv_p  = &start_inv;
		
	std::vector<geometry_msgs::Point> traj;
	std::vector<geometry_msgs::Point>* traj_p = &traj;
	goal = escapeWall(dir, 1);
	//stuck_time = ros::Time::now().toSec();
	//geometry_msgs::Point last_pos  = robot_pose.position;
	//geometry_msgs::Point *last_pos_p  = &last_pos;
	while(!finished){
		closenessCheck(plan_tol);
		if(featureCleared(active_feature)) finished = true;
		if(repeating(traj_p)) finished = true;
		if(crossedEdges(dir_p, other_side_p, traj_p, start_inv_p, active_feature)) finished = true;
		updateCostmap(o_grid); //has getOdom();
		goal = escapeWall(dir, clear_dist);
		setBlindGoal(goal, wg_rate);
		//if(stuck(last_pos_p, 7)) unStick(0.5, 1);
		
				
		if(!other_side){
			if(onLine(half_line[0], half_line[1])) other_side = true;
		}

		if(other_side){
			if(!obstacleOnLineMap(robot_pose.position, *start_inv_p, line_inflation)){
				finished = true;
			}		
		}	
		
		
		if(finished){
			std::cout<<"End bCheck :"<<active_feature<<"\n";
			edge_switch = true;
			//edge_one = true; //when dir change is not popped
			updateCheckedFeatures(active_feature);
			continueOnPlan();
			
		}
	}
	return true;
}

void continueOnPlan(){
	if(gl_->global_plan.poses.size() > 1){
		if(!obstacleOnMap(gl_->global_plan.poses[0].pose.position, g_inflation)){
			holdGoalObserve(gl_->global_plan.poses[0].pose);  
		}else{
			erasePlanPoint(0);
			if(!obstacleOnMap(gl_->global_plan.poses[0].pose.position, g_inflation)){
				holdGoalObserve(gl_->global_plan.poses[0].pose); 
			}else setBlindGoal(gl_->global_plan.poses[0].pose, 0);
		}
	}
}

void removeAroundFeatures(int feature_index){
	/*for(int j = 0; j < feature_edges[feature_index].size(); j++){
		std::cout<<"RMA feature_edges[feature_index]:"<<feature_edges[feature_index][j]<<"\n";
	}*/

	for(int i = 0; i < gl_->global_plan.poses.size(); i++){
		if(withinBounds(gl_->global_plan.poses[i].pose.position, feature_edges[feature_index], 0.5)){
			//std::cout<<"remove around :"<<gl_->global_plan.poses[i].pose.position<<"\n";
			erasePlanPoint(i);
			i--;
		}
	}
}

void closenessCheck(double margin){
	for(int i4 = 0; i4 <  gl_->global_plan.poses.size(); i4++){
		if(gl_->equalPoints(robot_pose.position,  gl_->global_plan.poses[i4].pose.position, margin)){
			erasePlanPoint(i4);
		}
	}
}

bool repeating(std::vector<geometry_msgs::Point> *steps_rec){

	double rec_rate = 1;
	double diff = 1;
	if((ros::Time::now().toSec() - rp_time) > rec_rate){	
		rp_time = ros::Time::now().toSec();
		if(steps_rec->size() > 0){
			if(gl_->getLength(robot_pose.position, steps_rec->at(steps_rec->size()-1)) > 0.5){
				steps_rec->push_back(robot_pose.position);
			}
		}else steps_rec->push_back(robot_pose.position);
		if(steps_rec->size() > 10){
			for(int i = 0; i < (steps_rec->size() - 9); i++){
				if(gl_->equalPoints(robot_pose.position, steps_rec->at(i), diff)){
					return true;
				}
			}
		}
	}
	
	return false;
}

bool stuck(geometry_msgs::Point* last_pos, double rec_rate){
	
	if((ros::Time::now().toSec() - stuck_time) > rec_rate){	
		stuck_time = ros::Time::now().toSec();
		if(gl_->getLength(robot_pose.position, *last_pos) < 0.05){
			std::cout<<"stuck at: "<<*last_pos<<"\n";
			*last_pos = robot_pose.position;
			return true;
		}
		*last_pos = robot_pose.position;
		//std::cout<<"last pose updated: "<<*last_pos<<"\n";
	}
	
	return false;
}

void orderToBeExplored(){	
	std::vector<int>  tbe;
	std::vector<std::vector<int> > goal_closest_to; //f_index and weight
	border_goals.erase(border_goals.begin(), border_goals.end());
	
	for(int i = 0; i < to_be_explored.size(); i++){//find closest goal and push
		int feat_index = to_be_explored[i];
		std::vector<int> smth; //feat_id and closest goal in oriplan
		int last_index;
		double closest;
		
		
		smth.push_back(feat_index);
		smth.push_back(ori_plan.size() - 1);//initialize as least priority
		last_index = ori_plan.size() - 1;
		closest = gl_->getLength(feature_edges[feat_index][0], ori_plan[last_index]);
		
		if(!featureChecked(feat_index)){
			//smth[1] is the earliest goal on ori plan				
			for(int j = 0; j < feature_edges[feat_index].size(); j++){			
				for(int k = 0; k < ori_plan.size(); k++){
					if(gl_->getLength(feature_edges[feat_index][j], ori_plan[k]) < closest){
						closest = gl_->getLength(feature_edges[feat_index][j], ori_plan[k]);
						smth[1] = k; //goal on ori_plan that triggers border check

					}
				}
			}
		}
		goal_closest_to.push_back(smth);
	}
	
	border_goals.resize(to_be_explored.size());
	for(int k = 0; k < to_be_explored.size(); k++){
		int smallest = ori_plan.size() - 1; //initialize as biggest possible
		int s_id = 0; // where g_c_t ini
		for(int i = 0; i < goal_closest_to.size(); i++){
			if(goal_closest_to[i][1] < smallest){
				smallest = goal_closest_to[i][1];
				s_id = i;
			}
		} 
		tbe.push_back(goal_closest_to[s_id][0]);
		border_goals[k].push_back(goal_closest_to[s_id][1]);
		goal_closest_to.erase(goal_closest_to.begin() + s_id);
	}
	to_be_explored = tbe;
	
	for(int i = 0; i < to_be_explored.size(); i++){
		int feat_id = to_be_explored[i];
		double expand = 0.5;
		if(!featureChecked(feat_id)){
			for(int k = 0; k < ori_plan.size(); k++){
				if(withinBounds(ori_plan[k], feature_edges[feat_id], expand)){
					border_goals[i].push_back(k);
				}
			}
		}
	}
	
}

bool setBlindGoal(geometry_msgs::Pose goal_in, double rate){
	std::cout<<"sBG -";
	t_now =ros::Time::now().toSec();
	double t_diff = t_now - last_time;
	if(t_diff >= rate){

		last_time = t_now; 
		geometry_msgs::PoseStamped goal; 
		goal.header.frame_id = map_frame;
		goal.header.stamp = ros::Time::now();
		goal.pose = goal_in;
		goal_publisher.publish(goal);
		std::cout<<"gpub -";
		int mb_status;

		ros::spinOnce();
		if(goal_status_->status_list.size() > 0){
			mb_status = goal_status_->status_list[goal_status_->status_list.size() - 1].status;
		}
		ros::Rate snooze(0.5);
		if(mb_status == 4 || mb_status == 5 || mb_status == 9){
			ROS_WARN("Move_base Cant process goal; trying fix");
			snooze.sleep();
			//goal.header.stamp = ros::Time::now();
			moveCloser(goal_in, plan_tol);
			snooze.sleep();
			return false;
		}
		goal_record = goal_in;
		return true;
	}else{
		if(t_diff > 10) last_time = t_now;
		return false;
	}
}

void createHalfLine(std::string dir){
	geometry_msgs::Point end;
	geometry_msgs::Point start;
	double remove_length = g_inflation * 1.5;

	if(getOdom()){
		end = pointOnLine(obstacle, robot_pose.position, robot_pose.position, longest_line, true);	

		geometry_msgs::Quaternion quat = gl_->getQuatFromP(obstacle, end);
		geometry_msgs::Pose int_pose;
		int_pose.position = obstacle;
		int_pose.orientation = quat;
		if(dir == "right") int_pose = gl_->rotatePose(int_pose, -pi/4); // 45 deg CW
		else if(dir == "left") int_pose = gl_->rotatePose(int_pose, pi/4); // 45 deg ACW
		
		//rotate by z
		double z = getZFromQuat(int_pose.orientation);
		end.x = obstacle.x + longest_line*cos(z);
		end.y = obstacle.y + longest_line*sin(z);
	
		//remove first meter inflation cost
		start = pointOnLine(obstacle, end, robot_pose.position, remove_length, true);
		half_line[0] = start;
		half_line[1] = end;
	}
}

geometry_msgs::Point pointOnLine(
	geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point ref, double dist, bool reverse){
	
	geometry_msgs::Point p_out;	
	//distance measured from p1
	double grad = (p1.y - p2.y) / (p1.x - p2.x);	
	double ppx1 = p1.x + sqrt(pow(dist, 2) / (1 + pow(grad, 2)));
	double ppy1 = grad * (ppx1 - p1.x) + p1.y;
	double ppx2 = p1.x - sqrt(pow(dist, 2) / (1 + pow(grad, 2)));
	double ppy2 = grad * (ppx2 - p1.x) + p1.y;
	
	double ref_dist_to_p1 = sqrt(pow((ppy1 - ref.y), 2) + pow((ppx1 - ref.x), 2));
	double ref_dist_to_p2 = sqrt(pow((ppy2 - ref.y), 2) + pow((ppx2 - ref.x), 2));
	
	if(!reverse){	
		if(ref_dist_to_p1 < ref_dist_to_p2){
			p_out.x = ppx1; 
			p_out.y = ppy1; 
		}else{
			p_out.x = ppx2; 
			p_out.y = ppy2;
		}
	}else{
		if(ref_dist_to_p1 > ref_dist_to_p2){
			p_out.x = ppx1; 
			p_out.y = ppy1; 
		}else{
			p_out.x = ppx2; 
			p_out.y = ppy2;
		}
	}

	return p_out;
} 

geometry_msgs::Pose firstWallGoal(std::string direction, geometry_msgs::Point obs, double hold_at){
	geometry_msgs::Pose goal;
	getOdom();
	goal.orientation = gl_->getQuatFromP(robot_pose.position, obs);//set orien towards obs
	
	double len_obs = gl_->getLength(robot_pose.position, obs);
	if((len_obs < hold_at) && (!obstacleOnMap(robot_pose.position, robot_span/2))){	
		goal.position = robot_pose.position;
		goal.orientation = gl_->getQuatFromP(robot_pose.position, obs);
		
		if(direction == "right"){
			goal =  gl_->rotatePose(goal, 7*pi/18); //70 deg ACW
		}else if(direction == "left"){
			goal =  gl_->rotatePose(goal, -7*pi/18);
		}
	}else{//fix orientation left or right
		goal.position = pointOnLine(obs, robot_pose.position, robot_pose.position, hold_at, false);
		while(obstacleOnPoint(goal.position) && hold_at < (sensor_range/2)){
			goal.position = pointOnLine(obs, robot_pose.position, robot_pose.position, hold_at, false);	
			hold_at += 0.2;
		}
		if(direction == "left"){
			goal = gl_->rotatePose(goal, -7*pi/18); //70 deg CW
		}else goal = gl_->rotatePose(goal, 7*pi/18);
	}
		
	//std::cout<<"first wall goal: "<<goal.position<<"\n";
	//std::cout<<"Z goal: "<<getZFromQuat(goal.orientation)<<"\n";
	
	return goal;
}

double getFrontScan(){
	ros::spinOnce();
	int mid_index = ((scans->angle_max - scans->angle_min)/ (2*scans->angle_increment));
	int one_degree = pi/(180*scans->angle_increment); //range
	double sum = 0;
	int count = 0;	
	
	if((scans->angle_max < 0) || (scans->angle_min > 0)){
		ROS_WARN("Laser data requested out of scope");
		return 0;
	}
	
	for(int i = (mid_index - one_degree); i < (mid_index + one_degree); i++){ 
		if(scans->ranges[i] > sensor_range) scans->ranges[i] = sensor_range;
		sum += scans->ranges[i];
		count++;
	}
	return (sum/count);
}

void updateCostmap(nav_msgs::OccupancyGrid og){
	double start_time =ros::Time::now().toSec();
	ros::spinOnce();
	
	getOdom();
	tf::TransformListener tom;
	tom.waitForTransform(odom_frame, map_frame,ros::Time(0), ros::Duration(2.0));
	geometry_msgs::PointStamped point_out;
	geometry_msgs::PointStamped point_in;
	std::vector<geometry_msgs::Point> obstacle_points;
	std::vector<geometry_msgs::Point> points_edge;
	point_in.header.frame_id = og.header.frame_id;
	point_out.header.stamp = ros::Time::now();
	
	bool too_close = false; 
	int threshold = 75;
	double map_margin = g_margin;
	int count = 0;
	int margin_count_max = map_margin / og.info.resolution;
	int row_count =  og.info.width / og.info.resolution; //No. of point in row
	
	for(int i = 0; i < og.data.size(); i++){
		if(og.data[i] > threshold){
			point_in.point.x = (i % og.info.width)*og.info.resolution + og.info.origin.position.x;
			point_in.point.y = (i /  og.info.width)*og.info.resolution + og.info.origin.position.y;
			try{
				tom.transformPoint(map_frame, point_in, point_out);
				obstacle_points.push_back(point_out.point);
				
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}

			if(count > margin_count_max){//min space btw points
				count = 0;
				/*if((i > row_count + 1) && (i < og.data.size() - row_count -1)){ //error block for firstand last row
					if(og.data[i+1] < threshold || og.data[i-1] < threshold ||
					   og.data[i + row_count] < threshold || og.data[i - row_count] < threshold){//either side of data
						points_edge.push_back(point_out.point);
					}
				}*/
				if(og.data[i] >= 99) points_edge.push_back(point_out.point);
			}
		
		}
			
		count ++;
	}
	
	obstacles = obstacle_points; 
	edge_obstacles = points_edge;
	//objects_map_p->updateMap(points_edge, active_feature);
	matchFeatures();
	//std::cout<<"features.size(): "<<features.size()<<"\n";
	//std::cout<<"obstacles.size(): "<<obstacles.size()<<"\n";
	//std::cout<<"edge_obstacles.size(): "<<edge_obstacles.size()<<"\n";
	/*for(int i = 0; i < features.size(); i++){
		for(int j = 0; j < features[i].size(); j++){
			std::cout<<"f["<<i<<"]["<<j<<"]: "<<features[i][j]<<"\n";
		}
	}*/
	
	double finish_time =ros::Time::now().toSec();
	if((finish_time - start_time) > 1){
		std::cout<<"UCM Time: "<<finish_time - start_time<<" \n";
	}
}

void matchFeatures(){
	double mf_strt_time =ros::Time::now().toSec();
	geometry_msgs::Point zero;
	zero.x = 0;
	
	bool initial = true;	
	double space_btw_points = 0.2;
	if(edge_obstacles.size() > 0 && features.size() < 1){
		std::vector<geometry_msgs::Point> new_feature;
		new_feature.push_back(edge_obstacles[0]);
		features.push_back(new_feature);
	}
	
	for(int i = 0; i < edge_obstacles.size(); i++){	

		std::vector<int> add_to; //add point to this feature
		std::vector<std::vector<int> > pos_vec;
		std::vector<std::vector<int> > *pos_vec_p = &pos_vec;
		bool sorted;
		bool *sorted_p = &sorted;

		add_to = possiblyInFeature(edge_obstacles[i], pos_vec_p, sorted_p);	

		if(add_to.size() == 0){
			std::vector<geometry_msgs::Point> new_feature2;
			new_feature2.push_back(edge_obstacles[i]);
			features.push_back(new_feature2);
		}else{
			
			if(add_to.size() > 1){ //only two expected
				std::vector<int>  temp_pos;
				
				if(add_to[1] == active_feature){//if active_feat is involved
					add_to[1] = add_to[0];
					add_to[0] = active_feature;
					
					temp_pos = pos_vec[1];
					pos_vec[1] = pos_vec[0];
					pos_vec[0] = temp_pos;
				}else{
					if(add_to[0] != active_feature){ //if not active find the smaller feat
						if((features[add_to[1]].size()) > (features[add_to[0]].size())){
							int temp_sn = add_to[0];
							add_to[0] = add_to[1];
							add_to[1] = temp_sn;

							temp_pos = pos_vec[0];
							pos_vec[0] = pos_vec[1];
							pos_vec[1] = temp_pos;
						}
					}
				}
				//std::cout<<"insert and merge \n";
				//std::cout<<"min feat["<<add_to[0]<<"]["<<pos_vec[0][0]<<"]: "<<features[add_to[0]][pos_vec[0][0]]<<"\n";
				//std::cout<<"min feat["<<add_to[0]<<"]["<<pos_vec[0][1]<<"]: "<<features[add_to[0]][pos_vec[0][1]]<<"\n";
				
				//std::cout<<"merge To \n";
				//std::cout<<"min feat["<<add_to[1]<<"]["<<pos_vec[1][0]<<"]: "<<features[add_to[1]][pos_vec[1][0]]<<"\n";
				
				for(int i3 = (add_to.size() - 1); i3 > 0; i3--){//only two; incase of scaling
					//std::cout<<"double from pos_vec: "<<pos_vec[0]<<"\n";
					if(sorted){
						insertToFeature(add_to[0], edge_obstacles[i], pos_vec[0][0], pos_vec[0][1], pos_vec[0][2]);
					}else {
						addToFeature(add_to[0], edge_obstacles[i]);
					}
					//addToFeature(add_to[0], edge_obstacles[i]);
					mergeFeatures(add_to[0], add_to[i3]);
				}
			}else{
				//std::cout<<"from pos_vec: "<<pos_vec[0]<<"\n";
				//std::cout<<"insert \n";
				//std::cout<<"min feat["<<add_to[0]<<"]["<<pos_vec[0][0]<<"]: "<<features[add_to[0]][pos_vec[0][0]]<<"\n";
				//std::cout<<"min feat["<<add_to[0]<<"]["<<pos_vec[0][1]<<"]: "<<features[add_to[0]][pos_vec[0][1]]<<"\n";
		
				if(sorted){
					insertToFeature(add_to[0], edge_obstacles[i], pos_vec[0][0], pos_vec[0][1], pos_vec[0][2]);
				}else{
					addToFeature(add_to[0], edge_obstacles[i]);
				}
				//addToFeature(add_to[0], edge_obstacles[i]);
			}			
		}
	}	
	//std::cout<<"edge_obstacles.size(): "<<edge_obstacles.size()<<"\n";
	//std::cout<<"features.size()"<<features.size()<<"\n";
	//double edge_obs_end =ros::Time::now().toSec();
	//std::cout<<"edge_obs time "<<ros::Time::now().toSec() - mf_strt_time<<"\n";
	/*if(first_run){
		std::cout<<"fist TIme #$%^&*() \n";
		for(int i = 0; i < features.size(); i++){
			for(int j = 0; j < features[i].size(); j++){
				std::cout<<"1stt feature["<<i<<"]["<<j<<"]: "<<features[i][j]<<"\n";
			}
		}
	}*/
	if(first_run) first_run = false;
	cm_count++;
	/*if(cm_count == 2){
		std::cout<<"2nd TIme #$%^&*() \n";
		for(int i = 0; i < features.size(); i++){
			for(int j = 0; j < features[i].size(); j++){
				std::cout<<"2nd feature["<<i<<"]["<<j<<"]: "<<features[i][j]<<"\n";
			}
		}
	}*/
	int wall_out_id;
	if(plan_points.size() > 0){
		wall_out_id =  gl_->loop_sn[plan_points[0]];
	}else wall_out_id =  gl_->loop_sn[ori_plan.size() - 1];
	
	std::vector<geometry_msgs::Point> outer_wall = gl_->all_walls[wall_out_id]; 
	int gv_size = outer_wall.size();
	
	std::vector<geometry_msgs::Point> inner_wall;
	bool inside_inner = true;
	if((wall_out_id+1) < gl_->all_sections.size()){
		inner_wall = gl_->all_sections[wall_out_id + 1]; 	
	}else{
		inside_inner = false;
	}
	
	generateEdges();
	//Mark features to be explored
	for(int i = 0; i < feature_edges.size(); i++){
		bool utbe = true; //close to outer wall
		bool sig = true; //is on inner side and not barely in
		if(featureClose(i)){
			for(int t = 0; t < feature_edges[i].size(); t++){
				geometry_msgs::Point ref_point = feature_edges[i][t];
				geometry_msgs::Point p1, p2;

				//if inside explrtion area and not on wall (margin inside)			
				p1 = outer_wall[gv_size - 1]; p2 = outer_wall[0]; //initialize closest edge to wall
				geometry_msgs::Point near_point = 
					closestPointOnLine(outer_wall[gv_size - 1], outer_wall[0], ref_point, 0.5);
				for(int i4 = 0; i4 < outer_wall.size() - 1; i4++){
					geometry_msgs::Point interim_sh = 
						closestPointOnLine(outer_wall[i4], outer_wall[i4 + 1], ref_point, 0.5);
					if(gl_->getLength(interim_sh, ref_point) < gl_->getLength(near_point, ref_point)){
						near_point = interim_sh;
						p1 = outer_wall[i4]; p2 = outer_wall[i4 + 1]; //vertice not used anw else
					}
				}

				bool inside_outer = isInside(ref_point, outer_wall); 
				if(inside_inner) inside_inner = isInside(ref_point, inner_wall);

				if((gl_->getLength(ref_point, near_point) > sensor_range) || inside_inner){ //distance to boundary
					utbe = false;
				}
				if((gl_->getLength(ref_point, near_point) < 1) || !inside_outer) sig = false;
			}
			if(utbe && sig) updateToBeExplored(i);
		}
	}
	
	//std::cout<<"tbe time "<<ros::Time::now().toSec() - edge_obs_end<<"\n";
	//std::cout<<"mf time "<<ros::Time::now().toSec() - mf_strt_time<<"\n";
	
}

void generateEdges(){
	feature_edges.resize(features.size()); //create edges of features **&*&* Display this at checked features
	for(int i = 0; i < features.size(); i++){
		feature_edges[i].resize(4); 
		
		for(int c = 0; c < 4; c++) feature_edges[i][c] = features[i][0];
		
		//TODO sorted dont have to go through entire feature
		for(int ii = 0; ii < features[i].size(); ii++){//clock wise left top ...
			if(features[i][ii].x < feature_edges[i][0].x) feature_edges[i][0] = features[i][ii];  
			if(features[i][ii].y > feature_edges[i][1].y) feature_edges[i][1] = features[i][ii]; 
			if(features[i][ii].x > feature_edges[i][2].x) feature_edges[i][2] = features[i][ii]; 
			if(features[i][ii].y < feature_edges[i][3].y) feature_edges[i][3] = features[i][ii]; 	
		}
		feature_edges[i][0].x -= g_margin;
		feature_edges[i][1].y += g_margin;
		feature_edges[i][2].x += g_margin;
		feature_edges[i][3].y -= g_margin;
	}

	if(active_feature != -1){
		geometry_msgs::PolygonStamped ac_poly = gl_->createPoly(feature_edges[active_feature]);
		ac_poly_pub.publish(ac_poly);
	}
	
}

bool closePoints(geometry_msgs::Point p1, geometry_msgs::Point p2, float margin){
	if ((std::abs(p1.x - p2.x) < margin) && (std::abs(p1.y - p2.y) < margin)){
		return true;			
	}
	else{
		return false;
	}
}

void insertToFeature(int feat_index, geometry_msgs::Point point, int min_sn, int max_sn, int mid_sn){
	//std::cout<<"try to insert: "<<point<<" \n";
	//std::cout<<"btw min f["<<feat_index<<"]["<<min_sn<<"]: "<<features[feat_index][min_sn]<<" \n";
	//std::cout<<"btw max f["<<feat_index<<"]["<<max_sn<<"]: "<<features[feat_index][max_sn]<<" \n";
	bool ins = true;
	//int loc_min = min_sn;
	//float min_dis = std::abs(features[feat_index][min_sn].y - point.y);
	for(int i = min_sn; i < (max_sn + 1); i++){
		if(closePoints(features[feat_index][i], point, g_margin)){
			ins = false;
			//std::cout<<i<<" is close \n";
			break;
		}
		/*if(std::abs(features[feat_index][i].y - point.y) < min_dis){
			loc_min = i;
		}*/
	}
	
	//min_sn = loc_min;
	
	if(ins){
		//std::cout<<"passed \n";
		if((features[feat_index].size() > (mid_sn + 1)) && ((mid_sn - 1) >= 0)){
			if((point.y > features[feat_index][mid_sn + 1].y)){
					features[feat_index].insert(features[feat_index].begin() + mid_sn + 2 , point);
				}else if((point.y < features[feat_index][mid_sn - 1].y)){
					features[feat_index].insert(features[feat_index].begin() + mid_sn - 1 , point);
				}else if((point.y < features[feat_index][mid_sn].y)){
					features[feat_index].insert(features[feat_index].begin() + mid_sn, point);	
				}else{
					features[feat_index].insert(features[feat_index].begin() + mid_sn + 1, point);
				}
			}else if(features[feat_index].size() > (mid_sn + 1)){//min_sn is not last obj 
				if((point.y > features[feat_index][mid_sn + 1].y)){
					if(view )std::cout<<"insert after: "<<features[feat_index][mid_sn + 1]<<"\n";
					features[feat_index].insert(features[feat_index].begin() + mid_sn + 2 , point);
				}else if((point.y < features[feat_index][mid_sn].y)){
					if(view )std::cout<<"insert b4 "<<features[feat_index][mid_sn + 1]<<"\n";
					features[feat_index].insert(features[feat_index].begin() + mid_sn , point);
				}else{
					if(view )std::cout<<"insert after: "<<features[feat_index][mid_sn + 1]<<"\n";
					features[feat_index].insert(features[feat_index].begin() + mid_sn + 1, point);	
				}
		}else{//(min == max)
			if((point.y < features[feat_index][mid_sn].y)){
				if(view )std::cout<<"insert b4: "<<features[feat_index][mid_sn]<<"\n";
				features[feat_index].insert(features[feat_index].begin() + mid_sn , point);
			}else{
				if(view )std::cout<<"insert after: "<<features[feat_index][mid_sn]<<"\n";
				features[feat_index].insert(features[feat_index].begin() + mid_sn + 1, point);	
			}
		}	
	}else if(view) std::cout<<"dont \n";
	
	if(view){
		for(int y = 0; y < features[feat_index].size(); y++){
			std::cout<<"features["<<feat_index<<"]["<<y<<"]: "<<features[feat_index][y]<<"\n";
		}
	}
	view = false;
}

void addToFeature(int feat_index, geometry_msgs::Point point){
	//std::cout<<"aTF \n";

	int f_size = features[feat_index].size();

	if(f_size == 1){//size = 1
		if(!closePoints(features[feat_index][0], point, g_margin)){
			if(point.y < features[feat_index][0].y) {
				features[feat_index].insert(features[feat_index].begin(), point);
			}
			else{
				features[feat_index].push_back(point);
			}
		}
	}else if(f_size == 2){//size = 2
		if((!closePoints(features[feat_index][0], point, g_margin)) &&
				(!closePoints(features[feat_index][1], point, g_margin))){
			if(point.y < features[feat_index][0].y ){
				features[feat_index].insert(features[feat_index].begin(), point);
			}else if ((point.y > features[feat_index][1].y)){
				features[feat_index].push_back(point);
			}else{
				features[feat_index].insert(features[feat_index].begin() + 1, point);
			}
		}
	}else{
		//std::cout<<"---- - sort Y \n";
		int mid_sn, big_sn, small_sn;
		big_sn = f_size - 1;
		small_sn = 0;
		bool y_sort = true;
		//bool x_sort = true;
		
		if((point.y < features[feat_index][small_sn].y)){
			y_sort = false;
			//x_sort = false;
			big_sn = small_sn;
			mid_sn = small_sn;//not needed for now
		}else if ((point.y > features[feat_index][big_sn].y)){
			y_sort = false;
			//x_sort = false;
			small_sn = big_sn;
			mid_sn = big_sn;//not needed for now
		}
		
		while(y_sort){//sort by y
			mid_sn = (big_sn + small_sn) / 2;
			if((big_sn - small_sn) <= 1){
				//x_sort = false;
				break;
			}
			
				
			if(point.y < features[feat_index][mid_sn].y){
				big_sn = mid_sn;
			}else if(point.y > features[feat_index][mid_sn].y){
				small_sn = mid_sn;
			}else{//mid_sn == point
				big_sn = mid_sn;
				small_sn = mid_sn;
				
				break;
			}
		}
		
		//incase more are in range (expand)
		bool bg_bool = false;
		big_sn = mid_sn;
		while(std::abs(point.y - features[feat_index][big_sn].y) < g_margin){
			big_sn ++;
			bg_bool = true;
			if(big_sn == (features[feat_index].size())) break;
		}
		if(bg_bool) big_sn --;
		
		bool sm_bool = false;
		small_sn = mid_sn;
		while(std::abs(point.y - features[feat_index][small_sn].y) < g_margin){
			small_sn --;
			sm_bool = true;
			if(small_sn < 0) break;
		}
		if(sm_bool) small_sn ++;
		
		/*std::cout<<"\n";
		std::cout<<"features["<<feat_index<<"].size(): "<<features[feat_index].size()<<"\n";
		std::cout<<"point: "<<point<<"\n";
		std::cout<<"f["<<feat_index<<"]["<<big_sn<<"]: "<<features[feat_index][big_sn]<<"\n";
		std::cout<<"f["<<feat_index<<"]["<<small_sn<<"]: "<<features[feat_index][small_sn]<<"\n";*/
		
		view = false;
		/*while(x_sort){//sort by x
			if((big_sn - small_sn) <= 1){
				break;
			}
			mid_sn = (big_sn + small_sn) / 2;
			
			if(point.x < features[feat_index][mid_sn].x){
				big_sn = mid_sn;
			}else if(point.x > features[feat_index][mid_sn].x){
				small_sn = mid_sn;
			}else if(point.x == features[feat_index][mid_sn].x){
				break;
			}	
		}*/
		//mid_sn is always small_sn
		//std::cout<<"mid_sn: "<<mid_sn<<"\n";
		insertToFeature(feat_index, point, small_sn, big_sn, mid_sn);
	}
	//std::cout<<"out aTF \n";
}

void mergeFeatures(int feat_1, int feat_2){
	//std::cout<<"merge \n";
	for(int k  = 0; k < features[feat_2].size(); k++){
		addToFeature(feat_1, features[feat_2][k]);
	}
	features.erase(features.begin() + feat_2);
	//std::cout<<"active_feature: "<<active_feature<<"\n";
	//std::cout<<"feat_2: "<<feat_2<<"\n";
	if(active_feature > feat_2) active_feature--;
	updateLists(feat_2, feat_1);
	//std::cout<<"merge end\n";
}

void updateLists(int index, int moved_to){
	for(int i = 0; i < checked_features.size(); i++){
		if(checked_features[i] == index){
			if(!featureChecked(moved_to)){
				checked_features[i] = moved_to;
			}else{
				checked_features.erase(checked_features.begin() + i);
				i--;
			}	
		}else if(checked_features[i] > index){
			checked_features[i] = checked_features[i] - 1;
		}	
	}
	for(int i = 0; i < to_be_explored.size(); i++){
		if(to_be_explored[i] == index){//the new feature will be detected in update
			to_be_explored.erase(to_be_explored.begin() + i); //might delay exploration***
			i--;
		}else if(to_be_explored[i] > index){
			to_be_explored[i] = to_be_explored[i] - 1;
		}	
	}
	generateEdges();	
}

void updateToBeExplored(int index){
	int min_feature_points = 10; //less than this is not fully formed
	if(features[index].size() > min_feature_points){ 
		bool push = true;

		for(int i = 0; i < to_be_explored.size(); i++){
			if(to_be_explored[i] == index){
				push = false;
				break;
			}
		}
		if(push) {
			to_be_explored.push_back(index);
			//orderToBeExplored();
		}		
	}
}

bool featureCleared(int feat_id){
	std::vector<geometry_msgs::Point> square;
	//will not scale (no. of sides)
	geometry_msgs::Point p00;
	p00.x = feature_edges[feat_id][0].x;
	p00.y = feature_edges[feat_id][3].y;
	square.push_back(p00);
		
	geometry_msgs::Point p11;
	p11.x = feature_edges[feat_id][0].x;
	p11.y = feature_edges[feat_id][1].y;
	square.push_back(p11);
	
	geometry_msgs::Point p22;
	p22.x = feature_edges[feat_id][2].x;
	p22.y = feature_edges[feat_id][1].y;
	square.push_back(p22);
	
	geometry_msgs::Point p33;
	p33.x = feature_edges[feat_id][2].x;
	p33.y = feature_edges[feat_id][3].y;
	square.push_back(p33);
	
	int i = 0;
	int j;
	while(i < square.size()){
		if(i == (square.size() - 1)) j = 0;
		else j = i + 1;
		if(isInside(feature_edges[feat_id][i], global_vertices)){//if edge is in area
			if(isInside(square[i], global_vertices) && isInside(square[j], global_vertices)){
				//everything is inside
				if(obstacleOnLineMap(square[i], square[j], 0)){
					return false;
				}
			}else{ //one point is outside
				geometry_msgs::Point cross;
				geometry_msgs::Point* cross_p = &cross;
				geometry_msgs::Point e1 = global_vertices[0]; //min is 3, should be OK
				geometry_msgs::Point e2 = global_vertices[global_vertices.size() - 1]; 
				
				//order of intersection matters
				bool int_bool = intersection(cross_p, e1, e2, square[i], square[j]);
				int vert = global_vertices.size() - 1;
				
				while((!int_bool) && (vert > 0)){
					e1 = global_vertices[vert];
					e2 = global_vertices[vert - 1];
					
					int_bool = intersection(cross_p, e1, e2, square[i], square[j]);
					vert --;				
				}
				if(isInside(square[i], global_vertices)){
					if(obstacleOnLineMap(square[i], cross, 0)){
						return false;
					}
				}else{
					if(obstacleOnLineMap(square[j], cross, 0)){
						return false;
					}
				}
			}
		}
		i++;
	}
	removeAroundFeatures(feat_id);
	std::cout<<"-- feature Cleared: "<<features[feat_id].size()<<"\n";
	return true;
}

bool obstacleOnMap(geometry_msgs::Point point, double inflation){
	ros::spinOnce();
	int threshold, side_index, row_start, end_index, max_index;
	
	geometry_msgs::Point lc_point = point; //left bottom corner of square
	geometry_msgs::Point tc_point = point; //left bottom corner of square
	lc_point. x -= inflation/2;
	lc_point. y -= inflation/2;
	tc_point. x += inflation/2;
	tc_point. y += inflation/2;
	
	threshold = 50;
	side_index = inflation / map_grid.info.resolution; // Number of indices on square side
	row_start = getIndexFromMap(lc_point);//origin of 
	end_index = getIndexFromMap(tc_point);//origin of
	max_index = map_grid.info.width * map_grid.info.height;
	
	if((row_start < 0) || (row_start > max_index) || (end_index < 0) || (end_index > max_index)){
		ROS_WARN("Point out of scope of recieved map");
		return true;
	}

	if(side_index <= 1){//if inflation is zero
		int indx = getIndexFromMap(point);
		if((map_grid.data[indx] > threshold) || (map_grid.data[indx] < 0)){
			return true;
		}else return false;
	}
	
	for(int j = 0; j < side_index; j++){ //height
		for(int i = row_start; i < (row_start+side_index); i++){ //width
			if((map_grid.data[i] > threshold) || (map_grid.data[i] < 0)){
				return true;
			}
		}
		row_start += map_grid.info.width; 
	}
	
	return false;
}

bool obstacleOnLineMap(geometry_msgs::Point p1, geometry_msgs::Point p2, double inflation){
	geometry_msgs::Point curr_obst;
	std::vector<geometry_msgs::Point> points = gl_->getPoints(p1, p2, g_margin/2);
	for(int i = 0; i < points.size(); i++){
		if(obstacleOnMap(points[i], inflation)){
			return true;
		}
	}
	
	return false;
}

int getIndexFromMap(geometry_msgs::Point point){
	geometry_msgs::Point grid_origin;
	
	float grid_res = map_grid.info.resolution;
	int grid_width = map_grid.info.width; //No. of points
	int grid_height = map_grid.info.height;
	grid_origin = map_grid.info.origin.position;
	
	int x_count = (point.x - grid_origin.x) / grid_res;
	int y_count = ((point.y - grid_origin.y) / grid_res);
	y_count = y_count * grid_width; 
	if(x_count < 0 || y_count < 0) {
		std::cout<<"Point is out of scope of recieved map \n";
	}else if((x_count + y_count) >= map_grid.data.size()) {
		std::cout<<"Point is out of scope of recieved map \n";
	}else {
		return (x_count + y_count);
	}
	return 0;
}

std::vector<int> possiblyInFeature(geometry_msgs::Point point, std::vector<std::vector<int> >* pos_vec, bool *sorted_p){
	double start_time =ros::Time::now().toSec();
	std::vector<int> pf;
	bool loc_sort = true;
	pos_vec->erase(pos_vec->begin(), pos_vec->end());
	//TODO keep a list of close features dont run every time
	
	for(int i = 0; i < features.size(); i++){
		 
		if(pf.size() > 1) break; //Mearging more than two at a time = mem problems 

		//after edges are created and big enough to warrant sort
		if((!first_run) && (features[i].size() > 5)){		

				bool irrelevant = false;
				if((feature_edges.size() > i)){//requires edges generated
					if((!withinBounds(point, feature_edges[i], g_margin * 1.2)) || (!featureClose(i))){
						irrelevant = true;
					}
				}
				
				if(!irrelevant){
					int count = 0; //TODO remove
					int max_sn = features[i].size() - 1;
					int min_sn = 0;
					int mid_sn = (max_sn + min_sn)/2;
					//Y sort
					while ((max_sn - min_sn) > 1){
						count++;

						mid_sn = (max_sn + min_sn) / 2;
						if(point.y > features[i][mid_sn].y){
							min_sn = mid_sn;
						}else if (point.y < features[i][mid_sn].y){
							max_sn = mid_sn;
						}else{
							//while runs atleast once
							max_sn = mid_sn;
							while(point.y == features[i][max_sn].y){
								max_sn ++;
								if(max_sn == (features[i].size() - 1)) break;
							}
							max_sn --;

							min_sn = mid_sn;
							while(point.y == features[i][min_sn].y){
								min_sn --;
								if(min_sn < 0) break;
							}
							min_sn ++;
							break;
						}
					}
					
					//incase more are in range (expand)
					bool bg_bool = false;
					max_sn = mid_sn;
					while(std::abs(point.y - features[i][max_sn].y) < g_margin){
						max_sn ++;
						bg_bool = true;
						if(max_sn >= (features[i].size())) break;
					}
					if(bg_bool) max_sn --;

					bool sm_bool = false;
					min_sn = mid_sn;
					while(std::abs(point.y - features[i][min_sn].y) < g_margin){
						min_sn --;
						sm_bool = true;
						if(min_sn < 0) break;
					}
					if(sm_bool) min_sn ++;
					//X sort
					/*while((features[i][max_sn].x != features[i][min_sn].x) 
						   && ((max_sn - min_sn) > 1)){
						count++;

						mid_sn = (max_sn + min_sn)/2;
						if(point.x > features[i][mid_sn].x){
							min_sn = mid_sn;
						}else if (point.x < features[i][mid_sn].x){
							max_sn = mid_sn;
						}else{ //if equal
							//min_sn = mid_sn;
							//max_sn = mid_sn;
							break;
						}
					}*/
					//std::cout<<"count: "<<count<<"\n";
					//std::cout<<"features["<<i<<"].size(): "<<features[i].size()<<"\n";
					//use g_magin*2 bcs used in (isinfeature) and insertfeature
					
					//possibly in if within margin*2
					bool ins = false;
					for(int sn = min_sn; sn < (max_sn + 1); sn++){
						if(closePoints(features[i][sn], point, g_margin)){
							ins = true;
							break;
						}
					}
					
					if(ins){
							pf.push_back(i);
							std::vector<int> min_max;
							min_max.push_back(min_sn);
							min_max.push_back(max_sn);
							min_max.push_back(mid_sn);
							pos_vec->push_back(min_max);
							//std::cout<<"mid_sn: "<<mid_sn<<"\n";
							//std::cout<<"max_sn: "<<max_sn<<"\n";
					}	
				}
		}else{
			loc_sort = false;
			for(int i1 = 0; i1 < features[i].size(); i1++){
				if(pf.size() > 1) break;
				if(closePoints(point, features[i][i1], g_margin)){
					//std::cout<<"Break from small feat  \n";
					//unsorted min_max wont be used
					pf.push_back(i);
					std::vector<int> min_max;
					min_max.push_back(i1);
					while((i1+1) < features[i].size()){
						i1++;
						if(std::abs(point.y - features[i][i1].y) > g_margin){
							i1--;
							break;
						}
					}
					min_max.push_back(i1);
					min_max.push_back(min_max[0]);
					pos_vec->push_back(min_max);
					
					//std::cout<<"i1: "<<i1<<"\n";
					//std::cout<<"i1+1: "<<i1+1<<"\n";
					break;
				}
			}
		}
	}

	if(loc_sort){
		*sorted_p = true;
	} else *sorted_p = false;
	return pf;
}

bool featureClose(int feat_id){
	//only use after edges are created
	for(int i = 0; i < feature_edges[feat_id].size(); i++){
		if(gl_->getLength(robot_pose.position, feature_edges[feat_id][i]) < (sensor_range*1.2)){
			return true;
		}
	}
	
	//if big feat, aprx fails
	geometry_msgs::Point mp;
	for(int i = 0; i < feature_edges[feat_id].size(); i++){
		if((i + 1) < feature_edges[feat_id].size()){
			mp = midPoint(feature_edges[feat_id][i], feature_edges[feat_id][i + 1]);
		}else{
			mp = midPoint(feature_edges[feat_id][i], feature_edges[feat_id][0]);
		}
		
		if(gl_->getLength(robot_pose.position, mp) < (sensor_range * 1.2)){
			return true;
		}
	}
		
	return false;
}

int isInFeature(geometry_msgs::Point point){
	int closest_feature = -1;
	bool dum_bool;
	bool *dum_bool_p = &dum_bool;
	std::vector<std::vector<int> > dummy;
	std::vector<std::vector<int> >* dummy_p = &dummy;
	std::vector<int> ft_id = possiblyInFeature(point, dummy_p, dum_bool_p);
	if(ft_id.size() > 1){
		if(featureChecked(ft_id[0])) return ft_id[0];
		else return ft_id[1];
	}else if(ft_id.size() == 1){
		return ft_id[0];
	}
	
	return closest_feature;
}

bool obstacleOnLine(geometry_msgs::Point p1, geometry_msgs::Point p2){
	ros::spinOnce();
	updateCostmap(o_grid);

	std::vector<geometry_msgs::Point> points = gl_->getPoints(p1, p2, g_margin/2);
	for(int i = 0; i < points.size(); i++){
		if(obstacleOnPoint(points[i])){
			int ftr = isInFeature(obstacle);
			if(ftr != -1) {
				return true;
			}
		}
	}
	return false;
}

bool obstacleOnPoint(geometry_msgs::Point p1){
	for(int i = 0; i < obstacles.size(); i++){
		if(gl_->equalPoints(obstacles[i], p1, (g_margin*1.1))){
			obstacle = p1;
			return true;
		}else {} //do Nothing
	}
	return false;
}

bool obstacleOnPoint(geometry_msgs::Point p1, double margin){
	for(int i = 0; i < obstacles.size(); i++){
		if(gl_->equalPoints(obstacles[i], p1, margin)){
			obstacle = p1;
			return true;
		}
	}
	return false;
}

bool goToAction(geometry_msgs::Pose goal, int repeats){
	int count = 0;
	//stuck_time = ros::Time::now().toSec();
	//geometry_msgs::Point last_pos  = robot_pose.position;
	//geometry_msgs::Point *last_pos_p  = &last_pos;
	
	while(count < repeats){
		if(goToAction(goal)) return true;
		//if(stuck(last_pos_p, 7)) unStick(0.5, 1);
		count ++;
	}
	return false;
}


bool goToAction(geometry_msgs::Pose goal){
	std::cout<<"goal gtA: "<<goal.position<<"\n";
	std::cout<<"Z goal: "<<getZFromQuat(goal.orientation)<<"\n";
	std::cout<<"orien_tol: "<<orien_tol<<"\n";
	std::cout<<"xy_tol: "<<xy_tol<<"\n";
 
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);
	action_client.waitForServer();

	move_base_msgs::MoveBaseGoal mbA;
	mbA.target_pose.header.frame_id = map_frame;
	mbA.target_pose.header.stamp = ros::Time::now();
	mbA.target_pose.pose = goal;

	action_client.sendGoal(mbA);
	
	getOdom();
	ros::Duration wait(0.1);
	//stuck_time = ros::Time::now().toSec();
	//geometry_msgs::Point last_pos  = robot_pose.position;
	//geometry_msgs::Point *last_pos_p  = &last_pos;
	float g_start = ros::Time::now().toSec();
	double reset = 5;
	while(!equalPose(goal, mb_robot_pose, xy_tol, orien_tol) && 
		 !equalPose(goal, robot_pose, xy_tol, orien_tol)){	
		updateCostmap(o_grid); //has a rosspin
		closenessCheck(plan_tol);
		//if(stuck(last_pos_p, 5)) unStick(1, 1);
		if(action_client.waitForResult(wait)){
			if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				std::cout<<"\n success \n";
				return true;
			}else if(action_client.getState() == actionlib::SimpleClientGoalState::REJECTED){
				std::cout<<"failed goal ac \n"; return false;
			}
		}
		if((ros::Time::now().toSec() - g_start) > reset){
			setBlindGoal(goal, 0);
			g_start = ros::Time::now().toSec();
		}			
	}
	std::cout<<"\n goTo reached \n";
	return true;
}

/*bool goTo(geometry_msgs::Pose goal){

	std::cout<<"goTo \n";
	ros::Rate snooze(0.5);
	if(!setBlindGoal(goal, 0)) ;
	snooze.sleep();
	int mb_status;
	float t_start = ros::Time::now().toSec();
	
	getOdom();
	geometry_msgs::Point last_pos  = robot_pose.position;
	geometry_msgs::Point *last_pos_p  = &last_pos;
	while(!equalPose(goal, robot_pose, xy_tol, orien_tol)){	
		closenessCheck(plan_tol);
		updateCostmap(o_grid);
		//if(stuck(last_pos_p, 5)) unStick(1, 1);
		if(obstacleOnPoint(goal.position)){//close should fix this
			goal = findFree(goal);
			setBlindGoal(goal, 0);
			//snooze.sleep();
			if(obstacleOnPoint(goal.position)){std::cout<<"goTo false \n"; return false;}
			
		}

		if(goal_status_->status_list.size() > 0){
			mb_status = goal_status_->status_list[goal_status_->status_list.size() - 1].status;
		}
		
		if(mb_status == 4 || mb_status == 5 || mb_status == 9){
			goal = findFree(goal);
			setBlindGoal(goal, 0);
			std::cout<<"goTo false \n";
			return false;
		}else if((mb_status == 3) && ((ros::Time::now().toSec() - t_start) > 5)){//Hot Fix
			break; 
		}
		//snooze.sleep();
	}
	std::cout<<"goTo reached \n";
	return true;
}*/

geometry_msgs::Pose translateLinear(geometry_msgs::Pose pose, double del_lin){
	double z = getZFromQuat(pose.orientation);	
	pose.position.x += del_lin*cos(z);
	pose.position.y += del_lin*sin(z);

	return pose;
}

/*void unStick(double clearance, double duration){

	geometry_msgs::Twist vel;
	vel.linear.x = safe_speed;
	vel.angular.z = 0;
	
	double min_angle = -pi / 12; //-15 degrees
	double max_angle = pi / 12; //15 degrees
	int start_id = ((min_angle - scans->angle_min) / scans->angle_increment); 
	int end_id = ((max_angle - scans->angle_min) / scans->angle_increment); 
	bool front_clear = true;
	int count = 0;
	double start_time = ros::Time::now().toSec();
	
	while((ros::Time::now().toSec() - start_time) < duration){
		ros::spinOnce();
		if(front_clear){
			for(int i = start_id; i != end_id; i++){ 
				double angle = min_angle + (count * scans->angle_increment);
				if(scans->ranges[i] < (clearance * sin(angle))){
					vel.linear.x = 0;
					cmd_publisher.publish(vel);
					std::cout<<"front blocked at: "<<angle<<"\n";
					front_clear = false;
					break;
				}
				count ++;
			}
		}

		if(front_clear){
			cmd_publisher.publish(vel);
		}else{

			geometry_msgs::Point behind;
			double bz = getZFromQuat(robot_pose.orientation);
			behind.x = robot_pose.position.x + (clearance * cos(pi + bz));
			behind.y = robot_pose.position.y + (clearance * sin(pi + bz));
			
			if(!obstacleOnMap(behind, robot_span/2)){ 
				vel.linear.x = -1 * safe_speed;
				cmd_publisher.publish(vel);
			}else {std::cout<<"cant step back either \n"; break;}
		}		
	}
	std::cout<<"out of unstick \n";

}*/

geometry_msgs::Pose escapeWall(std::string direction, double del_lin){
	//Laser range greater than del_lin is considered traversable space
	if(direction != "left"  && direction != "right"){
		ROS_WARN("Direction in wall follow not set.Use 'left' or 'right'");
		return robot_pose;
	}
	ros::spinOnce();
	geometry_msgs::Pose goal;
	std::vector<double> gaps;
	double gap;
	bool initial = true;
	//cosine rule is an aprx based on del_lin, (as del_lin changes ....)
	//double scan_segment = acos(((2*del_lin*del_lin) - (1.1 * robot_width * robot_width )) / (2*del_lin*del_lin)); 
	 
	int min_count = ( scan_segment / scans->angle_increment);
	std::cout<<"\n scan_segment: "<<scan_segment<<"\n";
	std::cout<<"robot_width: "<<robot_width<<"\n";
	std::cout<<"del_lin: "<<del_lin<<"\n";
	
	//set range of used scans
	double start_scan = -5*pi/12;
	double end_scan = 5*pi/12; // -15 deg for edges and small obs 
	
	int ini = (start_scan - scans->angle_min)/ scans->angle_increment;
	int max_index = ((end_scan - scans->angle_min) / scans->angle_increment);
	int num_scans = (scans->angle_max - scans->angle_min)/ scans->angle_increment;
	//std::cout<<"scans->angle_min: "<<scans->angle_min<<"\n";
	//std::cout<<"scans->angle_increment: "<<scans->angle_increment<<"\n";
	//std::cout<<"ini: "<<ini<<"\n";
	std::cout<<"min_count: "<<min_count<<"\n";
	
	if((ini < 0) || ((max_index) > num_scans )){		
		ROS_WARN("Requested scan out of range of laser Data, atleast 180 degrees required");
		ROS_WARN("min scan recieved %e", scans->angle_min);
		ROS_WARN("max scan recieved %e", scans->angle_max);
		return robot_pose;
	}
		
	//del_lin*0.6 used for gaps, goals set at del_lin if clear, if not at 0.5*del_lin
	int count = 0;
	for(int i = ini; i < max_index; i++){ 
		//create gaps in del/2, priotize full later
		if(scans->ranges[i] >= del_lin){
			if(initial){		
				gap = 0; //init
				count = 0;
				initial = false;
			}
			
			gap += scans->angle_min + (i * scans->angle_increment);
			if(count >= min_count) {
				gaps.push_back(gap/count); 
				initial = true;
			}
			count ++;
		}else{ 
			initial = true;
		}
	}
	/*for(int t = 0; t < gaps.size(); t++){
		std::cout<<"gaps["<<t<<"]: "<<gaps[t]<<"\n";
	}*/

	double wall_safe = inflation_angle;
	//std::cout<<"/n gaps.size(): "<<gaps.size()<<"\n";
	if(gaps.size() == 0){//rotate
		if(direction == "left"){
			goal = gl_->rotatePose(robot_pose, -pi/2);
		}else{
			goal = gl_->rotatePose(robot_pose, pi/2);
		}
		
		std::cout<<"gap == 0 goal: "<<goal.position<<" \n";
		return goal;
	}else{
		//where gap is the angle from -pi to pi robot frame
		bool increase;
		if((direction == "left") && (laser_clockwise)) increase = true;
		else if((direction == "right") && (laser_clockwise)) {
			wall_safe = -1*wall_safe; // TODO based on robot width
			increase = false;
		}
		else if((direction == "left") && (!laser_clockwise)){
			wall_safe = -1*wall_safe; // TODO based on robot width
			increase = false;
		}
		else if((direction == "right") && (!laser_clockwise)) increase = true;
		
		int sn;
		if(increase){
			std::cout<<"left on husky \n";
			sn = 0;
		}else{
			std::cout<<"right on husky \n";	
			sn = gaps.size() - 1;
		}
		count = 0;
		std::cout<<"gap.size(): "<<gaps.size()<<" \n";
		std::cout<<"laser_align: "<<laser_align<<" \n";
		while(count < gaps.size()){
			double robot_z = getZFromQuat(robot_pose.orientation);
			goal = gl_->rotatePose(robot_pose, laser_align*(gaps[sn] + wall_safe));//-ve bcs (+ve is ACW and vv)
			goal = translateLinear(goal, del_lin);
			std::cout<<"gaps["<<sn<<"]: "<<gaps[sn]<<" \n";
			std::cout<<"mb_z: "<<robot_z<<" \n";
			std::cout<<"rob pos: "<<robot_pose.position<<"\n \n";
			
			std::cout<<"goal_z: "<<getZFromQuat(goal.orientation)<<" \n";
			std::cout<<"goal: "<<goal.position<<" \n";
			
			if(!obstacleOnPoint(goal.position)){
				std::cout<<"accepted full \n";
				//std::cout<<"ret goal: "<<goal.position<<"\n";
				return goal;
			}else if ((count >= gaps.size()/2) && (del_lin/2 > xy_tol)){
				goal = translateLinear(goal, -0.4 * del_lin);
				std::cout<<" 1/2: "<<goal.position<<" \n";
				if(!obstacleOnPoint(goal.position)){
					
					//std::cout<<"ret half goal: "<<goal.position<<" \n";
					return goal;
				}//else std::cout<<"blckd @ gaps["<<i<<"]: "<<gaps[i]<<"\n";
			}
			if(increase) sn++;
			else sn--;
			count ++;
		}
	}
	
	//paranoid :)
	goal = gl_->rotatePose(robot_pose, pi);
	return goal;
	
}

geometry_msgs::Point midPoint(geometry_msgs::Point p1, geometry_msgs::Point p2){
	geometry_msgs::Point p_out;
	p_out.x = (p1.x + p2.x)/2;
	p_out.y = (p1.y + p2.y)/2;
	
	return p_out;
}

geometry_msgs::Pose midPose(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
	geometry_msgs::Pose p_out;
	p_out.position.x = (p1.position.x + p2.position.x)/2;
	p_out.position.y = (p1.position.y + p2.position.y)/2;
	p_out.orientation = gl_->getQuatFromP(p1.position, p2.position);
	
	return p_out;
}

bool equalPose(geometry_msgs::Pose p1, geometry_msgs::Pose p2, double xy_tol, double orien_tol){
	double p1z, p2z;
	p1z = getZFromQuat(p1.orientation);
	p2z = getZFromQuat(p2.orientation);

	if ((gl_->getLength(p1.position, p2.position) < xy_tol) && (std::abs(p1z - p2z) < orien_tol)){
		return true;
	}else return false;
}

bool intersection(geometry_msgs::Point* point_out, geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point a1, geometry_msgs::Point a2){
	bool intersection_bool;
	geometry_msgs::Point intersection;
	double margin = 0.01; //horizontals and verticals
	
	std::vector<geometry_msgs::Point> series;
	series.push_back(a1);
	series.push_back(a2);
	
	float x_min, y_min, x_max, y_max;
	
	float mp = (p2.y - p1.y)/(p2.x - p1.x); 
	float ma = (a2.y - a1.y)/(a2.x - a1.x); 
	float cp = p2.y - (mp * p2.x);
	float ca = a2.y - (ma * a2.x);
	if(((p2.x - p1.x) != 0) && ((a2.x - a1.x) != 0)){//most of the time
		intersection.x = (ca - cp)/(mp - ma);
	}else if(((p2.x - p1.x) == 0) && ((a2.x - a1.x) != 0) ){//division by zero
		intersection.x = p1.x;
	}else if(((a2.x - a1.x) == 0) && ((p2.x - p1.x) != 0) ){//division by zero
		intersection.x = a1.x;
	}else{//both vertical should never happen in this scope
		intersection.x = std::numeric_limits<float>::infinity();
	}
	intersection.y = mp*intersection.x + cp;
	
	x_max = series[0].x;
	x_min = series[0].x;
	y_max = series[0].y;
	y_min = series[0].y;
	
	for(int i = 0; i < series.size(); i++){
		if(series[i].x < x_min) x_min = series[i].x; 
		if(series[i].x > x_max) x_max = series[i].x; 
		if(series[i].y > y_max) y_max = series[i].y; 
		if(series[i].y < y_min) y_min = series[i].y; 
	}
	
	x_min -= margin; //expand range for horiz and vertical lines
	x_max += margin;
	y_min -= margin;
	y_max += margin;
	
	if((intersection.x <= x_max) && (intersection.x >= x_min) 
	   && (intersection.y >= y_min) && (intersection.y <= y_max)){
		*point_out = intersection;
		
		return true;
	}else{
		//std::cout<<"false \n";
		return false;
	}
}