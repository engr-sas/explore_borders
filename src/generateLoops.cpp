/* Author: Shamsudeen A Sodangi */
#include "generateLoops.h"

GenerateLoops::GenerateLoops(double sensor_range, std::string map_frame){
	this->map_frame = map_frame;
	this->sensor_range = sensor_range;
	pi = 3.14159;
	del_points = 0.5; //used in closest point check
	global_plan.header.frame_id = map_frame;
	sensor_range_ = &sensor_range;
	global_plan_ = &global_plan;
}

void GenerateLoops::getLoops(std::vector<geometry_msgs::Point> p){	
	all_walls = allInnerEdges(p, sensor_range);
	all_sections = allInnerEdges(p, sensor_range/2);
	std::vector<geometry_msgs::Point> curr_loop;
	exp_boundary = createPoly(p);
	p_boundary = p;

	for(int i = 0; i < all_walls.size(); i++){ //create inner loop using wall[], skip inner
		if(minSide(all_walls[i]) > sensor_range*2){
			curr_loop = buildLoop(all_walls[i], sensor_range);
			num_vertices.push_back(all_walls[i].size() * 3); //3 points per vertice

			for(int ii = 0; ii < curr_loop.size(); ii++){//assign points to global variable
				global_loops.push_back(curr_loop[ii]);
			}	
		}
	}
}

std::vector<geometry_msgs::Point> GenerateLoops::buildLoop
(std::vector<geometry_msgs::Point> wallEdges, float laser_range){

	std::vector<geometry_msgs::Point> edges; 
	std::vector<float> angles;
	std::pair<std::vector<geometry_msgs::Point>, float> temp;
	
	for(int i = 0; i < wallEdges.size(); i++ ){
		std::vector<geometry_msgs::Point> tempVec;
		float tempAngle;
		
		if(i == (wallEdges.size() - 2) ){
			temp = innerEdge(wallEdges[i] ,wallEdges[i+1], wallEdges[0], laser_range);
			tempVec = temp.first;
			tempAngle = temp.second;
		}
		else{
			if(i == (wallEdges.size() - 1) ){
				temp = innerEdge(wallEdges[i] ,wallEdges[0], wallEdges[1], laser_range);
				tempVec = temp.first;
				tempAngle = temp.second;
			}
			else{
				temp = innerEdge(wallEdges[i] ,wallEdges[i+1], wallEdges[i+2], laser_range);
				tempVec = temp.first;
				tempAngle = temp.second;
			}
		}
		angles.push_back(tempAngle);
		edges.push_back(tempVec[0]); 
		edges.push_back(tempVec[1]); 
		edges.push_back(tempVec[2]);
	}
	
	return edges;
}

std::pair<std::vector<geometry_msgs::Point>, float>  GenerateLoops::innerEdge(
	geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, float range){
	
	std::pair<std::vector<geometry_msgs::Point>, float>  pa; //points and angle
	std::vector<geometry_msgs::Point> edge;
	float opt_dist;
	geometry_msgs::Point p11;
	geometry_msgs::Point p22;
	geometry_msgs::Point p33;
	
	//mange opt_dist
	double min_side;
	double s1 = getLength(p1, p2);
	double s2 = getLength(p3, p2);
	if((s1 < range*4) || (s2 < range*4)){
		if(s1 < s2){
			min_side = s1;
		}else min_side = s2;
		opt_dist = (min_side * 0.45);
	}else{
		opt_dist = range * 2;
	}
	if(opt_dist < range) opt_dist = range;
	float min_ang = atan(range / opt_dist);
		
	float m1 = grad(p1.x, p1.y, p2.x, p2.y); 
	float m2 = grad(p2.x, p2.y, p3.x, p3.y);
	
	float ang_l12 = atan(m1);	
	float ang_l23 = atan(m2);

	ang_l12 = treatAngle(ang_l12, quad(p2, p1));
	ang_l23 = treatAngle(ang_l23, quad(p2, p3));
	float ang_l12_ = posAngle(ang_l12);
	float ang_l23_ = posAngle(ang_l23);
	float ang_lm = angleBetween(ang_l12, ang_l23);
	
	p22.x = p2.x + range * cos(ang_lm); //vertice of path
	p22.y = p2.y + range * sin(ang_lm);	
	p22.z = p2.z;
	
	float a3x = p2.x + opt_dist * cos(ang_l23);
	float a3y = p2.y + opt_dist * sin(ang_l23);
	float a1x = p2.x + opt_dist * cos(ang_l12);
	float a1y = p2.y + opt_dist * sin(ang_l12);

	float pm1 = (-1 / m1);
	float p11x1 = a1x + sqrt(pow(range, 2) / (1 + pow(pm1, 2)));
	float p11y1 =  pm1 * (p11x1 - a1x) + a1y;
	float p11x2 = a1x - sqrt(pow(range, 2) / (1 + pow(pm1, 2)));
	float p11y2 = pm1 * (p11x2 - a1x) + a1y;
	
	geometry_msgs::Point p111 = createPoint(p11x1, p11y1, p22.z);
	geometry_msgs::Point p112 = createPoint(p11x2, p11y2, p22.z);
	
	if(getLength(p111, p3) < getLength(p112, p3)){
		p11 = p111;
	}
	else {p11 = p112;}
	
	float pm2 = (-1 / m2);
	float p33x1 = a3x + sqrt(pow(range, 2) / (1 + pow(pm2, 2)));
	float p33y1 = pm2 * (p33x1 - a3x) + a3y;
	float p33x2 = a3x - sqrt(pow(range, 2) / (1 + pow(pm2, 2)));
	float p33y2 = pm2 * (p33x2 - a3x) + a3y;
	
	geometry_msgs::Point p331 = createPoint(p33x1, p33y1, p22.z);
	geometry_msgs::Point p332 = createPoint(p33x2, p33y2, p22.z);
	
	if(getLength(p331, p1) < getLength(p332, p1)){
		p33 = p331;
	}
	else {p33 = p332;}
	
	if(std::abs(ang_l12 - ang_lm) < min_ang){ //min angle
		p33.x = p2.x + sqrt(2) * range * cos(ang_lm);
		p33.y = p2.y + sqrt(2) * range * sin(ang_lm);	
		p11.x = p33.x;
		p11.y = p33.y;
	}
	
	if(getLength(p1, p2) < opt_dist * 2){//min length
		p11.x = p2.x + sqrt(2) * range * cos(ang_lm);
		p11.y = p2.y + sqrt(2) * range * sin(ang_lm);
	}
	
	if(getLength(p1, p3) < opt_dist * 2){//min length
		p33.x = p2.x + sqrt(2) * range * cos(ang_lm);
		p33.y = p2.y + sqrt(2) * range * sin(ang_lm);	
	}
	
	edge.push_back(p11);
	edge.push_back(p22);
	edge.push_back(p33);
	
	pa.first = edge;
	pa.second = std::abs(ang_l12_ - ang_l23_);
	if(pa.second > pi){
		pa.second = pi * 2 - pa.second;
	}
	return pa;
}

std::vector<std::vector<geometry_msgs::Point> > GenerateLoops::allInnerEdges
(std::vector<geometry_msgs::Point> vertices, float range){
	std::vector<std::vector<geometry_msgs::Point> > walls;
	std::vector<geometry_msgs::Point> curr_wall;
	curr_wall = vertices;
	walls.push_back(curr_wall);

	while(createInnerEdges(curr_wall, range, &curr_wall)){
		walls.push_back(curr_wall);
	}	
	return walls;
}

bool GenerateLoops::createInnerEdges
(std::vector<geometry_msgs::Point> vertices, float range, std::vector<geometry_msgs::Point> *vert_out){

	std::vector<geometry_msgs::Point> wall;
	std::vector<float> angles;
	
	for(int i = 0; i < vertices.size(); i++ ){
		geometry_msgs::Point tempPoint;
		float tempAngle;
		std::pair<geometry_msgs::Point, float> temp;
		
		if(i == (vertices.size() - 2) ){ //iterate through points using 3 vertices
			temp = innerEdgeWall(vertices[i] ,vertices[i+1], vertices[0], range);
			tempPoint = temp.first;
			tempAngle = temp.second;
		}else{
			if(i == (vertices.size() - 1) ){
				temp = innerEdgeWall(vertices[i] ,vertices[0], vertices[1], range);
				tempPoint = temp.first;
				tempAngle = temp.second;
			}
			else{
				temp = innerEdgeWall(vertices[i] ,vertices[i+1], vertices[i+2], range);
				tempPoint = temp.first;
				tempAngle = temp.second;
			}
		}
		angles.push_back(tempAngle);
		wall.push_back(tempPoint);
	}

	if(!offsetTrue(vertices, wall)){
		*vert_out = vertices;
		return false;	
	}
	
	*vert_out = wall;
	return true;
}

bool GenerateLoops::offsetTrue(std::vector<geometry_msgs::Point> outer, std::vector<geometry_msgs::Point> inner){
	//if offset properly, shortest length should be to same vertice
	//outer 1, is inner 0
	if(outer.size() != inner.size()){
		return false;
	}
	for(int i = 0; i < inner.size(); i++){
		int out_index;
		if(i < (outer.size() - 1)) out_index = i + 1;
		else out_index = 0;

		double length = getLength(inner[i], outer[out_index]);

		for(int j = 1; j < outer.size(); j++){
			
			if((out_index + 1) < outer.size()) out_index++;
			else out_index = 0;
			if(getLength(inner[i], outer[out_index]) < length){
				return false;
			}
		}
		
	}
	return true;
}

std::pair<geometry_msgs::Point, float> GenerateLoops::innerEdgeWall
(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, float range){
	
	std::pair<geometry_msgs::Point, float> pa;
	geometry_msgs::Point p22;
	
	float m1 = grad(p1.x, p1.y, p2.x, p2.y); 
	float m2 = grad(p2.x, p2.y, p3.x, p3.y);
	float ang_l12 = atan(m1);	
	float ang_l23 = atan(m2);
	
	ang_l12 = treatAngle(ang_l12, quad(p2, p1));
	ang_l23 = treatAngle(ang_l23, quad(p2, p3));
	float ang_lm = angleBetween(ang_l12, ang_l23);
	float mm = tan(ang_lm); //grad
	
	float ang_lm_ = posAngle(ang_lm);//angle to new point
	float ang_l12_ = posAngle(ang_l12);
	float ang_l23_ = posAngle(ang_l23);

	float hyp = sqrt(2)*2*range;
	
	if(ang_lm > 0){
		if(ang_lm < pi/2){ //first quad
			p22.x = p2.x + sqrt(pow(hyp, 2) / (1 + pow(mm, 2))); //pythagoras + grad
		}
		else{
			p22.x = p2.x - sqrt(pow(hyp, 2) / (1 + pow(mm, 2)));
		}	
	}else{
		if(ang_lm < -pi/2){
			p22.x = p2.x - sqrt(pow(hyp, 2) / (1 + pow(mm, 2)));
		}
		else{
			p22.x = p2.x + sqrt(pow(hyp, 2) / (1 + pow(mm, 2)));
		}
	}
	p22.y = mm * (p22.x - p2.x) + p2.y;
	p22.z = p2.z;
	
	pa.first = p22;
	pa.second = std::abs(ang_l12_ - ang_l23_);
	if(pa.second > pi){
		pa.second = pi * 2 - pa.second;
	}
	return pa;
}

float GenerateLoops::minSide(std::vector<geometry_msgs::Point> vertices){
	
	float min;
	if (vertices.size() > 2){
		min = getLength(vertices[vertices.size() - 1], vertices[0]);
		for(int i = 0; i < vertices.size() - 1; i++){
			if(getLength(vertices[i], vertices[i + 1]) < min){
				min = getLength(vertices[i], vertices[i + 1]);
			}
		}
	}
	else{
		//std::cout<<"less than 3 vertices, null returned"<<"\n";
	}
	return min;
}

double GenerateLoops::getMaxDiagonal(std::vector<geometry_msgs::Point> vertices){
	double diag;
	if (vertices.size() > 2){
		diag = getLength(vertices[vertices.size() - 1], vertices[0]);
		for(int j = 0; j < vertices.size(); j++){
			for(int i = 0; i < vertices.size(); i++){
				if(i != j){
					if(getLength(vertices[j], vertices[i]) > diag){
						diag = getLength(vertices[i], vertices[j]);
					}
				}
			}
			vertices.erase(vertices.begin());
		}
	}
	else{
		//std::cout<<"less than 3 vertices, null returned"<<"\n";
	}
	return diag;
}

std::vector<int> GenerateLoops::findReflex(std::vector<float> angles){

	std::vector<int> index;
	float margin = pi / 100; 
	if (angles.size() > 2){
		float sum_expected = (angles.size() - 2) * pi;
		float actual_sum = 0;
		for(int i = 0; i < angles.size(); i++){
			actual_sum += angles[i];
		}
		
		if(actual_sum < (sum_expected + margin) && actual_sum > (sum_expected - margin)){
			//do nothing return index
		}
		else{
			//TODO find num of reflex angles first
			for(int i1 = 0; i1 < angles.size(); i1++ ){ //for one reflex
				float refTotal = actual_sum - (2 * angles[i1]) + (2 * pi);
				
				if(refTotal < (sum_expected + margin) && refTotal > (sum_expected - margin)){
					index.push_back(i1 + 1);
				}
			}
		}
	}
	else{
		//std::cout<<"null returned, cant find reflex"<<"\n";
	}
	return index;
}

std::vector<geometry_msgs::Point> GenerateLoops::createInnerPointsAlt
(std::vector<geometry_msgs::Point> bounds, double off_dist){
	//create last inner loop for certain bounds

	std::vector<geometry_msgs::Point> points;
	std::vector<geometry_msgs::Point> points_r;
	geometry_msgs::Point point;
	geometry_msgs::Point center = getCenter(bounds).position; 
	geometry_msgs::Point start; 
	if(cur_goal_poses.size() > 0){
		start = cur_goal_poses[cur_goal_poses.size() - 1].position;
	}else start = curr_pose.position;
	
	for(int i = 0; i < bounds.size(); i++){
		geometry_msgs::Point mp;
		if(i < (bounds.size()-1)){
			mp = midPoint(bounds[i], bounds[i+1]);
		}else{
			mp = midPoint(bounds[bounds.size()-1], bounds[0]);
		}

		bool include_mp  = true;

		//possible plan point
		geometry_msgs::Point p1 = pointOnLine(mp, center, center, off_dist, false);

		if(getLength(mp, center) > off_dist){//if mp (on plan already) has not covered point
			for(int k = 0; k < points.size(); k++){//current considerations
				if(getLength(mp, points[k]) < off_dist){
					include_mp = false;
					break;
				}
			}
			
			if(include_mp){
				for(int l = 0; l < cur_goal_poses.size(); l++){//works if from outside (in not created yet)
					if(getLength(p1, cur_goal_poses[l].position) < sensor_range){
						include_mp = false;
						break;
					}
				}
				points.push_back(p1);
			}	
		}
		
		int i1, i2, i3;
		if(i == (bounds.size() - 1)){ //last
			i1 = i;
			i2 = 0; 
			i3 = 1;
		}else if(i == (bounds.size() - 2)){
			i1 = i;
			i2 = i + 1; 
			i3 = 0;
		}else{
			i1 = i;
			i2 = i + 1; 
			i3 = i + 2;
		}
		
		bool include_e  = true;
		if(getLength(bounds[i2], center) > sensor_range){
			for(int j = 0; j < points.size(); j++){
				//if target zone is already covered
				if((getLength(bounds[i2], points[j]) < sensor_range)){
					include_e = false;  
					break;
				}
			}
			if (include_e) {
				geometry_msgs::Point tp = innerEdge(bounds[i1], bounds[i2], bounds[i3], sensor_range).first[1];
	
				//if zone is covered by plan
				for(int l = 0; l < cur_goal_poses.size(); l++){
					if(getLength(tp, cur_goal_poses[l].position) < sensor_range){
						include_e = false;
						break;
					}
				}
				if (include_e)points.push_back(tp);
			}
		}		
	}
	
	if(points.size() == 0) points.push_back(center); 
	points.insert(points.begin(), start);
	
	//for(int r= 0; r < points.size(); r++) std::cout<<"points["<<r<<"]: "<<points[r]<<"\n";
	
	points_r = bestPath(points);
	
	//for(int r= 0; r < points_r.size(); r++) std::cout<<"points_r["<<r<<"]: "<<points_r[r]<<"\n";
	return points_r;	
}

std::vector<geometry_msgs::Point> GenerateLoops::bestPath(std::vector<geometry_msgs::Point> plan){
	//zero is start cant change, everything else can ish
	double min_len = std::numeric_limits<double>::infinity();//babban lamba
	int min_start = 0;
	bool reversed  = false;

	if(plan.size() < 3) return plan;
	for(int i = 1; i < plan.size(); i++){//limited to walking seq, just different starts
		double len = getLength(plan[0], plan[i]); //zero not included in loop
		int id  = i;

		//sum len going from 0 - id - everywher sequentially 
		for(int j = 1; j < plan.size() - 1; j ++){ //just a count
			if(id < (plan.size() - 1)){
				len += getLength(plan[id], plan[id + 1]);
				id++;
			}else{//if id = (plan.size() - 1)
				len += getLength(plan[plan.size() - 1], plan[1]);
				id = 1; //start at 1 zero not included
			}
		}
		
		if(len < min_len){
			min_len = len;
			min_start = i;		
		}	
	}
	//reverse direction
	for(int i = plan.size()-1; i > 0 ; i--){
		double len = getLength(plan[0], plan[i]); //zero not included in loop
		int id  = i;

		for(int j = 1; j < plan.size() - 1; j ++){ //just a count
			if(id > 1){
				len += getLength(plan[id], plan[id + 1]);
				id--;
			}else{//id = 1
				len += getLength(plan[plan.size() - 1], plan[1]);
				id = (plan.size() - 1); //start at max, zero not included
			}
		}
		
		if(len < min_len){
			min_len = len;
			min_start = i;	
			reversed = true;
		}	
	}
	
	std::vector<geometry_msgs::Point> b_plan;
	b_plan.resize(plan.size());
	b_plan[0] = plan[0];
	
	
	if(!reversed){
		int pr = 1;
		while(pr < plan.size()){
			b_plan[pr] = plan[min_start];

			if(min_start < (plan.size() - 1)) min_start ++;
			else min_start = 1;

			pr++;
		}
	}else{
		int pr = plan.size() - 1;
		while(pr > 0){
			b_plan[pr] = plan[min_start];

			if(min_start > 1) min_start --;
			else min_start = plan.size() - 1;

			pr--;
		}
	}

	return b_plan;
}

geometry_msgs::Point GenerateLoops::midPoint(geometry_msgs::Point p1, geometry_msgs::Point p2){
	geometry_msgs::Point p_out;
	p_out.x = (p1.x + p2.x)/2;
	p_out.y = (p1.y + p2.y)/2;
	return p_out;
}

void GenerateLoops::planThread(std::vector<geometry_msgs::Point> rl, std::vector<int> nv){
	std::vector<geometry_msgs::Point> path0 = extractWall(rl, nv, 0);
	geometry_msgs::Point gp0 = closestPoint(path0, curr_pose, rob_lin_vel, rob_rot_vel);
	geometry_msgs::Pose center = getCenter(path0);
	std::vector<geometry_msgs::Point> inner_bound = all_sections[all_sections.size()-1];
	
	if(nv.size() > 0){
		ros::spinOnce();
		
		
		if(getLength(center.position, curr_pose.position) < getLength(gp0, curr_pose.position)){
			std::vector<geometry_msgs::Point> square_points;
			int first_sq, last_sq;
			
			//if robot is inside explore (closer to center than out)
			if((all_sections.size()) == (all_walls.size()*2)){
				square_points = createInnerPointsAlt(inner_bound, sensor_range*2);
			
			}else{
				square_points = createInnerPointsAlt(inner_bound, sensor_range);
			}
			
			for(int ii = 0; ii < square_points.size(); ii++){
				primary_points.push_back(square_points[ii]);
			}
			
			first_sq = 0;
			for(int i = 0; i < square_points.size()-1; i++){
				std::vector<geometry_msgs::Pose> square = getPosesBtw(square_points[i], square_points[i+1], del_goal_points);
				cur_goal_poses.insert(cur_goal_poses.end(), square.begin(), square.end());
			}
			last_sq = cur_goal_poses.size() - 1;
			
			std::vector<geometry_msgs::Point> loop_l = extractWall(rl, nv, nv.size()-1);
			gp0 = closestPoint(loop_l, cur_goal_poses[cur_goal_poses.size()-1], rob_lin_vel, rob_rot_vel);
			primary_points.push_back(gp0);
			std::vector<geometry_msgs::Pose> curr_loop = orderPoses(generatePath(loop_l), gp0);
			curr_loop.erase(curr_loop.begin());
			std::vector<geometry_msgs::Pose> to_first = getPosesBtw
			(cur_goal_poses[cur_goal_poses.size()-1].position, curr_loop[0].position, del_goal_points);
			to_first.erase(to_first.begin());
			
			cur_goal_poses.insert(cur_goal_poses.end(), to_first.begin(), to_first.end());
			
			for(int i = 0; i < cur_goal_poses.size() ; i++){
				loop_sn.push_back(all_walls.size()-1);
			}
			
			//continue with loops from inside out
			std::vector<geometry_msgs::Point> loop = loop_l;
			for(int i = nv.size()-1 ; i > -1; i--){
				loop = extractWall(rl, nv, i);
				geometry_msgs::Point ex = 
				closestPoint(loop, cur_goal_poses[cur_goal_poses.size() - 1], rob_lin_vel, rob_rot_vel);
				primary_points.push_back(ex);
				curr_loop = orderPoses(generatePath(loop), ex);
				cur_goal_poses.insert(cur_goal_poses.end(), curr_loop.begin(), curr_loop.end());
				for(int i1 = 0; i1 < curr_loop.size(); i1++){ loop_sn.push_back(i);} //populate wall sn
			}
			
			//clean up for early inner	
			for(int m = last_sq + 1; m < cur_goal_poses.size(); m++){//not square
				if(cur_goal_poses.size() > (last_sq + 1)){
					for(int l = first_sq; l <= last_sq; l++){//square points
						if(getLength(cur_goal_poses[m].position, cur_goal_poses[l].position) < sensor_range){
							cur_goal_poses.erase(cur_goal_poses.begin() + l);
							loop_sn.erase(loop_sn.begin() + l);
							last_sq --;
							m --;
						}
					}
				}
			}
		

		}else{//else if from outside no rotation but go to center ##fine
			path0 = extractWall(rl, nv, 0);
			gp0 = closestPoint(path0, curr_pose, rob_lin_vel, rob_rot_vel);
			primary_points.push_back(gp0);
			std::vector<geometry_msgs::Pose> curr_loop = orderPoses(generatePath(path0), gp0);
			std::vector<geometry_msgs::Pose> to_first = getPosesBtw
			(curr_pose.position, curr_loop[0].position, del_goal_points);
			to_first.erase(to_first.begin());
			cur_goal_poses.insert(cur_goal_poses.end(), to_first.begin(), to_first.end());
			for(int i1 = 0; i1 < to_first.size()+1; i1++){ loop_sn.push_back(0);}
			cur_goal_poses.insert(cur_goal_poses.end(), curr_loop.begin(), curr_loop.end());
			for(int i1 = 1; i1 < curr_loop.size(); i1++){ loop_sn.push_back(0);}
			
			
			std::vector<geometry_msgs::Point> path = path0;
			
			for(int i = 1; i < nv.size(); i++){
				path = extractWall(rl, nv, i);
				geometry_msgs::Point ex = closestPoint
				(path, cur_goal_poses[cur_goal_poses.size() - 1], rob_lin_vel, rob_rot_vel);
				primary_points.push_back(ex);
				curr_loop = orderPoses(generatePath(path), ex);
				cur_goal_poses.insert(cur_goal_poses.end(), curr_loop.begin(), curr_loop.end());
				for(int i1 = 0; i1 < curr_loop.size(); i1++){ loop_sn.push_back(i);}	
				center = getCenter(path);//not/to overide initial	
			}
			
			
			double diag = getMaxDiagonal(inner_bound);
			double min_side = minSide(inner_bound);
			std::vector<geometry_msgs::Point> square_points;
			if((all_sections.size()) == (all_walls.size()*2)){
				//smallest wall could not be created, last wall has been used to gen path already
				square_points = createInnerPointsAlt(inner_bound, sensor_range*2);
				
			}else{//make a square
				square_points = createInnerPointsAlt(inner_bound, sensor_range);
			}
			
			int lp1 = loop_sn[loop_sn.size() - 1] + 1;
			for(int i = 0; i < square_points.size()-1; i++){
				std::vector<geometry_msgs::Pose> square = getPosesBtw(square_points[i], square_points[i+1], del_goal_points);
				cur_goal_poses.insert(cur_goal_poses.end(), square.begin(), square.end());
				for(int i = 0; i < square.size(); i++){
					loop_sn.push_back(lp1);
				}
			}
		}
		
		global_plan_->poses.resize(cur_goal_poses.size());
		global_plan_->poses = poseToStamped(cur_goal_poses);

	}else{
		ROS_INFO("Small Area Exp");
		
		if(getMaxDiagonal(p_boundary) > sensor_range* 2){
			std::vector<geometry_msgs::Point> ins;
			createInnerEdges(p_boundary, sensor_range/2, &ins); //function multiplies by 2
			std::vector<geometry_msgs::Pose> ins_p;	

			geometry_msgs::Point ex = closestPoint(ins, curr_pose, rob_lin_vel, rob_rot_vel);
			ins_p = orderPoses(generatePath(ins), ex);
			
			std::vector<geometry_msgs::PoseStamped> ins_ps = poseToStamped(ins_p);
			for(int i = 0; i < ins.size(); i++) primary_points.push_back(ins[i]);
			
			global_plan_->poses.resize(ins_ps.size());
			global_plan_->poses = ins_ps;
		}else{
			geometry_msgs::Pose center = getCenter(p_boundary);
			 std::vector<geometry_msgs::Pose> rot = rotateAtPoint(center);
			
			std::vector<geometry_msgs::PoseStamped> ins_ps = poseToStamped(rot);
			for(int i = 0; i < rot.size(); i++) primary_points.push_back(rot[i].position);
			global_plan_->poses.resize(ins_ps.size());
			global_plan_->poses = ins_ps;
		}
	}
	
	for(int i = 0; i < global_plan_->poses.size(); i++){
		global_plan_->poses[i].header.frame_id = map_frame;
		global_plan_->poses[i].header.stamp = ros::Time::now();
	}
	
	for(int ii = 0; ii <global_loops.size(); ii++){
		primary_points.push_back(global_loops[ii]);
	}
	
}

std::vector<geometry_msgs::PoseStamped>  GenerateLoops::rotateAtPoint(geometry_msgs::PoseStamped ref){
	std::vector<geometry_msgs::PoseStamped> p_out;
	geometry_msgs::PoseStamped temp = ref;
	
	for(int i = 3; i >= 0; i--){
		temp.pose = rotatePose(temp.pose, pi *0.5);
		p_out.push_back(temp);
	}
	return p_out;
}

std::vector<geometry_msgs::Pose>  GenerateLoops::rotateAtPoint(geometry_msgs::Pose ref){
	std::vector<geometry_msgs::Pose> p_out;
	geometry_msgs::Pose temp = ref;
	
	for(int i = 3; i >= 0; i--){
		temp = rotatePose(temp, pi *0.5);
		p_out.push_back(temp);
	}
	return p_out;
}

std::vector<geometry_msgs::Point> GenerateLoops::extractWall
(std::vector<geometry_msgs::Point> g_plan, std::vector<int> nodes, int wall){
	std::vector<geometry_msgs::Point> vertices;
	double prev_vert = 0;
	if(nodes.size() > wall && g_plan.size()> 3){
		for(int ii=0; ii < wall; ii++){
			prev_vert += nodes[ii];
		}
		
		for(int i=0; i < nodes[wall]; i++ ){
			vertices.push_back(g_plan[i + prev_vert]);
		}
	}
	else{
		//std::cout<<"wall requested invalid, null returned"<<"\n";
	}

	return vertices;
}

std::vector<geometry_msgs::Pose> GenerateLoops::generatePath(std::vector<geometry_msgs::Point> g_plan){
	//take points and make into poses at fixed distances
	std::vector<geometry_msgs::Pose> poses;
	int ip = 0;
	
	if(g_plan.size() > 2){
		for(int i = 0; i < g_plan.size() - 1; i++){
			
			std::vector<geometry_msgs::Point> tem_p = getPoints(g_plan[i], g_plan[i + 1], del_goal_points);
			poses.resize(poses.size() + tem_p.size() - 1);

			for (int i1 = 0; i1 < (tem_p.size() - 1); i1++ ){ //not taking last point, will come from next
				poses[ip].position = tem_p[i1];			
				ip++;
			}
		}

		std::vector<geometry_msgs::Point> tem_p2 = getPoints(g_plan[g_plan.size() - 1], g_plan[0], del_goal_points);
		poses.resize(poses.size() + tem_p2.size() - 1);
		for(int i2 = 0; i2 < tem_p2.size() - 1; i2++){
			poses[ip].position = tem_p2[i2];		
			ip++;
		}

		for(int ii = 0; ii < poses.size() - 1; ii++ ){
			geometry_msgs::Quaternion tem_q = getQuatFromP(poses[ii].position, poses[ii + 1].position);
			poses[ii].orientation = tem_q;
		}
		poses[poses.size() - 1].orientation = getQuatFromP(poses[poses.size()-1].position, poses[0].position);
		
	}
	else{
		//std::cout<<"empty generate path returned"<<"\n";
	}
	return poses;
}

std::vector<geometry_msgs::Pose> GenerateLoops::orderPoses
(std::vector<geometry_msgs::Pose> loop, geometry_msgs::Point start){
	//link the loops to a continuos path
	std::vector<geometry_msgs::Pose> order;
	
	if(loop.size() > 3){
		int first = 0;
		float diff = getLength(loop[0].position, start); 
		
		order.resize(loop.size());
		for(int i = 0; i < loop.size(); i++){
			if(getLength(loop[i].position, start) < diff){
				first = i;
				diff = getLength(loop[i].position, start);
			}
		}
		for(int i1 = 0; i1 < loop.size(); i1++){
			order[i1] = loop[first];
			if (first < loop.size() - 1){
				first++;
			}
			else{
				first = 0;
			}		
		}
		
		float gap = getLength(order[order.size() -1 ].position, order[0].position);
		std::vector<geometry_msgs::Pose> rec_order = order;
		double orig_grad = gradPoints(order[order.size() -1 ].position, order[0].position);
		
		while (gap < (sensor_range * sqrt(2))){
			if(rec_order.size() > 3){
				order.erase(order.end());
				gap += getLength(rec_order[order.size()].position, rec_order[order.size() -1].position);

				double c_grad = gradPoints(order[order.size() -1 ].position, order[0].position);
				if(std::abs(c_grad - orig_grad) > tan(pi/18) ) break; //ten degrees
			}else break;
		}
	}
	else{
		//std::cout<<"empty order Poses returned"<<"\n";
	}	
	
	return order;
}

geometry_msgs::Point GenerateLoops::closestPoint
(std::vector<geometry_msgs::Point> targets, geometry_msgs::Pose start, float rob_lin_vel, float rob_rot_vel){
	//std::cout<<"closest point"<<"\n";
	geometry_msgs::Point go_to;

	if(targets.size() > 0){
		go_to = targets[0];
		float curr_min = getDistTime(start.position, targets[0], rob_lin_vel) 
		+ getRotationTime(start.orientation, getQuatFromP(start.position, targets[0]), rob_rot_vel);//fix poses first 
		float time = curr_min;

		for(int ii =0; ii < (targets.size() ); ii++){
			std::vector<geometry_msgs::Point> points;
			
			
			if(ii == targets.size() - 1){
				points = getPoints(targets[ii], targets[0], del_points);
			}else points = getPoints(targets[ii], targets[ii+1], del_points);

			for(int i = 0; i < (points.size() - 1); i++){
				time = getDistTime(start.position, points[i], rob_lin_vel) 
					/*+ /*(start.orientation, getQuatFromP(points[i], points[i+1]), rob_rot_vel)*/;//fix poses first
				if(time < curr_min){
					go_to = points[i];
					curr_min = time;
				}		
			}	
		}
	}
	return go_to;
}

float GenerateLoops::getDistTime(geometry_msgs::Point p1, geometry_msgs::Point p2, float speed){
	return getLength(p1, p2)/speed;
}

float GenerateLoops::grad(float x1, float y1, float x2, float y2 ){
	return (y2 - y1) / (x2 - x1);
}

float GenerateLoops::gradPoints(geometry_msgs::Point p1, geometry_msgs::Point p2){
	return (p1.y - p2.y) / (p1.x - p2.x);
}

float GenerateLoops::getRotationTime(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2, float rotSpeed){
	tf::Quaternion tq1, tq2;
	double r1, r2, p1, p2, y1, y2;
		
	tf::quaternionMsgToTF(q1, tq1);
	tf::quaternionMsgToTF(q2, tq2);
	tq1.normalize();
	tq2.normalize();
	
	tf::Matrix3x3 m1(tq1);
	tf::Matrix3x3 m2(tq2);
	m1.getRPY(r1, p1, y1);
	m2.getRPY(r2, p2, y2);
	return std::abs((y2 - y1)/rotSpeed);
}

char GenerateLoops::quad(geometry_msgs::Point origin, geometry_msgs::Point p){
	if(p.x - origin.x >= 0 & p.y - origin.y >= 0){//first quad
	return 1;}
	if(p.x - origin.x <= 0 & p.y - origin.y >= 0){//second quad
	return 2;}
	if(p.x - origin.x <= 0 & p.y - origin.y <= 0){//third quad
	return 3;}
	if(p.x - origin.x >= 0 & p.y - origin.y <= 0){//fourth quad
	return 4;}
	if(p.x - origin.x == 0 & p.y - origin.y == 0){//origin
	return 5;}
}

float GenerateLoops::treatAngle(float ang, char quad){
	if (quad == 2){
		return ang + pi;
	}
	if (quad == 3){
		return ang - pi;
	}
	else {return ang;}
}

float GenerateLoops::posAngle(float ang){
	if (ang < 0){
			return ang + 2 * pi;
	}	
	else {return ang;}
}

float GenerateLoops::getLength(geometry_msgs::Point p1, geometry_msgs::Point p2){
	return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

geometry_msgs::Point GenerateLoops::createPoint(float x, float y, float z){
	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
	point.z = z;
	return point;
}

float GenerateLoops::angleBetween(float a1, float a2){
	float av = (a1 + a2) / 2;
		if(std::abs(a1 - a2) > pi){
			if (av < 0){
				av = av + pi;
			}
			else{
				av = av - pi;
			}
		}
	return av;
}

float GenerateLoops::getPerimeter(std::vector<geometry_msgs::Point> vertices){
	float perimeter = 0;
	for(int i = 0; i < vertices.size() - 1; i++){
		perimeter += getLength(vertices[i], vertices[i + 1]);
	}
	perimeter += getLength(vertices[vertices.size() - 1], vertices[0]);
	return perimeter;
}

bool GenerateLoops::equalPoints(geometry_msgs::Point p1, geometry_msgs::Point p2, float margin){
	if ((std::abs(p1.x-p2.x) < margin) && (std::abs(p1.y-p2.y) < margin)){
		return true;			
	}
	else{
		return false;
	}
}

geometry_msgs::Quaternion GenerateLoops::getQuatFromP(geometry_msgs::Point p1, geometry_msgs::Point p2){
	//std::cout<<"getQuatFromP"<<"\n";
	geometry_msgs::Quaternion mq;
	float z = treatAngle( atan( grad(p1.x, p1.y, p2.x, p2.y)), quad(p1, p2));
	tf::Quaternion tfquat = tf::createQuaternionFromRPY(0.0, 0.0, z);
	tfquat.normalize();
	tf::quaternionTFToMsg(tfquat, mq);
	return mq;
}

std::vector<geometry_msgs::Pose> GenerateLoops::getPosesBtw(geometry_msgs::Point p1, geometry_msgs::Point p2, float del){
	std::vector<geometry_msgs::Pose> poses; 
	std::vector<geometry_msgs::Point> points = getPoints(p1, p2, del);
	std::vector<geometry_msgs::Quaternion> quats;
	for(int i =0; i < points.size() - 1; i++){
		quats.push_back(getQuatFromP(points[i], points[i + 1]));
	}
	quats.push_back(quats[quats.size() - 1]);
	
	poses.resize(points.size());
	for(int i =0; i < points.size(); i++){
		poses[i].position = points[i];
		poses[i].orientation = quats[i];
	} 
	return poses;
}

std::vector<geometry_msgs::Point> GenerateLoops::getPoints
(geometry_msgs::Point p1, geometry_msgs::Point p2, float del_dist){
	
	std::vector<geometry_msgs::Point> points;
	points.push_back(p1);
	
	if(getLength(p1, p2) > del_dist){
		float len = getLength(p1, p2);
		float iter = del_dist;
		float gradient = grad(p1.x, p1.y, p2.x, p2.y);
		while(iter < len){
			geometry_msgs::Point point;
			
			if (p1.x > p2.x){
				point.x = p1.x - sqrt(pow(iter, 2) / (1 + pow(gradient, 2)));
			}
			else{
				point.x = p1.x + sqrt(pow(iter, 2) / (1 + pow(gradient, 2)));
			}
			
			if(std::abs(gradient) < std::numeric_limits<float>::infinity()){ //vertical line
				point.y = gradient * (point.x - p1.x) + p1.y;
			}else{
				if(p1.y < p2.y) point.y = p1.y + iter;
				else point.y =  p1.y - iter;
			}
			points.push_back(point);
			iter += del_dist;
		}
	}
		
	points.push_back(p2);	
	return points;
}

std::vector<geometry_msgs::PoseStamped> GenerateLoops::poseToStamped(std::vector<geometry_msgs::Pose> pose){
	//std::cout<<"poseToStamped"<<"\n";
	std::vector<geometry_msgs::PoseStamped> ps;

	ps.resize(pose.size());
	for (int i = 0; i < pose.size(); i++){
		ps[i].header.frame_id = map_frame;
		ps[i].header.stamp = ros::Time::now();
		ps[i].pose = pose[i];
	}
	return ps; 
}

geometry_msgs::Pose GenerateLoops::getCenter(std::vector<geometry_msgs::Point> wall){
	geometry_msgs::Pose center;

	for(int i = 0; i < wall.size(); i++){
		center.position.x += wall[i].x;
		center.position.y += wall[i].y;
		center.position.z += wall[i].z;
	}
	center.position.x = center.position.x /wall.size();
	center.position.y = center.position.y /wall.size();
	center.position.z = center.position.z /wall.size();
	
	center.orientation.x = 0.0;
	center.orientation.y = 0.0;
	center.orientation.z = 0.0;
	center.orientation.w = 1.0;
	
	return center;
}

geometry_msgs::Pose GenerateLoops::rotatePose(geometry_msgs::Pose origin, double z){
	geometry_msgs::Pose pose_out;
	
	tf::Quaternion q_orig, q_rot, q_new;
	double r = 0, p = 0, y = z;  
	q_rot = tf::createQuaternionFromRPY(r, p, y);

	quaternionMsgToTF(origin.orientation , q_orig); 

	q_new = q_rot*q_orig;  
	q_new.normalize();
	quaternionTFToMsg(q_new, pose_out.orientation); 
	pose_out.position = origin.position;
	
	return pose_out;	
}

geometry_msgs::PolygonStamped GenerateLoops::createPoly(std::vector<geometry_msgs::Point> pv){
	//std::cout<<"create poly"<<"\n";
	geometry_msgs::PolygonStamped ps_polygon;
	ps_polygon.header.frame_id = map_frame;
	ps_polygon.header.stamp = ros::Time::now();
	
	ps_polygon.polygon.points.resize(pv.size());
	for(int i =0; i < pv.size(); i++){
		ps_polygon.polygon.points[i].x = pv[i].x;
		ps_polygon.polygon.points[i].y = pv[i].y;
		ps_polygon.polygon.points[i].z = pv[i].z;
	}
	
	return ps_polygon;	
}

geometry_msgs::Point GenerateLoops::pointOnLine
(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point ref, double dist, bool reverse){
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