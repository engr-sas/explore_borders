Brief:
	g
Getting strated:
	Download packge and place it in a workspace. Build (make) package. "explore_node"is the primary package. A secondary 
	package "measure_distance" may be used to get total translation and rotation of mobile base.  
	
	A mobile base configured with the "move_base" stack is regquired. Simply launch "explore_node" 
	and define exploration area on rviz. An example launch file is included in the launch folder.
	
	The launch file also launches a node that periodically reports the total translation and rotation of the
	mobile base."measure_distance" subscribes to "odom_topic" (can be changed in param see example launch file).

params:
	map_frame: 
		The frame on which the map is being created.
	map_topic: gg
		The topic publishing the map being created.Expects nav_msgs/OccupancyGrid.
	odom_frame: 
		The frame attached to the robot odometry.
	odom_topic: 
		The topic broadcasting the most accurate location of the robot. 
		The data is expected in "nav_msgs/Odogmetry" format.
	scan_topic: 
		The topic broadcasting the scan data from the sensor. Expects sensor_msgs/LaserScan.
	clicked_point:
		The topic broadcasting points used to define exploration area. Expects "geometry_msgs/PointStamped". 
	vertices:
		The number of vertices to be used to define the region to be explored. 
	max_rot_vel:
		Robot maximum linear velocity. Used to estimate a time optimal path for the robot.
	max_lin_vel:
		Robot maximum angular/rotational velocity. Used to estimate a time optimal path for the robot.
	sensor_range:
		Maximum range at which sensor adds obstacles to map (ray trace range). Used to plot initial path through area.
	inflation:
		Secondary map inflation. Should set same as the robot radius.
	robot_span:
		Half of max robot dimension (length or width). Robot radius.
	resolution:
		Innner resolution of secondary map. Highly Influences speed of algorithm. 
		Set lower values in high performing PC for better perfomance.
	
	
Subcribes:
	Default subscriptions; can be changed in params:
		"odom_topic":
		"clicked_point":
		"scan":
		"map":
	Move base subscriptions (required): 
		"/move_base/local_costmap/costmap", 200, costmapCallback);
		"/move_base/local_costmap/costmap_updates", 10, costmapUpCallback);
		"/move_base/status", 100, statusCallback);
		"/move_base/feedback", 200, feedbackCallback);
	
Publishes: 
	"/global_plan_path":
		The general path planned published periodically as nav_msg/Path.
	"/move_base_simple/goal":
		Used to send goals to move_base.
	"/exp_boundary":
		A polygon showing the defined region for exploration. Published as geometry_msgs/PolygonStamped.
	"/active_polygon":
		A polygon showing the polygon of interset in at current stage of exploration.
		Published as geometry_msgs/PolygonStamped.
	"odom_total":
		Sends a request to the "measure_distance" node to publish the total translation and rotation.
		Data is an std_msgs::Bool.