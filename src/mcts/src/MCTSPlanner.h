#include "ros/ros.h"
#include "MonteCarloGrove.h"
#include "std_msgs/Float32MultiArray.h"



class MCTSPlanner
{
public:
  int number_of_robots;
  
  // Map variables
  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid coverage_map;
  int received_map;
  int received_coverage_map;

  ros::Subscriber mapsub;
  ros::Subscriber coveragemapsub;

  // Vector of robot states (each state is a Float32MultiArray message)
  std::vector<std_msgs::Float32MultiArray> robot_states;
  std::vector<int> received_states;
  int received_all_states;

  // Vector of robot state subscribers
  std::vector<ros::Subscriber> robot_state_subs;

  // Vector of planned path publishers
  std::vector<ros::Publisher> path_publishers;

  // Solve time variables
  double tic;
  double toc;
  double solve_time;
  double dt;

  // Variable to hold the best paths
  std::vector<nav_msgs::Path> best_paths;

  // Variables to hold the snapshots
  nav_msgs::OccupancyGrid current_map;
  nav_msgs::OccupancyGrid current_coverage_map;
  std::vector<std_msgs::Float32MultiArray> current_robot_states;

  //---------------------------MCTS Specific----------------------------------//

  std::vector<MonteCarloTree >  mctrees;  
  std::vector<MonteCarloTree *>  mctree_pointers;
  MonteCarloGrove grove;

  //---------------------------MCTS Specific----------------------------------//

  
  // CONSTRUCTOR FUNCTION
  MCTSPlanner(int argc, char **argv, int N)
  {
    number_of_robots = N;
    received_map = 0;
    received_coverage_map = 0;
    received_states.resize(number_of_robots);
    received_all_states = 0;
    solve_time = 2.0;
    dt = 0.5;

    current_robot_states.resize(number_of_robots);
    
    // Make a ROS node
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle n;

    // Make a subscriber for the map
    mapsub = n.subscribe<nav_msgs::OccupancyGrid>("map", 1, boost::bind(&MCTSPlanner::mapCallback,this, _1));

    // Make a subscriber for the coverage map
    coveragemapsub = n.subscribe<nav_msgs::OccupancyGrid>("coverage_map", 1, boost::bind(&MCTSPlanner::coverage_mapCallback,this,_1));

    // Make number_of_robots subscribers to the /robotN/state topic
    for(int i=0; i<number_of_robots; i++)
      {
    	std::string topic_string = "/robot" + std::to_string(i) + "/state";
    	ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>(topic_string, 1, boost::bind(&MCTSPlanner::stateCallback, this, _1,i));

    	robot_state_subs.push_back(sub);
      }

    // Make number_of_robots publishers to the /robotN/path topic
    for(int i=0; i<number_of_robots; i++)
      {
    	std::string topic_string = "/robot" + std::to_string(i) + "/path";
    	ros::Publisher pub = n.advertise<nav_msgs::Path>(topic_string,1);
    	path_publishers.push_back(pub);	
      }

    robot_states.resize(number_of_robots);

    while(received_map==0)
      {
	ros::spinOnce();
      }
    while(received_coverage_map==0)
      {
	ros::spinOnce();
      }
    while(received_all_states == 0)
      {
	int min = 1;
	for(int i=0; i<received_states.size(); i++)
	  {
	    if(received_states[i]<min){min = received_states[i];}
	  }
	if(min==1){received_all_states = 1;}
	ros::spinOnce();
      }
    
  }
  
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    // Update the map member variable
    map.header = msg->header;
    map.info = msg->info;
    map.data = msg->data;
    received_map = 1;
  }

  void coverage_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    // Update the map member variable
    coverage_map.header = msg->header;
    coverage_map.info = msg->info;
    coverage_map.data = msg->data;
    received_coverage_map = 1;
  }
  
  
  void stateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg, int which_state)
  {
    robot_states[which_state].data = msg->data;
    received_states[which_state] = 1;
  }

  void predict_future_state(int waypoint)
  {
    if(best_paths.size()>0)
      {
    	for(int i=0; i<number_of_robots; i++)
    	  {
    	    current_robot_states[i].data[0] = best_paths[i].poses[waypoint].pose.position.x;
    	    current_robot_states[i].data[1] = best_paths[i].poses[waypoint].pose.position.y;
    	    current_robot_states[i].data[2] = 2.0*atan2(best_paths[i].poses[waypoint].pose.orientation.z,best_paths[i].poses[waypoint].pose.orientation.w);
    	  }
      }
  }

  void predict_future_coverage_map(int waypoint)
  {
    if(best_paths.size()>0)
      {
	for(int i=0; i<number_of_robots; i++)
	  {
	    // For each timestep
	    for(int t=0; t<waypoint; t++)
	      {
		float x = best_paths[i].poses[t].pose.position.x;
		float y = best_paths[i].poses[t].pose.position.y;
		
		// Find the index of the map that corresponds to the x and y positions
		int map_x = (x-current_coverage_map.info.origin.position.x)/current_coverage_map.info.resolution;
		int map_y = (y-current_coverage_map.info.origin.position.y)/current_coverage_map.info.resolution;
		int index = current_coverage_map.info.width*map_y + map_x;
		
		// If the robot is within the map...
		if(index < current_coverage_map.data.size() && x < (current_coverage_map.info.origin.position.x + current_coverage_map.info.width*current_coverage_map.info.resolution) && y < (current_coverage_map.info.origin.position.y + current_coverage_map.info.height*current_coverage_map.info.resolution))
		  {
		    if( x > (current_coverage_map.info.origin.position.x) && y > (current_coverage_map.info.origin.position.y))
		      {
			// Eliminate the value of the cell the robot is in
			current_coverage_map.data[index] = 0;
			
		      }
		  }
	      }
	  }
      }
  }

  
  
  void plan(float time)
  {
    tic = ros::Time::now().toSec();

    // Get a snapshot of the current map, coverage map, and robot states
    current_map = map;
    current_coverage_map = coverage_map;
    current_robot_states = robot_states;

    // Predict the robot's future state assuming it follows the path for solve_time seconds
    int waypoint = solve_time / dt;    
    predict_future_state(waypoint);

    // Predict what the coverage_map will look like after it follows that path
    predict_future_coverage_map(waypoint);
    

    // --------------------------------MCTS Specific--------------------------//
    // Set the initial states for each robot
    grove.update_initial_maps_and_states(&current_map, &current_coverage_map, &current_robot_states);
    
    // Run MCTS
    while(toc = ros::Time::now().toSec()-tic < time)
      {
	grove.run_parallel_mcts(1);
      }

    // get the best paths from the planner and publish them
    best_paths = grove.return_best_paths();

    // --------------------------------MCTS Specific--------------------------//    

    for(int i=0; i<number_of_robots; i++)
      {
    	path_publishers[i].publish(best_paths[i]);
      }
    toc = ros::Time::now().toSec();
  }  
};




