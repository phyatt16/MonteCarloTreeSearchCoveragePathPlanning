/*
This is a simulator of multiple robots doing a coverage task. The robots each have
publishers for their states, and subscribers for their paths. Once they recieve a 
path, they follow it until the end or until they get a new path. They move in a map
that is updated through a subscriber and they update a coverage map which is
published by this simulator.
*/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <math.h>
#include <random>

class MultiRobotSim
{
public:
  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid coverage_map;
  std::vector<nav_msgs::Path> planned_paths;
  std::vector<nav_msgs::Path> actual_paths;  
  std::vector<std_msgs::Float32MultiArray> robot_states;
  int received_map;
  int received_planned_paths;
  std::vector<int> planned_paths_iterator;
  int number_of_robots;
  ros::Publisher coverage_map_pub;
  
  // Vector of robot path subscribers
  std::vector<ros::Subscriber> path_subs;

  // Vector of planned state publishers
  std::vector<ros::Publisher> state_publishers;

  // Vector of robot pose publishers
  std::vector<ros::Publisher> pose_publishers;

  // Vector of robot path publishers
  std::vector<ros::Publisher> actual_path_publishers;
  

  MultiRobotSim(int argc, char **argv, int N)
  {
    number_of_robots = N;
    received_map = 0;
    received_planned_paths = 0;

    robot_states.resize(number_of_robots);
    for(int i=0; i<number_of_robots; i++)
      {
	robot_states[i].data.resize(3);
      }
    
    planned_paths.resize(number_of_robots);
    planned_paths_iterator.resize(number_of_robots);
    actual_paths.resize(number_of_robots);
    
    ros::init(argc, argv,"MultiRobotSim");
    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("map", 10, &MultiRobotSim::map_callback, this);
    coverage_map_pub = n.advertise<nav_msgs::OccupancyGrid>("coverage_map",1);
    
    // Make number_of_robots subscribers to the /robotN/path topic
    for(int i=0; i<number_of_robots; i++)
      {
    	std::string topic_string = "/robot" + std::to_string(i) + "/path";
    	ros::Subscriber sub = n.subscribe<nav_msgs::Path>(topic_string, 1, boost::bind(&MultiRobotSim::pathCallback, this, _1, i));
	path_subs.push_back(sub);
      }


    // Make number_of_robots publishers to the /robotN/state and /robotN/pose topics
    for(int i=0; i<number_of_robots; i++)
      {
    	std::string topic_string = "/robot" + std::to_string(i) + "/state";
    	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>(topic_string,1);
    	state_publishers.push_back(pub);

	std::string topic_string2 = "/robot" + std::to_string(i) + "/pose";
    	ros::Publisher pub2 = n.advertise<geometry_msgs::PoseStamped>(topic_string2,1);
    	pose_publishers.push_back(pub2);
      }

    while(received_map==0)
      {
	ros::spinOnce();
      }
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg, int which_path)
  {
    planned_paths[which_path].header = msg->header;
    planned_paths[which_path].poses = msg->poses;
    received_planned_paths = 1;
    for(int i=0; i<planned_paths_iterator.size(); i++){planned_paths_iterator[i]=0;}
  }


  void publish_states()
  {
    for(int i=0; i<number_of_robots; i++)
      {
	state_publishers[i].publish(robot_states[i]);

	// Create a pose message and publish it
	geometry_msgs::PoseStamped robot_pose;
	robot_pose.header.frame_id = "map";
	robot_pose.pose.position.x = robot_states[i].data[0];
	robot_pose.pose.position.y = robot_states[i].data[1];
	robot_pose.pose.orientation.w = cos(.5*robot_states[i].data[2]);
	robot_pose.pose.orientation.z = sin(.5*robot_states[i].data[2]);
	pose_publishers[i].publish(robot_pose);
	
      }
  }

  void publish_coverage_map()
  {
    coverage_map_pub.publish(coverage_map);
  }

  void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    //Update the map member variable
    map.data = msg->data;
    map.info = msg->info;
    map.header = msg->header;

    std::cout<<"Updated the map"<<std::endl;

    if(received_map == 0)
      {
	received_map = 1;
    
	// Fill the coverage map
	coverage_map.header = map.header;
	coverage_map.info.origin = map.info.origin;
	coverage_map.info.resolution = 0.5;
	coverage_map.info.width = (map.info.resolution*map.info.width)/coverage_map.info.resolution;
	coverage_map.info.height = (map.info.resolution*map.info.height)/coverage_map.info.resolution;
	coverage_map.data.resize(coverage_map.info.width*coverage_map.info.height);
	for(int i=0; i<coverage_map.data.size(); i++)
	  {
	    coverage_map.data[i] = 1;
	  }
      }
  }

  void orderly_initialization()
  {
    for(int i=0; i<number_of_robots; i++)
      {
	robot_states[i].data[0] = -7.25;
	robot_states[i].data[1] = 0.5*i-4.25;
	robot_states[i].data[2] = 0;
      }
  }

  // For convenience of querying the map given x and y coordinates (in meters)
  int query_map(float x, float y)
  {
    // Find the map index corresponding to the x and y position
    int map_x = (x - map.info.origin.position.x)/map.info.resolution;
    int map_y = (y - map.info.origin.position.y)/map.info.resolution;
    int index = map.info.width*map_y + map_x;
    int value = 0;

    // If inside the map...
    if(index < map.info.width*map.info.height && index>=0)
    {
      value = map.data[index];
    }
    return value;
  }

  void update_coverage_map()
  {
    // For Each robot
    for(int i=0; i<number_of_robots; i++)
    {
      float x = robot_states[i].data[0];
      float y = robot_states[i].data[1];

      // Find the index of the map that corresponds to the x and y positions
      int map_x = (x-coverage_map.info.origin.position.x)/coverage_map.info.resolution;
      int map_y = (y-coverage_map.info.origin.position.y)/coverage_map.info.resolution;
      int index = coverage_map.info.width*map_y + map_x;

      // If the robot is within the map...
      if(index < coverage_map.data.size() && x < (coverage_map.info.origin.position.x + coverage_map.info.width*coverage_map.info.resolution) && y < (coverage_map.info.origin.position.y + coverage_map.info.height*coverage_map.info.resolution))
	{
	  if( x > (coverage_map.info.origin.position.x) && y > (coverage_map.info.origin.position.y))
	    {
	      // Eliminate the cost of the cell the robot is in
	      coverage_map.data[index] = 0;
	    }
	}
    }
  }

  // The format for the state is x, y, theta
  void simulate_path_following(float dt)
  {

    // For each robot
    for(int i=0; i<number_of_robots; i++)
      {
	// Find the next point in the robot's path
	geometry_msgs::Pose goal_point = planned_paths[i].poses[planned_paths_iterator[i]].pose;

	// If I am close enough to the end of the path, stop
	float dist_to_goal = sqrt(pow((goal_point.position.y-robot_states[i].data[1]),2.0) + pow((goal_point.position.x-robot_states[i].data[0]),2.0));
	if(planned_paths_iterator[i]!=planned_paths[i].poses.size())
	  {

	    // Find the angle (theta) between my current point and the goal point
	    float theta = atan2(goal_point.position.y-robot_states[i].data[1],goal_point.position.x-robot_states[i].data[0]);
	    (robot_states[i].data[2]) = theta;

	    // Calculate x and y based on the new theta and a move speed of 1 m/s
	    (robot_states[i].data)[0] = (robot_states[i].data)[0] + 1.0*cos((robot_states[i].data)[2])*dt;
	    (robot_states[i].data)[1] = (robot_states[i].data)[1] + 1.0*sin((robot_states[i].data)[2])*dt;

	    // If this position is inside of an obstacle, go back to the previous state
	    int map_value = query_map((robot_states[i].data)[0],(robot_states[i].data)[1]);
	    if(map_value != 0)
	      {
		(robot_states[i].data)[0] = (robot_states[i].data)[0] - 1.0*cos((robot_states[i].data)[2])*dt;
		(robot_states[i].data)[1] = (robot_states[i].data)[1] - 1.0*sin((robot_states[i].data)[2])*dt;
	      }
	    // If I am close enough to the goal point, make the next point in the path the goal point
	    float dist_to_goal = sqrt(pow((goal_point.position.y-robot_states[i].data[1]),2.0) + pow((goal_point.position.x-robot_states[i].data[0]),2.0));
	    if(dist_to_goal<.02 && planned_paths_iterator[i]<planned_paths[i].poses.size())
	      {
		planned_paths_iterator[i]++;
	      }
	  }
	
      }    
  }
};




int main(int argc, char **argv)
{
  // Instantiate an instance of MultiRobotSim class
  int num_robots = 4;
  MultiRobotSim sim(argc, argv, num_robots);  
  
  // Initialize robots in positions in the map
  sim.orderly_initialization();

  // Begin simulating
  int hertz = 100;
  ros::Rate r(hertz); // Hz
  while(ros::ok())
    {

      if(sim.received_planned_paths==1)
	{
	  sim.simulate_path_following((float)1.0/hertz);
	}
      
      sim.update_coverage_map();
      sim.publish_coverage_map();
      sim.publish_states();

      ros::spinOnce();
      r.sleep();
    }
  
  

  return 0;
}
