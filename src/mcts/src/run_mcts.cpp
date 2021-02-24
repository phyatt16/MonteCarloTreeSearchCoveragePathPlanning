#include "MCTSPlanner.h"

// For convenience of querying the map given x and y coordinates (in meters)
int query_map(nav_msgs::OccupancyGrid * map, float x, float y)
{
  // Find the map index corresponding to the x and y position
  int map_x = (x - map->info.origin.position.x)/map->info.resolution;
  int map_y = (y - map->info.origin.position.y)/map->info.resolution;
  int index = map->info.width*map_y + map_x;
  int value = 0;
  
  // If the robot is within the map...
  if(index < map->data.size() && x < (map->info.origin.position.x + map->info.width*map->info.resolution) && y < (map->info.origin.position.y + map->info.height*map->info.resolution))
    {
      if( x > (map->info.origin.position.x) && y > (map->info.origin.position.y))
	{
	  if(map->data[index] != 0)
	    {
	      // Assign a value of 1 to any square that isn't a 0
	      value = 1;
	    }
	}
    }
  
  else
    {
      value = -1;
    }
  return value;
}

// This is my default policy
int my_fancy_default_policy(nav_msgs::OccupancyGrid * map, nav_msgs::OccupancyGrid * coverage_map, std::vector<float> * state)
  {
    int action = 0;
    float x = (*state)[0];
    float y = (*state)[1];
    float theta = (*state)[2];

    // Look ahead
    x = x + cos(theta)*0.5;
    y = y + cos(theta)*0.5;
    int map_value = query_map(coverage_map,x,y);

    // If there's nothing ahead
    if(map_value != 1)
      {
	// Look left
	x = x - cos(theta)*0.5 + cos(theta+3.14159/2.0)*0.5;
	y = y - cos(theta)*0.5 + cos(theta+3.14159/2.0)*0.5;
	int map_value = query_map(coverage_map,x,y);

	// If there is something there
	if(map_value == 1)
	  {
	    action = 1;
	  }
	// Otherwise look right
	else
	  {
	    // Look right
	    x = x - cos(theta+3.14159/2.0)*0.5 + cos(theta-3.14159/2.0)*0.5;
	    y = y - cos(theta+3.14159/2.0)*0.5 + cos(theta-3.14159/2.0)*0.5;
	    int map_value = query_map(coverage_map,x,y);
	    
	    // If there is something there
	    if(map_value == 1)
	      {
		action = 2;
	      }
	  }
      }
    return action;
  }




int main(int argc, char **argv)
{

  // Create an MCTSPlanner instance
  int num_robots = 4;
  MCTSPlanner planner(argc,argv,num_robots);

  // --------------------------------MCTS Specific--------------------------//  
  planner.grove.establish_default_policy(my_fancy_default_policy);
  planner.grove.plant_trees(num_robots);
  // --------------------------------MCTS Specific--------------------------//  

  while(ros::ok())
    {
      planner.plan(planner.solve_time);
      ros::spinOnce();
    }

  return 0;
}
