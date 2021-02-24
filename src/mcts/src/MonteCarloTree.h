/*------------------------------------------------------------------
The MonteCarloTree class has methods for Selection, Expansion,
and Backpropogation. It includes a simulate function with propogates
the state vector forward in time given an input. These methods are
used in the MonteCarloGrove class to run the Monte Carlo Tree Search
Algorithm. This class also includes methods for dealing with maps. 
It also includes a method to calculate a value
based on the state of the tree and the current map.
--------------------------------------------------------------------*/
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "MonteCarloTreeNode.h"
#include <string>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <random>

class MonteCarloTree
{
public:
  float Cp; // Exploration coefficient
  float HORIZON;
  float dt;

  float best_path_value;
  std::vector<int> best_path;

  // Map variables
  nav_msgs::OccupancyGrid * map;
  nav_msgs::OccupancyGrid * coverage_map;

  // State parameters
  std::vector<float> initstate;

  // Values to be updated and available to all member functions
  std::vector<MonteCarloTreeNode> nodes;
  MonteCarloTreeNode* active_node;
  
  std::vector<int> active_path;
  std::vector<std::vector<float>> active_path_states;
  std::vector<std::vector<float>> best_path_states;
  float t;
  float active_path_value;
  std::default_random_engine generator;

  // Function pointer to the default policy
  int (*default_policy)(nav_msgs::OccupancyGrid * map, nav_msgs::OccupancyGrid * coverage_map, std::vector<float> * state);
  
  MonteCarloTree()
  {
  }

  void initialize(int (&this_policy)(nav_msgs::OccupancyGrid * new_map, nav_msgs::OccupancyGrid * new_coverage_map, std::vector<float> * new_initstates)=MonteCarloTree::straight_policy)
  {
    // Reserve space for the nodes vector (You can get segfaults if this limit is exceeded)
    nodes.reserve(1e6);

    Cp = .5; // Exploration coefficient
    t = 0.0;
    HORIZON = 10;
    dt = 0.5;

    // Variables for the active and best paths of this tree
    active_path_value = 0;
    active_path.resize(HORIZON/dt+1);
    active_path_states.resize((HORIZON/dt+1));
    best_path_states.resize((HORIZON/dt+1));
    best_path.resize(HORIZON/dt+1);
    best_path_value = -9e15;

    // Add the root node to the nodes vector
    MonteCarloTreeNode root_node;
    nodes.push_back(root_node);
    nodes.back().id = nodes.size();

    // Establish the node currently visiting (root node)
    active_node = &nodes.back();

    // Size the initstate
    initstate.resize(3);

    // Establish the default policy
    default_policy = this_policy;
  }

  void reset()
  {
    // Clear all of the nodes
    nodes.clear();

    // Variables for the active and best paths of this tree
    active_path_value = 0;
    active_path.resize(HORIZON/dt+1);
    active_path_states.resize((HORIZON/dt+1));
    best_path_states.resize((HORIZON/dt+1));
    best_path.resize(HORIZON/dt+1);
    best_path_value = -9e15;

    // Add the root node to the nodes vector
    MonteCarloTreeNode root_node;
    nodes.push_back(root_node);
    nodes.back().id = nodes.size();

    // Establish the node currently visiting (root node)
    active_node = &nodes.back();

  }
  
  nav_msgs::Path return_best_path()
  {
    // Return the best path (highest value) explored by this tree
    nav_msgs::Path mypath;
    mypath.header.frame_id = "map";
    mypath.poses.resize(best_path.size());
    for(int i=0;i<active_path_states.size();i++)
    {
      mypath.poses[i].pose.position.x = best_path_states[i][0];
      mypath.poses[i].pose.position.y = best_path_states[i][1];
      mypath.poses[i].pose.orientation.w = cos(.5*best_path_states[i][2]);
      mypath.poses[i].pose.orientation.z = sin(.5*best_path_states[i][2]);
    }
    return mypath;
  }

  void selection()
  {
    // Start at the root node
    active_node = &nodes[0];

    // Increment num_visits of the root node
    active_node->num_visits = active_node->num_visits + 1;

    // Reset the time and active_path_value
    active_path_value = 0;
    t = 0;

    // Reset the active_path to negative ones (default policy)
    std::fill(active_path.begin(), active_path.end(), -1);

    // Include the initstate in the path states
    active_path_states[0] = active_node->state;

    // While we are at a node with all of its children
    while(active_node->is_leaf==false && t<=HORIZON)
    {
      // Find the best child to visit
      int best_child = 0;
      float best_UCT = -1.0e6;

      // For all of this node's children...
      for(int i=0;i<active_node->children.size();i++)
      {
        // If this child has the highest UCT...
        if(active_node->children[i]->get_UCT(Cp) >= best_UCT)
        {
          // Then make it the best child
          best_child = i;
          best_UCT = active_node->children[i]->get_UCT(Cp);
        }
      }

      // Move to the best child
      active_node = active_node->children[best_child];

      // Increment num_visits of the new node
      active_node->num_visits = active_node->num_visits + 1;

      // Record the input for the chosen path
      active_path[t/dt] = active_node->action;

      // Increment time
      t = t + dt;
    }
  }

  void expansion()
  {
    
    // If we haven't reached the end of the simulation horizon...
    if(t<HORIZON)
    {
      // Add a node for a child
      MonteCarloTreeNode node;
      nodes.push_back(node);
      nodes.back().id = nodes.size();

      // Add the child to the parent's children list
      active_node->children.push_back(&nodes.back());

      // Add the parent to the child
      nodes.back().parent = active_node;

      // Inherit state from parent
      nodes.back().state = active_node->state;

      // We are at a leaf node, but if we have added the last child, it is no longer a leaf
      if(active_node->children.size()==3)
      {
        active_node->is_leaf = false;
      }

      // Find the action of the new node (0,1,2)
      int action = 0;
      
      // If this is the first child
      if(active_node->children.size()==0)
	{
	  // Pick any random number
	  std::uniform_int_distribution<int> distribution(0,2);
	  action = distribution(generator);
	}

      // If this is the second child
      else if(active_node->children.size()==1)
	{
	  if(active_node->children[0]->action==0)
	    {
	      std::uniform_int_distribution<int> distribution(1,2);
	      action = distribution(generator);
	    }
	  else if(active_node->children[0]->action==1)
	    {
	      std::uniform_int_distribution<int> distribution(0,1);
	      action = 2*distribution(generator);
	    }
	  else if(active_node->children[0]->action==2)
	    {
	      std::uniform_int_distribution<int> distribution(0,1);
	      action = distribution(generator);
	    }
	}

      // If this is the last child
      else
	{
	  // Pick the number that hasn't been picked yet
	  if(active_node->children[0]->action != 0 && active_node->children[1]->action != 0)
	    {
	      action = 0;
	    }
	  else if(active_node->children[0]->action != 1 && active_node->children[1]->action != 1)
	    {
	      action = 1;
	    }
	  else
	    {
	      action = 2;
	    }
	}

      // Move to the new node
      active_node = &nodes.back();

      // Assign the action to this new node
      active_node->action = action;

      // Find the state of the new node using the simulate method of this tree
      simulate(&active_node->state,action,dt);

      // Record the input for the chosen path
      active_path[t/dt] = action;

      // Increment time
      t = t + dt;
    }
  }


  void back_propogation(float path_value)
  {
    
    // Assign J the path_value of the active node
    active_node->J = path_value;

    // If this node has the highest path value...
    if( path_value > best_path_value)
    {
      // Then the current active_path is the best path
      best_path_value = path_value;
      best_path = active_path;
      best_path_states = active_path_states;
      
    }

    // Step back to the parent
    active_node = active_node->parent;

    // While not at the root node
    while(active_node!=&nodes[0])
    {
      // Make the parent J the average of the childrens' Js
      float avg_J = 0;
      for(int i=0;i<active_node->children.size();i++)
      {
        avg_J = avg_J + active_node->children[i]->J;
      }
      active_node->J = avg_J/((float)active_node->children.size());

      // Step back to its parent
      active_node = active_node->parent;
    }
  }

  // The format for x is x, y, theta
  // The default policy is u==-1
  void simulate(std::vector<float> *x, int u, float dt)
  {
    // If u = -1 do the default policy
    if(u==-1)
    {
      u = default_policy(map, coverage_map, x);
    }
    
    // If u = 1 turn left by pi/2
    if(u==1)
    {
        (*x)[2] = (*x)[2] + 3.14159/2.0;
    }
    // If u = 0 go straight
    if(u==0)
    {
        (*x)[2] = (*x)[2];
    }
    // If u = 2 turn right pi/2
    if(u==2)
    {
        (*x)[2] = (*x)[2] - 3.14159/2.0;
    }

    // Calculate x and y based on the new theta and a move speed of 1 m/s
    (*x)[0] = (*x)[0] + 1.0*cos((*x)[2])*dt;
    (*x)[1] = (*x)[1] + 1.0*sin((*x)[2])*dt;

    // If this position is inside of an obstacle, go back to the previous state
    int map_value = query_map((*x)[0],(*x)[1]);
    if(map_value != 0)
    {
        (*x)[0] = (*x)[0] - 1.0*cos((*x)[2])*dt;
        (*x)[1] = (*x)[1] - 1.0*sin((*x)[2])*dt;
    }
  }

  static int straight_policy(nav_msgs::OccupancyGrid * new_map, nav_msgs::OccupancyGrid * new_coverage_map, std::vector<float> * new_initstates)
  {
    return 0;
  }

  
  // For convenience of querying the map given x and y coordinates (in meters)
  int query_map(float x, float y)
  {
    // Find the map index corresponding to the x and y position
    int map_x = (x - map->info.origin.position.x)/map->info.resolution;
    int map_y = (y - map->info.origin.position.y)/map->info.resolution;
    int index = map->info.width*map_y + map_x;
    int value = 0;

    // If inside the map...
    if(index < map->info.width*map->info.height && index>=0)
    {
      if(map->data[index] != 0)
      {
        // Assign a value of 1 to any square that isn't a 0
        value = 1;
      }
    }
    return value;
  }

  void load_map(nav_msgs::OccupancyGrid * new_map,nav_msgs::OccupancyGrid * new_coverage_map)
  {
    map = new_map;
    coverage_map = new_coverage_map;
  }

};
