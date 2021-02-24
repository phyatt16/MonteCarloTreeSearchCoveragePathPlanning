/*------------------------------------------------------------------
The MonteCarloGrove class is designed to take several MonteCarloTree
instances and to run Selection, Expansion, Simulation, and Backpropogation,
as well as other methods, in every tree.

This was specifically designed to find trajectories for multiple robots
performing a coverage task. The main parts to be changed for another
application would be the default policy and the value function.
--------------------------------------------------------------------*/
#include "MonteCarloTree.h"
#include <thread>
#include <ctime>
#include <time.h>
#include <iostream>
#include "std_msgs/Float32MultiArray.h"

class MonteCarloGrove
{
public:

  // Map variables
  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid coverage_map;
  std::vector<nav_msgs::OccupancyGrid> coverage_maps_vector;
  int received_map;

  // Variables dealing with trees
  std::vector<MonteCarloTree> trees;
  std::vector<MonteCarloTree*> tree_pointers;
  int num_trees;
  int HORIZON;
  float dt;

  // Variable for values to be backpropogated through trees
  std::vector<float> global_values;
  std::vector<float> local_values;

  // Function pointer to the default policy
  int (*default_policy)(nav_msgs::OccupancyGrid * new_map, nav_msgs::OccupancyGrid * new_coverage_map, std::vector<float> * new_initstates);

  MonteCarloGrove()
  {
  }

  void plant_trees(int N)
  {
    num_trees = N;
    trees.reserve(num_trees);
    tree_pointers.reserve(num_trees);
    coverage_maps_vector.resize(num_trees);

    // Create the trees in the grove
    for(int i=0;i<num_trees;i++)
    {
      MonteCarloTree mct;
      trees.push_back(mct);
      trees.back().initialize(*default_policy);
      tree_pointers.push_back(&mct);
    }
    global_values.resize(num_trees);
    local_values.resize(num_trees);

    // Assume HORIZON and dt are the same for all trees
    HORIZON = trees[0].HORIZON;
    dt = trees[0].dt;
  }

  void establish_default_policy(int (&this_policy)(nav_msgs::OccupancyGrid * new_map, nav_msgs::OccupancyGrid * new_coverage_map, std::vector<float> * new_initstates)=MonteCarloGrove::straight_policy)
  {
    default_policy = this_policy;
  }

  void update_initial_maps_and_states(nav_msgs::OccupancyGrid * new_map, nav_msgs::OccupancyGrid * new_coverage_map, std::vector<std_msgs::Float32MultiArray> * new_initstates)
  {
    //Update the map member variable
    map.data = new_map->data;
    map.info = new_map->info;
    coverage_map.data = new_coverage_map->data;
    coverage_map.info = new_coverage_map->info;

    for(int i=0; i<num_trees; i++)
      {
	// Reset the tree (clear the nodes)
	trees[i].reset();

	// Update the initial states to the tree
	trees[i].initstate[0] = (*new_initstates)[i].data[0];
	trees[i].initstate[1] = (*new_initstates)[i].data[1];
	trees[i].initstate[2] = (*new_initstates)[i].data[2];

	trees[i].nodes[0].set_state(trees[i].initstate);

	// Update the map to the tree
	trees[i].load_map(&map, &coverage_map);

      }
  }

  void grove_selection(int tree_idx)
  {
    // Reset coverage_maps
    for(int i=0; i<num_trees; i++)
    {
      coverage_maps_vector[i].info = coverage_map.info;
      coverage_maps_vector[i].data = coverage_map.data;
    }

    // Run selection through the tree
    trees[tree_idx].selection();
  }

  void grove_expansion(int tree_idx)
  {
    // Run expansion through this tree
    trees[tree_idx].expansion();
  }


  static int straight_policy(nav_msgs::OccupancyGrid * new_map, nav_msgs::OccupancyGrid * new_coverage_map, std::vector<float> * new_initstates)
  {
    return 0;
  }


  // Use (node->initstate and tree->active_path) + default policy to simulate entire system until HORIZON
  void grove_simulation(int tree_idx)
  {
    // Make a map, global_value, and states vectors to be used in this simulation
    global_values[tree_idx] = 0;
    local_values[tree_idx] = 0;
    std::vector<std::vector<std::vector<float>>> states_vector;
    states_vector.resize(num_trees);

    int sim_steps = HORIZON/dt;
    float t=0;

    for(int i=0;i<num_trees;i++)
    {
      // Get the initstates for each tree
      states_vector[i].resize(sim_steps+1);
      states_vector[i][0] = trees[i].initstate;
    }
    // Increment time
    t = t + dt;
    // Simulate to the end of the HORIZON
    for(int i=0;i<sim_steps;i++)
    {
      // Update state of each tree
      for(int j=0;j<num_trees;j++)
      {
	states_vector[j][i+1] = states_vector[j][i];
        if(j==tree_idx)
        {
          // Use the active_path to simulate this tree
	  trees[j].simulate(&states_vector[j][i+1], trees[j].active_path[i], dt);
	  
          // Record the active_path_states for the chosen path
	  trees[j].active_path_states[t/dt] = states_vector[j][i+1];
        }
        else
        {
          // Use the best_paths to simulate all other trees
          trees[j].simulate(&states_vector[j][i+1], trees[j].best_path[i], dt);
        }
      }
      // Increment time
      t = t + dt;
    }
    // Assign the local and global values based on paths
    value_function(&trees[tree_idx].active_path,&states_vector,&coverage_maps_vector[tree_idx],tree_idx);
  }

  void grove_back_propogation(int tree_idx)
  {
    // Back propogate through this tree
    // Local values seem to be more effective than global ones right now...
    trees[tree_idx].back_propogation(local_values[tree_idx]*1.0 + 1.0*global_values[tree_idx]);
  }

  void run_one_iteration_of_mcts(int tree_idx)
  {
    grove_selection(tree_idx);
    grove_expansion(tree_idx);
    grove_simulation(tree_idx);
    grove_back_propogation(tree_idx);
  }  

  void run_parallel_mcts(int iterations)
  {
    for(int i=0;i<iterations;i++)
      {
	std::vector<std::thread> threads;
	// Start all Trees running MCTS
	for(int j=0; j<num_trees; j++)
	  {
	    threads.push_back(std::thread(&MonteCarloGrove::run_one_iteration_of_mcts,this,j));
	  }

	// Wait for all Trees to finish running MCTS	
	for(int j=0; j<num_trees; j++)
	  {
	    threads[j].join();
	  }
      }
  }

  std::vector<nav_msgs::Path> return_best_paths()
    {
      std::vector<nav_msgs::Path> best_paths;
      best_paths.resize(num_trees);
      for(int i=0; i<num_trees; i++)
	{
	  best_paths[i] = trees[i].return_best_path();
	}
      return best_paths;
    }

    // Clears cells that the robots are in (based on the states vector) then calculates the coverage_map_cost
  float value_function(std::vector<int> * tree_active_path, std::vector<std::vector<std::vector<float>>> * this_states_vector, nav_msgs::OccupancyGrid * this_coverage_map, int tree_index)
  {
    float num_uncovered_cells = 0;

    // For Each tree other than the tree_index one
    for(int i=0; i<num_trees; i++)
    {
      if(i!=tree_index)
	{
	  // For each timestep
	  for(int t=0; t<(*this_states_vector)[0].size(); t++)
	    {
	      
	      float x = (*this_states_vector)[i][t][0];
	      float y = (*this_states_vector)[i][t][1];
	      
	      // Find the index of the map that corresponds to the x and y positions
	      int map_x = (x-this_coverage_map->info.origin.position.x)/this_coverage_map->info.resolution;
	      int map_y = (y-this_coverage_map->info.origin.position.y)/this_coverage_map->info.resolution;
	      int index = this_coverage_map->info.width*map_y + map_x;
	      
	      // If the robot is within the map...
	      if(index < this_coverage_map->data.size() && x < (this_coverage_map->info.origin.position.x + this_coverage_map->info.width*this_coverage_map->info.resolution) && y < (this_coverage_map->info.origin.position.y + this_coverage_map->info.height*this_coverage_map->info.resolution))
		{
		  if( x > (this_coverage_map->info.origin.position.x) && y > (this_coverage_map->info.origin.position.y))
		    {
		      // Eliminate the cost of the cell the robot is in
		      this_coverage_map->data[index] = 0;
		    }
		}
	    }
	}
    }

    // For the tree_index tree
    // For each timestep
    for(int t=0; t<(*this_states_vector)[0].size(); t++)
      {

	// Penalize turning
	if((*tree_active_path)[t]==1 || (*tree_active_path)[t]==2)
	  {
	    local_values[tree_index] = local_values[tree_index] - .1;
	  }
	
	float x = (*this_states_vector)[tree_index][t][0];
	float y = (*this_states_vector)[tree_index][t][1];
	
	// Find the index of the map that corresponds to the x and y positions
	int map_x = (x-this_coverage_map->info.origin.position.x)/this_coverage_map->info.resolution;
	int map_y = (y-this_coverage_map->info.origin.position.y)/this_coverage_map->info.resolution;
	int index = this_coverage_map->info.width*map_y + map_x;
	
	// If the robot is within the map...
	int within = 0;
	if(index < this_coverage_map->data.size() && x < (this_coverage_map->info.origin.position.x + this_coverage_map->info.width*this_coverage_map->info.resolution) && y < (this_coverage_map->info.origin.position.y + this_coverage_map->info.height*this_coverage_map->info.resolution))
	  {
	    if( x > (this_coverage_map->info.origin.position.x) && y > (this_coverage_map->info.origin.position.y))
	      {
		within = 1;
		if(this_coverage_map->data[index] != 0)
		  {
		    // Reward this robot if it is clearing a cell (especially soon)
		    local_values[tree_index] = local_values[tree_index] + 1.0*pow(1.0/(1*t+1),0.25);
		  }
		// Eliminate the cost of the cell the robot is in
		this_coverage_map->data[index] = 0;
	      }
	  }
      }
    // Calculate cost based on the entire map (cost on cells not yet covered)
    for(int i=0;i<this_coverage_map->data.size();i++)
    {
      num_uncovered_cells = num_uncovered_cells + this_coverage_map->data[i];
    }

    global_values[tree_index] = -num_uncovered_cells/this_coverage_map->data.size();
  }

};
