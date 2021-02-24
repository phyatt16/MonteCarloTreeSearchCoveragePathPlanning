/*------------------------------------------------------------------
The MonteCarloNode class serves mostly as a container for information
regarding a node of a MonteCarloTree. It provides some simple methods
for accessing this information and one for calculating the UCT of a
node based on this information.
--------------------------------------------------------------------*/
#include <iostream>
#include <math.h>

class MonteCarloTreeNode
{
public:
  int num_visits;                              // Number of times this node was visited
  int id;                                      // Unique ID for this node within the tree
  std::vector<float> state;
  bool is_leaf;                                // is_leaf==False if all children are created
  float J;                                     // Value associated with this node
  MonteCarloTreeNode* parent;                  // Pointer to parent node
  std::vector<MonteCarloTreeNode*> children;   // Pointers to children nodes
  int action;                                  // Action taken to get to this node

  MonteCarloTreeNode()
  {
    parent=NULL;
    is_leaf=true;
    num_visits=1;
    state.resize(3);
    J = 0;
  }

  void set_state(std::vector<float> x)
  {
    // Set the state of the node equal to the elements of x
    for(int i=0; i<x.size(); i++)
      {
	state[i] = x[i];
      }
  }

  std::vector<MonteCarloTreeNode*> get_children()
  {
    return children;
  }

  float get_UCT(float Cp)
  {
    // Calculate the UCT of this node based on node information
    float UCT = J + 2.0*Cp*sqrt(2*log(parent->num_visits)/num_visits);
    return UCT;
  }
};
