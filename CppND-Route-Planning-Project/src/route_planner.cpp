#include "route_planner.h"
#include <algorithm>
#include <typeinfo>
#include <vector>
#include "route_planner.h"

using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
	// convert the starting coords to percentages
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
  
  	// Create a node from the starting end ending coordinates. Save the address of those nodes
  	start_node = &m_Model.FindClosestNode(start_x, start_y);
  	end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  	// calculate the distance from the end node
  	float h_val = node->distance(*RoutePlanner::end_node);
 	return h_val;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
    for (RouteModel::Node* neighbor_node : current_node->neighbors){
      // for each of the neighbor nodes, treat the current node as the parent
      neighbor_node->parent = current_node;
      // add the distance from the current node to the snake
      neighbor_node->g_value = current_node->g_value + neighbor_node->distance(*current_node);
      // treat the neighbor node as visited  
      neighbor_node->visited = true;
      // calculate the h value from the neighbor node
      neighbor_node->h_value = CalculateHValue(neighbor_node);
      //add the neighbor node to the open list
      RoutePlanner::open_list.push_back(neighbor_node);
    }
}

bool HeuristicVal(RouteModel::Node *first, RouteModel::Node *second){
  // compute the cost function for two nodes and weigh them against one another
  // thanks https://www.geeksforgeeks.org/sort-c-stl/
  return (first->h_value + first->g_value) > (second->h_value + second->g_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
  	//sort by the heuristic vector
  	sort(RoutePlanner::open_list.begin(), RoutePlanner::open_list.end(), HeuristicVal);
  	uint16_t vec_length = size(RoutePlanner::open_list);
  	// find the next node to look at from the end of the vector and pop it out of the open list
	RouteModel::Node* next_node = open_list[vec_length-1];
  	RoutePlanner::open_list.pop_back();
	return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

  	while(start_node != current_node){
        // while the start node differs from the current node
  		path_found.push_back(*current_node);
        // add the distance to the parent node, add to the path found vector
  		distance += current_node->distance(*current_node->parent);
      	current_node = current_node->parent;
    }
  	// add the starting node to the end of the vector and then reverse it
    path_found.push_back(*start_node);
  	reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
  	// treat the current node as the start node. mark it as visited and add it to the open list
	RouteModel::Node *current_node = start_node;
  	start_node -> visited = true;
  	RoutePlanner::open_list.push_back(start_node);
  
	while (current_node->distance(*end_node)){
        // while not at the end node, add neighbors and use the heuristic to choose our next node
  		RoutePlanner::AddNeighbors(current_node);
      	current_node = RoutePlanner::NextNode();
    }
  	// construct the final path with the ending node
  	m_Model.path = RoutePlanner::ConstructFinalPath(end_node);
}