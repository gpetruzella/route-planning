#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
     current_node->FindNeighbors();
     for (auto neighbor_node : current_node->neighbors) {
        neighbor_node->parent = current_node;
        neighbor_node->h_value = CalculateHValue(neighbor_node);
        neighbor_node->g_value = current_node->g_value +  current_node->distance(*neighbor_node);
        open_list.emplace_back(neighbor_node);
        neighbor_node->visited = true;
      }
}

RouteModel::Node *RoutePlanner::NextNode() {
  
  std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* a, RouteModel::Node* b) {
  return (a->h_value + a->g_value) < (b->h_value + b->g_value);
  });
  RouteModel::Node* lowest_node = open_list.front();
  open_list.pop_back();
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
      path_found.emplace_back(*current_node);
      distance += current_node->distance(*current_node->parent);
      current_node = current_node->parent;
    }
    path_found.push_back(*start_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  start_node->visited = true;
  open_list.push_back(start_node);

  while (open_list.size() > 0) {
    current_node = NextNode();
    if (current_node->distance(*end_node) == 0) {
      m_Model.path = ConstructFinalPath(current_node);
      break;
    }
    AddNeighbors(current_node);
  }
  m_Model.path = ConstructFinalPath(end_node);
}
