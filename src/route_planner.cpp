#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    // Store the nodes found in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

// Use the distance to the end_node for the h value
float RoutePlanner::CalculateHValue(const RouteModel::Node *node)
{
    return node->distance(*end_node);
}

// Expand the current node by adding all unvisited neighbors to the open list.
/// param: current node
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();

    for (auto& each_node : current_node->neighbors)
    {
        each_node->parent = current_node;
        each_node->h_value = RoutePlanner::CalculateHValue(each_node);
        each_node->g_value = current_node->g_value + current_node->distance(*each_node);

        each_node->visited = true;
        this->open_list.push_back(each_node);
    }
}

// Sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode()
{
    RouteModel::Node *node_obj;
    std::sort(this->open_list.begin(), this->open_list.end(), [](const RouteModel::Node *_x, const RouteModel::Node *_y)
              { return _x->h_value + _x->g_value < _y->h_value + _y->g_value; });

    RouteModel::Node *lowest_sum_node = open_list.front();
    this->open_list.erase(this->open_list.begin());

    return lowest_sum_node;
}

//  Iteratively follow the chain of parents of nodes until the starting node is found.
/// param: current (final) node.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;
    // Create path_found vector
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr)
    {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back((*current_node));
    // NOTE: Need to reverse the nodes inside the path_found after finishing the loop.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;
}

// Write the A* Search algorithm.
void RoutePlanner::AStarSearch()
{
    this->start_node->visited = true;
    this->open_list.push_back(this->start_node);

    RouteModel::Node *current_node = nullptr;

    while (open_list.size() > 0)
    {

        current_node = this->NextNode();

        if (current_node->distance(*end_node) == 0)
        {
            this->m_Model.path = ConstructFinalPath(this->end_node);
            return;
        }
        this->AddNeighbors(current_node);
    }
}