#pragma once
#include <unordered_map>
#include <queue>

#include "multiagent_planning/Point.h"
#include "boost/functional/hash.hpp"

struct Pose2D{
    int x = 0;
    int y = 0;
    float yaw = 0;
};

class Node{
    public:
        int x;
        int y;
        std::vector<std::pair<int, int>> neighbours;

        bool is_visited = false;
        bool is_inqueue = false;
        bool is_inpath = false;

        double distance_from_start = INT_MAX;
        double heuristic = INT_MAX;
        double sum = INT_MAX;

        Node* parent = nullptr;
};

extern std::unordered_map<std::pair<int, int>, Node*, boost::hash<std::pair<int, int>>> node_map;

typedef std::pair<std::pair<int, int>, std::pair<int, int>> pair_of_points;

extern std::unordered_map<pair_of_points, std::vector<std::pair<int, int>>, boost::hash<pair_of_points>> path_buffer;

extern int cost;

struct Compare {
    bool operator()(Node* n1, Node* n2) {
        return n1->sum > n2->sum;
    }
};

extern std::priority_queue<Node*, std::vector<Node*>, Compare> pq;


void make_grid();
double get_heuristic(int node_x, int node_y, int goal_x, int goal_y);
std::vector<std::pair<int, int>> get_path(Node* goal_node, Node* start_node);
std::vector<std::pair<int, int>> get_neighbours(int x, int y);
std::vector<std::pair<int, int>> a_star(int start_x, int start_y, int goal_x, int goal_y);
std::vector<multiagent_planning::Point> convert_path_to_message_type(std::vector<std::pair<int, int>> &path);
void display_path(std::vector<std::pair<int, int>> path);
void save_path(int start_x, int start_y, int goal_x, int goal_y, std::vector<std::pair<int, int>> path);
void clear_path();