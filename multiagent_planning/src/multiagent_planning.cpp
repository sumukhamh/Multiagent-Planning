#include "../include/multiagent_planning.h"
#include "multiagent_planning/Point.h"

std::unordered_map<std::pair<int, int>, Node*, boost::hash<std::pair<int, int>>> node_map;
std::unordered_map<pair_of_points, std::vector<std::pair<int, int>>, boost::hash<pair_of_points>> path_buffer;
int cost = 10;
std::priority_queue<Node*, std::vector<Node*>, Compare> pq;

void make_grid(){
    for(int i = 0; i < 10; i++){
        for(int j = 0; j < 10; j++){
            Node* node = new Node();
            node->x = i;
            node->y = j;
            node->neighbours = get_neighbours(i, j);
            node_map[std::make_pair(i, j)] = node;
            
            node = nullptr;
            delete node;
        }
    }
}

double get_heuristic(int node_x, int node_y, int goal_x, int goal_y){
    return pow((node_x - goal_x), 2) + pow((node_y - goal_y), 2);
}

std::vector<std::pair<int, int>> get_neighbours(int x, int y){
    std::vector<std::pair<int, int>> neighbours;
    if(x-1>=0 && x-1<10 && y>=0 && y<10)
        neighbours.push_back(std::make_pair(x-1, y));
    if(x+1>=0 && x+1<10 && y>=0 && y<10)
        neighbours.push_back(std::make_pair(x+1, y));
    if(x>=0 && x<10 && y-1>=0 && y-1<10)
        neighbours.push_back(std::make_pair(x, y-1));
    if(x>=0 && x<10 && y+1>=0 && y+1<10)
        neighbours.push_back(std::make_pair(x, y+1));
    return neighbours;
}

std::vector<std::pair<int, int>> a_star(int start_x, int start_y, int goal_x, int goal_y){

    std::vector<std::pair<int, int>> path;
    Node* start_node = node_map[std::make_pair(start_x, start_y)];
    Node* goal_node = node_map[std::make_pair(goal_x, goal_y)];

    start_node->distance_from_start = 0;

    pq.push(start_node);

    Node* current_node;
    Node* neighbour_node;
    std::pair<int, int> current_pair;
    std::pair<int, int> goal_pair = std::make_pair(goal_x, goal_y);
    pair_of_points pp;
    
    while(!pq.empty()){
        current_node = pq.top();
        pq.pop();

        current_node->is_visited = true;
        
        if(current_node == goal_node)
            break;

        current_pair = std::make_pair(current_node->x, current_node->y);
        pp = std::make_pair(current_pair, goal_pair);
        if(path_buffer.find(pp) != path_buffer.end()){
            std::vector<std::pair<int, int>> temp_path = get_path(current_node, start_node);
            std::vector<std::pair<int, int>> full_path = path_buffer[pp];
            full_path.insert(full_path.end(), temp_path.begin()+1, temp_path.end());
            return full_path;
        }

        for(auto neighbour_pair : current_node->neighbours){
            neighbour_node = node_map[neighbour_pair];
            if(!neighbour_node->is_visited){
                if(neighbour_node->distance_from_start > current_node->distance_from_start + cost){
                    neighbour_node->distance_from_start = current_node->distance_from_start + cost;
                    neighbour_node->sum = neighbour_node->distance_from_start + get_heuristic(neighbour_node->x, neighbour_node->y, goal_node->x, goal_node->y);
                    neighbour_node->parent = current_node;
                    if(!neighbour_node->is_inqueue){
                        pq.push(neighbour_node);
                        neighbour_node->is_inqueue = true;
                    }
                }
            } 
        }
    }
    
    path = get_path(goal_node, start_node);

    return path;
}

std::vector<std::pair<int, int>> get_path(Node* goal_node, Node* start_node){
    std::vector<std::pair<int, int>> path;
    Node* node = goal_node;
    while(true){
        path.push_back(std::make_pair(node->x, node->y));
        if(node == start_node)
            break;
        node = node->parent;
    }
    return path;
}

std::vector<multiagent_planning::Point> convert_path_to_message_type(std::vector<std::pair<int, int>> &path){
    std::vector<multiagent_planning::Point> point_path;
    multiagent_planning::Point pt;
    for(auto P : path){
        pt.x = P.first;
        pt.y = P.second;
        point_path.push_back(pt);
    }
    return point_path;
}

void display_path(std::vector<std::pair<int, int>> path){
    for(auto p : path){
        node_map[std::make_pair(p.first, p.second)]->is_inpath = true;
    }
    for(int i = 0; i < 10; i++){
        for(int j = 0; j < 10; j++){
            if(node_map[std::make_pair(i, j)]->is_inpath)
                std::cout << '#';
            else
                std::cout << '*';
            std::cout << " ";
        }
        std::cout << std::endl;
    }
} 

void save_path(int start_x, int start_y, int goal_x, int goal_y, std::vector<std::pair<int, int>> path){
    std::pair<int, int> start_pair = std::make_pair(start_x, start_y);
    std::pair<int, int> goal_pair = std::make_pair(goal_x, goal_y);
    pair_of_points pp = std::make_pair(start_pair, goal_pair);
    if(path_buffer.find(pp) == path_buffer.end()){
        path_buffer[pp] = path;
    }
}

void clear_path(){
    for(auto node : node_map){
        node.second->is_visited = false;
        node.second->is_inqueue = false;
        node.second->is_inpath = false;
        node.second->distance_from_start = INT_MAX;
        node.second->heuristic = INT_MAX;
        node.second->sum = INT_MAX;
        node.second->parent = nullptr;
    }
}

