#include <ros/ros.h> 
#include <utility> 
#include <vector> 
#include <queue> 
#include <cmath> 
#include <Eigen/Dense> 
#include "visualization_msgs/MarkerArray.h" 
#include <geometry_msgs/Point.h> 
#include "nav_msgs/Path.h" 
#include <memory> 

struct Node {
    int x, y;        
    double g_cost;   
    double h_cost;   
    std::shared_ptr<Node> parent;    

    
    
    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {} 

    double f() const { return g_cost + h_cost; } 

};


struct cmp{  
    
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b){
        return a->f() > b->f(); 
    }
  
};


struct GridMap {
    int width;  
    int height;
    double map_max; 
    double map_min; 
    double grid_resolution;
    std::vector<std::vector<int>> grid; 

    
    GridMap(int w, int h, double map_min_, double map_max_, double res) : width(w), height(h), map_min(map_min_), map_max(map_max_), grid_resolution(res), grid(w, std::vector<int>(h, 0)) {}

        void markObstacle(double cx, double cy, double radius) {
                
        int grid_cx = std::round((cx - map_min) / grid_resolution); 
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution); 
        
        for (int dx = -grid_radius; dx <= grid_radius; ++dx) {
            for (int dy = -grid_radius; dy <= grid_radius; ++dy) { 
                int nx = grid_cx + dx;
                int ny = grid_cy + dy;

            
                if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
                    
                    double distance = std::sqrt(dx * dx + dy * dy);
                    if (distance <= grid_radius) {
                        grid[nx][ny] = 1;  
                    }
                                                    
                                       
                }
            }
        }
        
    }
};

class AStarPlanner {
public:
    AStarPlanner(int width, int height, double m_min, double m_max, double res) : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res), grid_map_(width, height, map_min_, map_max_, grid_resolution_), num_of_obs_(0) {

    }

    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
    }


    void printGridMap(){
        for(int i = 0; i < width_; i++){
            for(int j = 0; j < height_; j++){
                std::cout<<grid_map_.grid[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<"num of obstacles: "<<num_of_obs_<<std::endl;
    }

    
    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal) {
    
        if(num_of_obs_ == 0){
            ROS_WARN("No obstacles detected. Returning empty path.");
            return {};
        }else{
            ROS_INFO("obstacle number: %d",num_of_obs_);
        }
        
        auto gridStart = worldToGrid(start);
        auto gridGoal = worldToGrid(goal);

        ROS_INFO("gridStart:(%d,%d), gridGoal:(%d,%d)",gridStart.first,gridStart.second,gridGoal.first,gridGoal.second);

        if (grid_map_.grid[gridStart.first][gridStart.second] == 1 ||
            grid_map_.grid[gridGoal.first][gridGoal.second] == 1) {
            ROS_ERROR("Start or Goal is i坐nside an obstacle, please re-launch the program!");
            return {};
        }

        
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;

        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false)); 

        
        open_list.push(std::make_shared<Node>(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal)));

        
        std::vector<std::vector<bool>> in_open_list(width_, std::vector<bool>(height_, false));
        in_open_list[gridStart.first][gridStart.second] = true;

        
        while (!open_list.empty()) {
            auto current = open_list.top(); 
            open_list.pop();
                       

            if (current->x == gridGoal.first && current->y == gridGoal.second) {
                
                ROS_INFO("optimal path found!!!");
                return reconstructPath(current);
            }

            
            if (closed_list[current->x][current->y]) {
                continue; 
            }

            
            closed_list[current->x][current->y] = true;

            
            std::vector<Node> neighbors = getNeighbors(*current);
            for (const auto& neighbor : neighbors) {
                if (closed_list[neighbor.x][neighbor.y] || grid_map_.grid[neighbor.x][neighbor.y] == 1) {
                    continue;  
                }

                
                double new_g_cost = current->g_cost + distance(*current, neighbor);
                double new_h_cost = heuristic({neighbor.x, neighbor.y}, gridGoal);

                
                auto neighbor_node = std::make_shared<Node>(neighbor.x, neighbor.y, new_g_cost, new_h_cost, current);

                
                
              if (!in_open_list[neighbor.x][neighbor.y]) {
                  
                  open_list.push(neighbor_node);
                  in_open_list[neighbor.x][neighbor.y] = true;
               } else if(new_g_cost < neighbor.g_cost) {
                    
                    open_list.push(neighbor_node);
               }
            }
        }
    
        
        ROS_WARN("No path found from start to goal.");
        return {};
    }

    void reset(){
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }
private:

    
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2)); 
        
    }

    
    double distance(const Node& a, const Node& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    
    
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& position) {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        return {x, y};
    }

    
    Eigen::Vector2d gridToWorld(int x, int y) {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    
    std::vector<Node> getNeighbors(const Node& current) {
        std::vector<Node> neighbors;

        
        std::vector<std::pair<int, int>> directions = {
                {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (const auto& dir : directions) {
            

                
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            
            if (nx >= 0 && ny >= 0 && nx < width_ && ny < height_) {
                
                if (grid_map_.grid[nx][ny] == 1) {
                    continue; 
                }

                                
                neighbors.emplace_back(nx, ny, 0.0, 0.0); 

            }
            
        }

        return neighbors;
    }

    
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) {
        std::vector<Eigen::Vector2d> path;
        while (node) {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end()); 
        reset();
        return path;
    }
      
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_; 
    int num_of_obs_;
};



int main(int argc, char** argv) {

    setlocale(LC_ALL,"");
    ros::init(argc, argv, "astar_planner_1");
    ros::NodeHandle nh;
    double map_min_, map_max_, grid_resolution_;
    double start_x_, start_y_, goal_x_, goal_y_;
    nh.param("astar_planner/map_min", map_min_, -5.0);
    nh.param("astar_planner/map_max", map_max_, 5.0);
    nh.param("astar_planner/grid_resolution", grid_resolution_, 0.1);
    nh.param("astar_planner/start_x", start_x_, -4.5);
    nh.param("astar_planner/start_y", start_y_, -4.5);
    nh.param("astar_planner/goal_x", goal_x_, 4.5);
    nh.param("astar_planner/goal_y", goal_y_, 4.5);

    
    int grid_width = std::round((map_max_ - map_min_) / grid_resolution_);
    int grid_height = grid_width;

    AStarPlanner planner(grid_width, grid_height, map_min_, map_max_, grid_resolution_);
    
    ros::Subscriber obstacle_sub = nh.subscribe<visualization_msgs::MarkerArray>("obstacles", 1,
            [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr& msg) {
                for (const auto& marker : msg->markers) {
                    planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                }
          });

    
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path_1", 1);
    
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);

    ros::Duration(5.0).sleep();

    bool path_found = false; 
    while (ros::ok()) {
        if (path_found) {
            continue;  
        }
        planner.reset();
        ros::Duration(1.0).sleep();
        ros::spinOnce(); 
        

        
        ros::Time search_time_start = ros::Time::now();
        ROS_INFO("start new findPath iteration!");
        std::vector<Eigen::Vector2d> path = planner.findPath(start, goal); 
        ros::Time search_time_end = ros::Time::now();
        ros::Duration search_time_duration = search_time_end - search_time_start;
        ROS_INFO("current findPath iteration finished! Search time=%.5f for A-star using priority_queue",search_time_duration.toSec());
        

        
        
        if (!path.empty()) {
            
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = ros::Time::now();
            for (const auto& point : path) {
                
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = point.x();
                pose.pose.position.y = point.y();
                pose.pose.position.z = 0.0;
                path_msg.poses.push_back(pose);
            }
            path_pub.publish(path_msg);
            ROS_INFO("path published!");
            path_found = true; 
        } else {
            ROS_WARN("Path not found, starting new iteration...");
        }
        rate.sleep();
    }

    return 0;
}