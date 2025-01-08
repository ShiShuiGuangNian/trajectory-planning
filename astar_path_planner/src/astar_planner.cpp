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
#include <algorithm>
#include <iostream>

// 定义一个节点结构体，用于表示 A* 算法中的每个节点
struct Node {
    int x, y;                        // 节点所在的网格坐标
    double g_cost;                   // 从起点到当前节点的实际代价
    double h_cost;                   // 从当前节点到终点的估计代价（启发式代价）
    std::shared_ptr<Node> parent;    // 父节点，用于回溯路径

    // 构造函数
    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {}

    // 计算总代价（f = g + h）
    double f() const { return g_cost + h_cost; }
};

// 比较器，用于优先队列，根据节点的f值进行排序（小的f值优先）
struct cmp {
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
        return a->f() > b->f();
    }
};

// 网格地图表示结构体
struct GridMap {
    int width;                           // 地图的宽度（网格数）
    int height;                          // 地图的高度（网格数）
    double map_max;                      // 地图的最大坐标值
    double map_min;                      // 地图的最小坐标值
    double grid_resolution;              // 网格分辨率（每个网格的实际尺寸，单位：米）
    std::vector<std::vector<int>> grid;  // 二维网格地图，0: 空闲, 1: 占用

    // 构造函数，初始化网格地图
    GridMap(int w, int h, double map_min_, double map_max_, double res) 
        : width(w), height(h), map_min(map_min_), map_max(map_max_), grid_resolution(res), 
          grid(w, std::vector<int>(h, 0)) {}

    // 标记圆形障碍物为占用
    void markObstacle(double cx, double cy, double radius) {
        // 将圆心坐标从世界坐标转换为网格坐标
        int grid_cx = std::round((cx - map_min) / grid_resolution);
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution);

        // 遍历障碍物圆形区域对应的网格
        for (int i = grid_cx - grid_radius-1; i <= grid_cx + grid_radius+1; ++i) {
            for (int j = grid_cy - grid_radius-1; j <= grid_cy + grid_radius+1; ++j) {
                // 判断网格是否在有效范围内
                if (i >= 0 && i < width && j >= 0 && j < height) {
                    // 计算当前网格到障碍物圆心的距离
                    double dist = std::sqrt(std::pow(i - grid_cx, 2) + std::pow(j - grid_cy, 2));
                    if (dist <= grid_radius+1) {
                        // 如果距离小于等于半径，标记为占用
                        grid[i][j] = 1;
                    }
                }
            }
        }
    }
};

// A* 算法路径规划类
class AStarPlanner {
public:
    // 构造函数，初始化路径规划器
    AStarPlanner(int width, int height, double m_min, double m_max, double res) 
        : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res), 
          grid_map_(width, height, map_min_, map_max_, grid_resolution_), num_of_obs_(0) {}

    // 设置障碍物
    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
    }

    // 打印网格地图，用于调试
    void printGridMap(){
        for(int i = 0; i < width_; i++){
            for(int j = 0; j < height_; j++){
                std::cout<<grid_map_.grid[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<"num of obstacles: "<<num_of_obs_<<std::endl;
    }

    // 查找路径，返回从起点到终点的路径
    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal) {
        if(num_of_obs_ == 0){
            ROS_WARN("No obstacles set. Cannot perform pathfinding.");
            return {};
        }

        // 将起点和终点从世界坐标转换为网格坐标
        auto gridStart = worldToGrid(start);
        auto gridGoal = worldToGrid(goal);

        // 检查起点和终点是否在障碍物上
        if(grid_map_.grid[gridStart.first][gridStart.second] == 1 || grid_map_.grid[gridGoal.first][gridGoal.second] == 1){
            ROS_WARN("Start or goal is on an obstacle.");
            return {};
        }

        // 初始化开放列表（优先队列）和关闭列表（二维布尔数组）
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));

        // 起点加入开放列表
        open_list.push(std::make_shared<Node>(Node(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal))));

        // 用于记录某个节点是否已在开放列表中，避免重复添加
        std::vector<std::vector<bool>> in_open_list(width_, std::vector<bool>(height_, false));
        in_open_list[gridStart.first][gridStart.second] = true;
        
        while (!open_list.empty()) {
          // 获取当前代价最低的节点
          auto current = open_list.top();
          open_list.pop();
          if (closed_list[current->x][current->y] ) {
              continue;
              }
          in_open_list[current->x][current->y] = false; // 从开放列表中移除

          // 如果当前节点是目标节点，回溯路径ros::spinOnce()ros::spinOnce()ros::spinOnce()
          if (current->x == gridGoal.first && current->y == gridGoal.second) {
              return reconstructPath(current);
              }

          // 将当前节点加入关闭列表
          closed_list[current->x][current->y] = true;

          // 遍历当前节点的所有邻居
          for (const auto& neighbor_pos : getNeighbors(*current)) {
              int nx = neighbor_pos.x;
              int ny = neighbor_pos.y;

              // 如果邻居已在关闭列表中，跳过
              if (closed_list[nx][ny]) {
                  continue;
                  }

              // 计算新的g_cost和h_cost
              double new_g_cost = current->g_cost + distance(*current, Node(nx, ny, 0, 0));
              double new_h_cost = heuristic({nx, ny}, gridGoal);

              // 如果邻居不在开放列表中
              if (!in_open_list[nx][ny]) {
                  // 创建新的邻居节点并加入开放列表
                  std::shared_ptr<Node> neighbor = std::make_shared<Node>(Node(nx, ny, new_g_cost, new_h_cost, current));
                  open_list.push(neighbor);
                  in_open_list[nx][ny] = true;
               } else {
                   // 如果邻居已经在开放列表中，重新插入一个更优的节点
                     std::shared_ptr<Node> updated_node = std::make_shared<Node>(Node(nx, ny, new_g_cost, new_h_cost, current));
                     open_list.push(updated_node);
                   }
    }
}


        // 如果没有找到路径，返回空路径
        ROS_WARN("No path found from start to goal.");
        return {};
    }

    // 重置地图（清除所有障碍物）
    void reset(){
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }

private:
    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        double dx = from.first - to.first;
        double dy = from.second - to.second;
        return std::sqrt(dx * dx + dy * dy);
    }

    // 计算两节点之间的距离（用于邻居代价计算）
    double distance(const Node& a, const Node& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // 从世界坐标转换到栅格坐标
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& position) {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        // 确保坐标在地图范围内
        x = std::clamp(x, 0, width_ - 1);
        y = std::clamp(y, 0, height_ - 1);
        return {x, y};
    }

    // 从栅格坐标转换到世界坐标（主要用于路径结果显示）
    Eigen::Vector2d gridToWorld(int x, int y) {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    // 获取当前节点的所有邻居节点
    std::vector<Node> getNeighbors(const Node& current) {
        std::vector<Node> neighbors;

        // 八连通邻居
        std::vector<std::pair<int, int>> directions = {
                {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            // 检查是否在地图范围内
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                // 检查是否为障碍物
                if (grid_map_.grid[nx][ny] == 0) {
                    neighbors.emplace_back(Node(nx, ny, 0, 0)); // 代价会在 findPath 中计算
                }
            }
        }

        return neighbors;
    }

    // 回溯路径，从目标节点回溯到起点
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) {
        std::vector<Eigen::Vector2d> path;
        while (node) {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        reset(); // 重置地图以便下一次搜索
        return path;
    }

    // 地图数据
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_; // 栅格地图，0: 空闲，1: 障碍物
    int num_of_obs_;
};

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
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

    // 地图参数
    int grid_width = std::round((map_max_ - map_min_) / grid_resolution_);
    int grid_height = grid_width;

    AStarPlanner planner(grid_width, grid_height, map_min_, map_max_, grid_resolution_);
    // 障碍物订阅
    ros::Subscriber obstacle_sub = nh.subscribe<visualization_msgs::MarkerArray>("obstacles", 1,
                                                                                 [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr& msg) {
                                                                                     for (const auto& marker : msg->markers) {
                                                                                         planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                                                                                     }
                                                                                 });



    // 发布路径
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    // 起点和终点参数
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);
    while (ros::ok()) {
        planner.reset();
        // 等待障碍物加载
        ros::Duration(1.0).sleep();
        ros::spinOnce();
        // 执行路径搜索
        std::vector<Eigen::Vector2d> path = planner.findPath(start, goal);

        // 路径可视化
        if (path.empty()){
            ROS_WARN("No path found ，hhh");
            continue;
        }
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        path_pub.publish(path_msg);
        rate.sleep();
    }

    return 0;
}