.
├── astar_path_planner
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── astar_planner.launch
│   ├── package.xml
│   ├── rviz
│   │   ├── astar_path_planner_1.rviz
│   │   ├── astar_path_planner_2.rviz
│   │   └── final_draft.rviz
│   └── src
│       ├── astar_planner_1.cpp
│       ├── astar_planner_2.cpp
│       └── obstacle_generator.cpp
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── structure.md
└── traj_smooth_closed            轨迹平滑功能包
    ├── CMakeLists.txt            
    ├── config                    
    │   ├── Traj_gen.rviz         rviz配置文件
    │   └── Traj_gen.yaml         边界条件、时间分配模型参数设置文件
    ├── include
    │   └── traj_smooth_closed
    │       ├── trajectory.hpp    轨迹相关计算所需基本函数
    │       └── visualizer.hpp    轨迹合成、可视化发布相关
    ├── launch
    │   └── LaunchAll.launch      启动文件
    ├── package.xml        
    └── src
        └── Traj_gen.cpp          轨迹平滑节点文件，完成多项式系数解算

10 directories, 19 files
