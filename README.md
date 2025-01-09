#采取二次规划求解最小化jerk方法

#astar_path_planner包轨迹优化部分需要使用OsqpEigen和Osqp，以下是安装代码

#安装Qsqp

git clone --recursive https://github.com/osqp/osqp.git

cd osqp

mkdir build && cd build

cmake ..

make -j4

sudo make install

cd ../..

#安装OsqpEigen

git clone https://github.com/robotology/osqp-eigen.git

cd osqp-eigen

mkdir build && cd build

cmake ..

make -j4

sudo make install

cd ../..

#运行代码如下：

roslaunch astar_path_planner astar_planner.launch




