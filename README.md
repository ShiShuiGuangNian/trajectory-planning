# trajectory-planning
A ROS-based trajectory planning project using QP
轨迹优化部分需要使用OsqpEigen和Osqp，以下是安装代码
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
