
#include "traj_smooth_closed/visualizer.hpp"
#include "traj_smooth_closed/trajectory.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Path.h"


#include <cmath>
#include <iostream>
#include <vector>

struct Config
{
    std::vector<double> initialVel;
    std::vector<double> initialAcc;
    std::vector<double> terminalVel;
    std::vector<double> terminalAcc;
    double allocationSpeed;
    double allocationAcc;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("InitialVel", initialVel);
        nh_priv.getParam("InitialAcc", initialAcc);
        nh_priv.getParam("TerminalVel", terminalVel);
        nh_priv.getParam("TerminalAcc", terminalAcc);
        nh_priv.getParam("AllocationSpeed", allocationSpeed);
        nh_priv.getParam("AllocationAcc", allocationAcc);
    }
};

double timeTrapzVel(const double dist,
                    const double vel,
                    const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d) 
    {
        return 2.0 * sqrt(dist / acc);
    }
    else 
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

void minimumJerkTrajGen(
    const int pieceNum,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,

    Eigen::MatrixX3d &coefficientMatrix){
   
        int n_order = 5;
        int m = pieceNum * (n_order + 1);
 

        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(m, m); 
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(m, 3); 

        M(0, 0) = 1; 
        M(1, 1) = 1;
        M(2, 2) = 2; 

        B(0,0) = initialPos(0);  
        B(0,1) = initialPos(1);  
        B(0,2) = initialPos(2);  
        B(1,0) = initialVel(0);
        B(1,1) = initialVel(1);
        B(1,2) = initialVel(2);
        B(2,0) = initialAcc(0);
        B(2,1) = initialAcc(1);
        B(2,2) = initialAcc(2);

        for (int k = 0; k < pieceNum - 1; k++)
        {
            Eigen::MatrixXd M_k(Eigen::MatrixXd::Zero(n_order + 1, 2*(n_order + 1)));
            Eigen::MatrixXd B_k(Eigen::MatrixXd::Zero(n_order + 1, 3));
            
            M_k(0,0) = 1;
            M_k(0,1) = timeAllocationVector(k);
            M_k(0,2) = pow(timeAllocationVector(k),2);
            M_k(0,3) = pow(timeAllocationVector(k),3);
            M_k(0,4) = pow(timeAllocationVector(k),4);
            M_k(0,5) = pow(timeAllocationVector(k),5);
            M_k(1,6) = 1;

            M_k(2,1) = 1;
            M_k(2,2) = 2*timeAllocationVector(k);
            M_k(2,3) = 3*pow(timeAllocationVector(k),2);
            M_k(2,4) = 4*pow(timeAllocationVector(k),3);
            M_k(2,5) = 5*pow(timeAllocationVector(k),4);
            M_k(2,7) = -1;

            M_k(3,2) = 2;
            M_k(3,3) = 6*timeAllocationVector(k);
            M_k(3,4) = 12*pow(timeAllocationVector(k),2);
            M_k(3,5) = 20*pow(timeAllocationVector(k),3);
            M_k(3,8) = -2;
            
            M_k(4,3) = 6;
            M_k(4,4) = 24*timeAllocationVector(k);
            M_k(4,5) = 60*pow(timeAllocationVector(k),2);
            M_k(4,9) = -6;

            M_k(5,4) = 24;
            M_k(5,5) = 120*timeAllocationVector(k);
            M_k(5,10) = -24;

            B_k(0,0) = intermediatePositions(0,k); 
            B_k(0,1) = intermediatePositions(1,k);
            B_k(0,2) = intermediatePositions(2,k);

            B_k(1,0) = intermediatePositions(0,k);
            B_k(1,1) = intermediatePositions(1,k);
            B_k(1,2) = intermediatePositions(2,k);
            
            M.block(k * (n_order + 1) + 3, k * (n_order + 1), n_order + 1, 2*(n_order + 1)) = M_k; 
            B.block(k * (n_order + 1) + 3, 0, n_order + 1, 3) = B_k;
        }
        
        M(m-3, m-6) = 1;
        M(m-3, m-5) = timeAllocationVector(pieceNum-1);
        M(m-3, m-4) = pow(timeAllocationVector(pieceNum-1),2);
        M(m-3, m-3) = pow(timeAllocationVector(pieceNum-1),3);
        M(m-3, m-2) = pow(timeAllocationVector(pieceNum-1),4);
        M(m-3, m-1) = pow(timeAllocationVector(pieceNum-1),5);

        M(m-2, m-5) = 1;
        M(m-2, m-4) = 2*timeAllocationVector(pieceNum-1);
        M(m-2, m-3) = 3*pow(timeAllocationVector(pieceNum-1),2);
        M(m-2, m-2) = 4*pow(timeAllocationVector(pieceNum-1),3);
        M(m-2, m-1) = 5*pow(timeAllocationVector(pieceNum-1),4);

        M(m-1, m-4) = 2;
        M(m-1, m-3) = 6*timeAllocationVector(pieceNum-1);
        M(m-1, m-2) = 12*pow(timeAllocationVector(pieceNum-1),2);
        M(m-1, m-1) = 20*pow(timeAllocationVector(pieceNum-1),3);

        B(m-3,0) = terminalPos(0);
        B(m-3,1) = terminalPos(1);
        B(m-3,2) = terminalPos(2);
        B(m-2,0) = terminalVel(0);
        B(m-2,1) = terminalVel(1);
        B(m-2,2) = terminalVel(2);
        B(m-1,0) = terminalAcc(0);
        B(m-1,1) = terminalAcc(1);
        B(m-1,2) = terminalAcc(2);

        coefficientMatrix = M.inverse() * B;
    }

class TrajectoryGenerator
{
private:
    Config config;
    Visualizer visualizer;
    Trajectory<5> traj; 
public:
    ros::Duration duTraj;


public:
    TrajectoryGenerator(const Config &conf, ros::NodeHandle &nh)
                        : config(conf),
                          visualizer(nh),
                          duTraj(0){}

    void trajCalculation(const std::vector<Eigen::Vector3d> &path) {
        int positionNum = path.size();
        if (positionNum > 1) {
            
            ros::Time startTime_1 = ros::Time::now();

            Eigen::VectorXd times(positionNum - 1); 
            Eigen::Matrix3Xd positions_(3,positionNum);
            positions_.col(0) = path[0];
            for (int i = 1; i < positionNum; i++){
                positions_.col(i) = path[i];
                const double dist = (positions_.col(i) - positions_.col(i - 1)).norm();
                times(i-1) = timeTrapzVel(dist, config.allocationSpeed, config.allocationAcc);
            }

            const int pieceNum = positionNum - 1;
            const Eigen::Vector3d initialPos = positions_.col(0);
            const Eigen::Vector3d initialVel(config.initialVel[0], config.initialVel[1], config.initialVel[2]);
            const Eigen::Vector3d initialAcc(config.initialAcc[0], config.initialAcc[1], config.initialAcc[2]);
            const Eigen::Vector3d terminalPos = positions_.col(pieceNum);
            const Eigen::Vector3d terminalVel(config.terminalVel[0], config.terminalVel[1], config.terminalVel[2]);
            const Eigen::Vector3d terminalAcc(config.terminalAcc[0], config.terminalAcc[1], config.terminalAcc[2]);
            const Eigen::Matrix3Xd intermediatePositions = positions_.middleCols(1, pieceNum - 1);
            const Eigen::VectorXd timeAllocationVector = times;

            Eigen::MatrixX3d coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);

            minimumJerkTrajGen(pieceNum,
                               initialPos, initialVel, initialAcc,
                               terminalPos, terminalVel, terminalAcc,
                               intermediatePositions,
                               timeAllocationVector,
                               coefficientMatrix);
            
            traj.reserve(pieceNum);
            for (int i = 0; i < pieceNum; i++)
            {
                traj.emplace_back(timeAllocationVector(i),
                                  coefficientMatrix.block<6, 3>(6 * i, 0).transpose().rowwise().reverse()); 
            }

            ros::Time endTime_1 = ros::Time::now();
            duTraj = endTime_1 - startTime_1;

            visualizer.visualize(traj, positions_); 

            traj.clear(); 

        }

        return;
    }
};


void pathCallback(const nav_msgs::Path::ConstPtr &msg, std::vector<Eigen::Vector3d> &path){
    path.clear(); 
    size_t pathPointNum = msg->poses.size();
    for (size_t i = 0; i < pathPointNum; i++){
        path.emplace_back(
            msg->poses[i].pose.position.x,
            msg->poses[i].pose.position.y,
            msg->poses[i].pose.position.z
        );
    }
}


int main(int argc, char **argv)
{

    setlocale(LC_ALL,"");
    ros::init(argc, argv, "Traj_gen");
    ros::NodeHandle nh;

    std::vector<Eigen::Vector3d> path;
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("path_2",1,
        [&path](const nav_msgs::Path::ConstPtr &msg){
            pathCallback(msg,path);
        }
    );

    TrajectoryGenerator trajGen(Config(ros::NodeHandle("~")), nh);
    ros::Rate rate(10);
    while (ros::ok()) {

        trajGen.trajCalculation(path);
        
        if (path.size()>0) {
            ROS_INFO("Calculation time duration for Trajectory Smooth is :%.5f",trajGen.duTraj.toSec());
        }

        ros::spinOnce(); 

        rate.sleep();
    }

    return 0;
}
