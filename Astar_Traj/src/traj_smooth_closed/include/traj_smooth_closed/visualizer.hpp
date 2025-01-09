#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "traj_smooth_closed/trajectory.hpp"

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "traj_smooth_closed/TrajData_msgs.h"

class Visualizer
{
private:
    ros::NodeHandle nh;

    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher trajDataPub; 
    

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
        trajDataPub = nh.advertise<traj_smooth_closed::TrajData_msgs>("/plotjuggler/trajData",10);


    }

    template <int D>
    inline void visualize(const Trajectory<D> &traj,
                          const Eigen::Matrix3Xd &route)
    {

        ros::Time current_time = ros::Time::now(); 

        visualization_msgs::Marker wayPointsMarker, trajMarker;

        wayPointsMarker.id = 0;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.header.stamp = current_time;
        wayPointsMarker.header.frame_id = "map";
        wayPointsMarker.pose.orientation.w = 1.00; 
        wayPointsMarker.action = visualization_msgs::Marker::ADD;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.color.a = 1.00;
        wayPointsMarker.scale.x = 0.05;
        wayPointsMarker.scale.y = 0.05;
        wayPointsMarker.scale.z = 0.05;

        trajMarker = wayPointsMarker;

        trajMarker.type = visualization_msgs::Marker::LINE_LIST;
        trajMarker.header.frame_id = "map";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.05;

        for (int i = 0; i < route.cols(); ++i)
        {
            geometry_msgs::Point point;
            point.x = route.col(i)(0);
            point.y = route.col(i)(1);
            point.z = route.col(i)(2);
            wayPointsMarker.points.push_back(point);
        }
        wayPointsPub.publish(wayPointsMarker);
        
        if (traj.getPieceNum() > 0)
        {

            const double T = std::min(0.01, traj.getTotalDuration() / 1000); 
            Eigen::Vector3d lastPos = traj.getPos(0.0); 
            Eigen::Vector3d lastVel = traj.getVel(0.0);
            Eigen::Vector3d lastAcc = traj.getAcc(0.0);
            Eigen::Vector3d lastJerk = traj.getJer(0.0);
            double lastTime = 0.0;

            ROS_INFO("Total motion time: %.2f s",traj.getTotalDuration());

            for (double t = T; t < traj.getTotalDuration(); t += T) 
            {
                geometry_msgs::Point point;
                traj_smooth_closed::TrajData_msgs trajData;

                point.x = lastPos(0); 
                point.y = lastPos(1);
                point.z = lastPos(2);
                trajMarker.points.push_back(point);

                trajData.header.stamp = current_time;
                trajData.timeMotion.x = lastTime;
                trajData.point.x = point.x;
                trajData.point.y = point.y;
                trajData.point.z = point.z;
                trajData.pointVel.x = lastVel(0);
                trajData.pointVel.y = lastVel(1);
                trajData.pointVel.z = lastVel(2);
                trajData.pointAcc.x = lastAcc(0);
                trajData.pointAcc.y = lastAcc(1);
                trajData.pointAcc.z = lastAcc(2);
                trajData.pointJerk.x = lastJerk(0);
                trajData.pointJerk.y = lastJerk(1);
                trajData.pointJerk.z = lastJerk(2);
                trajDataPub.publish(trajData);

                Eigen::Vector3d Pos = traj.getPos(t);
                Eigen::Vector3d Vel = traj.getVel(t);
                Eigen::Vector3d Acc = traj.getAcc(t);
                Eigen::Vector3d Jerk = traj.getJer(t);

                point.x = Pos(0);
                point.y = Pos(1);
                point.z = Pos(2);
                trajMarker.points.push_back(point);

                lastTime = t;
                lastPos = Pos;
                lastVel = Vel;
                lastAcc = Acc;
                lastJerk = Jerk;

            }
            trajectoryPub.publish(trajMarker);
        }
        else
        {
            trajMarker.action = visualization_msgs::Marker::DELETEALL;
            trajectoryPub.publish(trajMarker);
            
            traj_smooth_closed::TrajData_msgs trajData;
            trajData.header.stamp = current_time;
            trajDataPub.publish(trajData);

        }
    }
};

#endif