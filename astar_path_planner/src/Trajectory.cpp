#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>   // 用于 pow, sqrt 等数学函数
#include <memory>  // 用于智能指针（例如 std::shared_ptr, std::unique_ptr）
#include <OsqpEigen/OsqpEigen.h>


// 定义路径点类型
typedef std::vector<Eigen::Vector2d> Path;

// 定义多项式系数类型
typedef Eigen::VectorXd Polynomial;

class TrajectoryGenerator {
public:
    TrajectoryGenerator() = default;

    // 主函数：生成轨迹
    std::pair<std::vector<Polynomial>, std::vector<Polynomial>>  generateTrajectory(const Path& path, double avg_speed) {
        // 检查路径点是否足够
        if (path.size() < 2) {
            ROS_WARN("Path must contain at least two points.");
            return {};
        }

        // 分配时间：每段路径固定时间分配
        std::vector<double> time_segments = allocateTime(path, avg_speed);

        // 构造 Q 矩阵（目标函数）
        Eigen::MatrixXd Q = constructQMatrix(time_segments);

        // 构造 A 矩阵和 b 向量（约束条件）
        Eigen::MatrixXd A;
        Eigen::VectorXd bx;
        Eigen::VectorXd by;
        constructConstraintMatrices(path, time_segments, A, bx,by);

        // 求解二次规划
        Eigen::VectorXd coeffsx = solveQP(Q, A, bx);
        Eigen::VectorXd coeffsy = solveQP(Q, A, by);

         // 提取多项式
        auto x_polys = extractPolynomials(coeffsx, path.size() - 1);
        auto y_polys = extractPolynomials(coeffsy, path.size() - 1);
        ROS_INFO("QP problem solved successfully. Optimal trajectory generated.");

        // 同时返回 x 和 y 的多项式
        return {x_polys, y_polys};
    }

private:
    // 时间分配函数
    std::vector<double> allocateTime(const Path& path, double avg_speed) {
        std::vector<double> time_segments;
        for (size_t i = 1; i < path.size(); ++i) {
            double dist = (path[i] - path[i - 1]).norm(); // 欧几里得距离
            time_segments.push_back(dist / avg_speed);   // 固定平均速度
        }
        return time_segments;
    }

    // 构造 Q 矩阵（目标函数：最小化 Jerk 的平方积分）
    Eigen::MatrixXd constructQMatrix(const std::vector<double>& time_segments) {
        int num_segments = time_segments.size();
        int num_coeffs = 6 * num_segments; // 每段 6 个系数
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_coeffs, num_coeffs);

        for (int i = 0; i < num_segments; ++i) {
            double T = time_segments[i];
            Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(6, 6);

            Qk(3, 3) = 36 * T;
            Qk(3, 4) = 72 * T * T;
            Qk(3, 5) = 120 * T * T * T;
            Qk(4, 4) = 192 * T * T * T;
            Qk(4, 5) = 360 * T * T * T * T;
            Qk(5, 5) = 720 * T * T * T * T * T;
            Eigen::MatrixXd temp = Qk + Qk.transpose();
            Qk = temp;
            Qk(3, 3) /= 2;
            Qk(4, 4) /= 2;
            Qk(5, 5) /= 2;

            Q.block(6 * i, 6 * i, 6, 6) = Qk;
        }
        return Q;
    }
    // 构造 A 矩阵和 b 向量（位置、速度、加速度连续约束）
    void constructConstraintMatrices(const Path& path, const std::vector<double>& time_segments, Eigen::MatrixXd& A, Eigen::VectorXd& bx, Eigen::VectorXd& by) {
    // 确保路径点和时间片段数量正确
    int num_segments = path.size() - 1; // 路径段数
    if (time_segments.size() != num_segments) {
        std::cerr << "Error: Mismatch between path size and time_segments size." << std::endl;
        return;
    }

    // 将时间片段转换为全局时间数组 T
    std::vector<double> T(num_segments + 1, 0.0);
    for (int i = 1; i <= num_segments; ++i) {
        T[i] = T[i - 1] + time_segments[i - 1];
    }

    // 将路径拆分为 X 和 Y
    std::vector<double> X, Y;
    for (const auto& point : path) {
        X.push_back(point.x());
        Y.push_back(point.y());
    }

    int M = path.size() - 1; // 段数
    int N = M * 6;           // 系数数量（每段6个系数）
    int K = 3;               // 约束最高阶为加速度
    int num_constraints = 2 * K + (M - 1) + (M - 1) * 3; // 总约束数

    // 初始化 A 和 b
    A = Eigen::MatrixXd::Zero(num_constraints, N);
    bx = Eigen::VectorXd::Zero(num_constraints);
    by = Eigen::VectorXd::Zero(num_constraints);
    int constraint_idx = 0;

    // 添加首末状态约束（位置、速度、加速度）
    for (int k = 0; k < K; ++k) {
        for (int i = k; i < 6; ++i) {
            double c = 1;
            for (int j = 0; j < k; ++j) {
                c *= (i - j);
            }

            // 起点约束
            A(constraint_idx, i) = c * std::pow(T[0], i - k);
            // 终点约束
            A(constraint_idx + 1, (M - 1) * 6 + i) = c * std::pow(T[M], i - k);
        }
        bx(constraint_idx) = (k == 0) ? X[0] : 0.0;
        by(constraint_idx) = (k == 0) ? Y[0] : 0.0;
        bx(constraint_idx + 1) = (k == 0) ? X[M] : 0.0;
        by(constraint_idx + 1) = (k == 0) ? Y[M] : 0.0;
        constraint_idx += 2;
    }

    // 添加每段轨迹的初始位置约束
    for (int m = 1; m < M; ++m) {
        for (int i = 0; i < 6; ++i) {
            A(constraint_idx, m * 6 + i) = std::pow(T[m], i);
        }
        bx(constraint_idx) = X[m];
        by(constraint_idx) = Y[m];
        ++constraint_idx;
    }

    // 添加连续性约束（位置、速度、加速度）
    for (int m = 0; m < M - 1; ++m) {
        for (int k = 0; k < 3; ++k) { // 最多两阶导数相等
            for (int i = k; i < 6; ++i) {
                double c = 1;
                for (int j = 0; j < k; ++j) {
                    c *= (i - j);
                }

                // 第 m 段的终点导数
                A(constraint_idx, m * 6 + i) = c * std::pow(T[m + 1], i - k);
                // 第 m+1 段的起点导数
                A(constraint_idx, (m + 1) * 6 + i) = -c * std::pow(T[m + 1], i - k);
            }
            ++constraint_idx;
        }
    }
}


   // 使用 OSQP-Eigen 求解二次规划问题
 Eigen::VectorXd solveQP(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    // 初始化 OSQP 求解器
    OsqpEigen::Solver solver;

    // 获取问题维度
    int num_vars = Q.rows();          // 优化变量个数（轨迹系数总数）
    int num_constraints = A.rows();  // 约束数量

    // 设置优化问题的尺寸
    solver.data()->setNumberOfVariables(num_vars);
    solver.data()->setNumberOfConstraints(num_constraints);

    // 设置目标函数 (1/2 x^T Q x + 0^T x)
    Eigen::SparseMatrix<double> Q_sparse = Q.sparseView().eval();
    solver.data()->setHessianMatrix(Q_sparse);

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num_vars);
    solver.data()->setGradient(gradient);

    // 设置等式约束 (A x = b)
    Eigen::SparseMatrix<double> A_sparse = A.sparseView().eval();
    solver.data()->setLinearConstraintsMatrix(A_sparse);

    Eigen::VectorXd lower_bound = b;
    Eigen::VectorXd upper_bound = b;
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);
    solver.settings()->setMaxIteration(10000);
    solver.settings()->setAbsoluteTolerance(1e-2);
    solver.settings()->setRelativeTolerance(1e-2);

    // 初始化求解器
    if (!solver.initSolver()) {
        throw std::runtime_error("Failed to initialize OSQP solver.");
    }

    // 求解二次规划问题
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    ROS_WARN("Failed to solve QP problem with OSQP. Using default trajectory.");
    return Eigen::VectorXd::Zero(Q.rows()); // 返回一个零向量以避免后续错误
}


    // 返回求解结果
    return solver.getSolution();
}





    // 提取多项式系数
    std::vector<Polynomial> extractPolynomials(const Eigen::VectorXd& coeffs, int num_segments) {
        std::vector<Polynomial> polynomials;

        for (int i = 0; i < num_segments; ++i) {
            Polynomial p = coeffs.segment<6>(i * 6); // 每段 6 个系数
            polynomials.push_back(p);
        }

        return polynomials;
    }
};

// 全局变量，用于存储订阅到的路径
Path global_path;

// 回调函数：接收 A* 发布的路径消息
void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    global_path.clear();
    for (const auto& pose : msg->poses) {
        Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
        global_path.push_back(point);
    }
    ROS_INFO("ReceiveRd path with %ld points", global_path.size());
}
std::vector<double> calculateTimeSegments(const Path& path, double avg_speed) {
    std::vector<double> time_segments;
    // 遍历路径点，计算每段路径的时间
    for (int i = 1; i < path.size(); ++i) {
        double dist = (path[i] - path[i - 1]).norm(); // 计算路径点之间的欧几里得距离
        time_segments.push_back(dist / avg_speed);    // 根据平均速度计算时间
    }
    
    std::vector<double> T1(path.size(), 0.0);
    for (int i = 1; i < path.size(); ++i) {
        T1[i] = T1[i - 1] + time_segments[i - 1];
    }
    return T1;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_planner");
    ros::NodeHandle nh;

    double avg_speed;
    nh.param("trajectory_planner/avg_speed", avg_speed, 1.0);

    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("path", 1, pathCallback);
    ros::Publisher traj_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 1,true);

    TrajectoryGenerator trajectory_generator;

    Path local_path;
    ros::Duration(2).sleep();
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        // 更新局部路径
        if (!global_path.empty()) {
            local_path = global_path;
            global_path.clear();
            break;
        }

        // 检查路径是否为空
        if (local_path.empty()) {
            loop_rate.sleep();
            continue;
        }
        loop_rate.sleep();
    }
    // 生成轨迹
    auto [x_trajectories, y_trajectories] = trajectory_generator.generateTrajectory(local_path, avg_speed);
    auto T1 = calculateTimeSegments(local_path, avg_speed);
    // 可视化轨迹
    visualization_msgs::MarkerArray traj_markers;
    int marker_id = 0;
    for (size_t i = 0; i < x_trajectories.size(); ++i) {
        const auto& x_poly = x_trajectories[i];
        const auto& y_poly = y_trajectories[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;

        for (double t = T1[i]; t <= T1[i+1]; t += 0.01) {
            double x = 0.0, y = 0.0;
            for (int j = 0; j < x_poly.size(); ++j) {
                x += x_poly[j] * std::pow(t, j);
                y += y_poly[j] * std::pow(t, j);
            }
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
        traj_markers.markers.push_back(marker);
        }

    // 发布轨迹
    while (ros::ok()) {
    if (!traj_markers.markers.empty()) {
        traj_pub.publish(traj_markers);
        
    } else {
        ROS_WARN("Generated trajectory markers are empty.");
    }
    loop_rate.sleep();
    }

    return 0;
}
