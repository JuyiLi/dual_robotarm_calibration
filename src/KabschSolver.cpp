//
// Created by lucia on 25-3-11.
//

#include "KabschSolver.h"
bool KabschSolver::LeastSquareSolve(const std::vector<Eigen::Vector3d> &MainPoints, const std::vector<Eigen::Vector3d> &SlavePoints, Eigen::Isometry3d &T)
{
    std::cout << "MainArmPoints size: " << MainPoints.size() << std::endl;
    std::cout << "SlaveArmPoints size: " << SlavePoints.size() << std::endl;
    size_t N = MainPoints.size();
    if (N == 0 || N != SlavePoints.size())
        return false;
    // 计算两组点的质心
    Eigen::Vector3d centroidSlave = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroidMain = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < N; ++i) {
        centroidSlave += SlavePoints[i];
        centroidMain += MainPoints[i];
    }
    centroidSlave /= static_cast<double>(N);
    centroidMain /= static_cast<double>(N);
    // 构造中心化后的数据矩阵
    // 每一列代表一个点（减去各自质心）
    Eigen::MatrixXd X(3, N); // MainArm
    Eigen::MatrixXd Y(3, N); // SlaveArm
    for (size_t i = 0; i < N; ++i) {
        X.col(i) = MainPoints[i] - centroidMain;
        Y.col(i) = SlavePoints[i] - centroidSlave;
    }

    // 构造协方差矩阵 H = X * Y^T
    Eigen::Matrix3d H = Y * X.transpose();
    // 对 H 进行奇异值分解： H = U * Σ * V^T
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // 根据 Kabsch 算法，旋转矩阵 R = V * U^T
    Eigen::Matrix3d R_MainToSlave = V * U.transpose();

    // 如果 R 的行列式为负，则修正以保证为正交矩阵（避免反射）
    if (R_MainToSlave.determinant() < 0) {
        V.col(2) *= -1;
        R_MainToSlave= V * U.transpose();
    }

    // 平移向量 T = centroidTarget - R * centroidSource
    Eigen::Vector3d t = centroidMain- R_MainToSlave * centroidSlave;

    T.linear() = R_MainToSlave;
    T.translation() = t;
    return true;
}



