//
// Created by lucia on 25-3-11.
//

#ifndef KABSCHSOLVER_H
#define KABSCHSOLVER_H
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <iostream>


class KabschSolver {
public:
        KabschSolver()=default;
        ~KabschSolver()=default;
        bool LeastSquareSolve(const std::vector<Eigen::Vector3d>& MainPoints,const std::vector<Eigen::Vector3d>& SlavePoints,Eigen::Isometry3d &T);




};



#endif //KABSCHSOLVER_H
