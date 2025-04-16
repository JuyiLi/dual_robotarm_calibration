#include "dual_robotarm_calibration.h"

using namespace std;

bool dual_robotarm_calibration::Three_points_calibration(const vector<Eigen::Vector3d>& master_point,
                               const vector<Eigen::Vector3d>& slave_point,
                               Eigen::Isometry3d &master2slave)
{
    if (master_point.size() != 3 || slave_point.size() != 3)
    {
        cerr << "Error: wrong point number (should be 3 points exactly)" << endl;
        return false;
    }

    Eigen::Vector3d m_p1 = master_point[0];
    Eigen::Vector3d m_p2 = master_point[1];
    Eigen::Vector3d m_p3 = master_point[2];
    Eigen::Vector3d s_p1 = slave_point[0];
    Eigen::Vector3d s_p2 = slave_point[1];
    Eigen::Vector3d s_p3 = slave_point[2];

    // check if there are two points too close to each other
    if ((s_p1 - s_p2).norm() < eps || (s_p1 - s_p3).norm() < eps || (s_p2 - s_p3).norm() < eps)
    {
        cerr<<"Error: coincident points exists"<<endl;
        return false;
    }
    // calculate unit vector in master-arm base coordinate
    Eigen::Vector3d m_u = (m_p2 - m_p1).normalized();
    Eigen::Vector3d m_w = (m_p3 - m_p1).cross(m_p2 - m_p1).normalized();
    if (m_w.norm() < eps)
    {
        cerr<<"Error: 3 points are in a line"<<endl;
        return false;
    }
    Eigen::Vector3d m_v = m_w.cross(m_u);

    // calculate unit vector in slave-arm base coordinate
    Eigen::Vector3d s_u = (s_p2 - s_p1).normalized();
    Eigen::Vector3d s_w = (s_p3 - s_p1).cross(s_p2 - s_p1).normalized();
    Eigen::Vector3d s_v = s_w.cross(s_u);

    // get rotation and translation matrix from there-point coordinate to master and slave coordinate
    Eigen::Isometry3d T_3toMain, T_3toSlave;
    Eigen::Matrix3d R_3toMain, R_3toSlave;

    R_3toMain.col(0) = m_u;
    R_3toMain.col(1) = m_v;
    R_3toMain.col(2) = m_w;
    T_3toMain.linear() = R_3toMain;
    T_3toMain.translation() = m_p1;

    R_3toSlave.col(0) = s_u;
    R_3toSlave.col(1) = s_v;
    R_3toSlave.col(2) = s_w;
    T_3toSlave.linear() = R_3toSlave;
    T_3toSlave.translation() = s_p1;

    // get rotation and translation matrix from master to slave coordinate
    master2slave = T_3toMain * T_3toSlave.inverse();
    return true;
}
