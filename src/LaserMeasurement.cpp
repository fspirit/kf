//
//  LaserMeasurement.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#include "LaserMeasurement.hpp"

VectorXd LaserMeasurement::GetDiff(VectorXd& state)
{
    MatrixXd H = GetHMatrix(state);
    VectorXd diff = Eigen::Vector2d(x, y) -  H * state;
    
    return diff;
}

MatrixXd LaserMeasurement::GetHMatrix(VectorXd& state)
{
    MatrixXd H = MatrixXd(2, 4);
    H << 1, 0, 0, 0,
        0, 1, 0, 0;
    return H;
}

MatrixXd LaserMeasurement::GetNoiseMatrix()
{
    MatrixXd R = MatrixXd(2, 2);
    R << 0.0225, 0,
        0, 0.0225;
    return R;
}

VectorXd LaserMeasurement::GetCartesianCoords()
{
    return Eigen::Vector4d(x, y, 0, 0);
}
