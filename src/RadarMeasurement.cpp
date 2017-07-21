//
//  RadarMeasurement.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#include "RadarMeasurement.hpp"

const float PI = 3.14159265;

VectorXd RadarMeasurement::CartesianToPolar(const VectorXd &state) {
    
    const float eps = 0.00001;
    
    float px = state(0);
    float py = state(1);
    float vx = state(2);
    float vy = state(3);
    
    double ro = sqrt(px * px + py * py);
    
    double phi = 0.0;
    if (fabs(px) > eps)
        phi = atan2(py, px);
    
    double ro_dot =  0.0;
    if (fabs(ro) > eps)
        ro_dot = (px * vx + py * vy) / ro;
    
    return Eigen::Vector3d(ro, phi, ro_dot);
}

float RadarMeasurement::NormaliseAngle(float angle)
{
    float normilisedAngle = angle;
    
    while (normilisedAngle > PI)
        normilisedAngle -= 2 * PI;
    while (normilisedAngle < -PI)
        normilisedAngle += 2 * PI;
    
    return normilisedAngle;
}

VectorXd RadarMeasurement::GetDiff(VectorXd& state)
{
    VectorXd y = Eigen::Vector3d(ro, phi, ro_dot) - CartesianToPolar(state);
    
    y(1) = NormaliseAngle(y(1));
    
    return y;
}

MatrixXd RadarMeasurement::GetHMatrix(VectorXd& state)
{
    MatrixXd Hj(3,4);
    
    float px = state(0);
    float py = state(1);
    float vx = state(2);
    float vy = state(3);
    
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);
    
    if (fabs(c1) < 0.0001)
        return Hj;
    
    Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    
    return Hj;
}

MatrixXd RadarMeasurement::GetNoiseMatrix()
{
    MatrixXd R = MatrixXd(3, 3);
    R << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
    
    return R;
}

VectorXd RadarMeasurement::GetCartesianCoords()
{
    return Eigen::Vector4d(ro * cos(phi), ro * sin(phi), 0, 0);
}
