//
//  RadarMeasurementModel.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#include "RadarMeasurementModel.hpp"

RadarMeasurementModel::RadarMeasurementModel(double roNoiseVar,
                                             double phiNoiseVar,
                                             double phiDotNoiseVar) :
    MeasurementModel(3)
{
    R = MatrixXd(spaceSize, spaceSize);
    R << roNoiseVar, 0, 0,
        0, phiNoiseVar, 0,
        0, 0, phiDotNoiseVar;
}

MatrixXd RadarMeasurementModel::GetNoiseMatrix()
{
    return R;
}

VectorXd RadarMeasurementModel::GetZSpacePointsDiff(const VectorXd& pointA, const VectorXd& pointB)
{
    VectorXd diff = pointA - pointB;
    diff(1) = NormaliseAngle(diff(1));
    return diff;
}

MatrixXd RadarMeasurementModel::GetHMatrix(const VectorXd& cvSpacePoint)
{
    MatrixXd Hj(spaceSize, cvSpacePoint.size());
    
    float px = cvSpacePoint(0);
    float py = cvSpacePoint(1);
    float vx = cvSpacePoint(2);
    float vy = cvSpacePoint(3);
    
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

VectorXd RadarMeasurementModel::CVSpaceToZSpace(const VectorXd &cvSpacePoint)
{
    const float eps = 0.00001;
    
    float px = cvSpacePoint(0);
    float py = cvSpacePoint(1);
    float vx = cvSpacePoint(2);
    float vy = cvSpacePoint(3);
    
    double ro = sqrt(px * px + py * py);
    
    double phi = 0.0;
    if (fabs(px) > eps)
        phi = atan2(py, px);
    
    double ro_dot = 0.0;
    if (fabs(ro) > eps)
        ro_dot = (px * vx + py * vy) / ro;
    
    phi = NormaliseAngle(phi);
    
    return Eigen::Vector3d(ro, phi, ro_dot);
}

VectorXd RadarMeasurementModel::CTRVSpaceToZSpace(const VectorXd& ctrvSpacePoint)
{
    double x = ctrvSpacePoint(0);
    double y = ctrvSpacePoint(1);
    double v = ctrvSpacePoint(2);
    double yaw = ctrvSpacePoint(3);
    
    double vx = cos(yaw) * v;
    double vy = sin(yaw) * v;
    
    return CVSpaceToZSpace(Eigen::Vector4d(x, y, vx, vy));
};

VectorXd RadarMeasurementModel::ZSpaceToСVSpace(const VectorXd &zSpacePoint)
{
    float ro = zSpacePoint(0);
    float phi = zSpacePoint(1);
    
    return Eigen::Vector4d(ro * cos(phi), ro * sin(phi), 0, 0);
}

VectorXd RadarMeasurementModel::ZSpaceToСTRVSpace(const VectorXd &zSpacePoint)
{
    float ro = zSpacePoint(0);
    float phi = zSpacePoint(1);
    
    return Eigen::Vector4d(ro * cos(phi), ro * sin(phi), 0, 0);    
}




