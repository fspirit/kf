//
//  LaserMeasurementModel.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#include "LaserMeasurementModel.hpp"

LaserMeasurementModel::LaserMeasurementModel(double xNoiseVar, double yNoiseVar) :
    MeasurementModel(2)
{
    R = MatrixXd(spaceSize, spaceSize);
    R << xNoiseVar, 0,
        0, yNoiseVar;
}

MatrixXd LaserMeasurementModel::GetNoiseMatrix()
{
    return R;
}

VectorXd LaserMeasurementModel::GetZSpacePointsDiff(const VectorXd& pointA, const VectorXd& pointB)
{
    return pointA - pointB;
}

MatrixXd LaserMeasurementModel::GetHMatrix(const VectorXd& cvSpacePoint)
{
    MatrixXd H = MatrixXd(spaceSize, 4);
    H << 1, 0, 0, 0,
        0, 1, 0, 0;
    return H;
}

VectorXd LaserMeasurementModel::CVSpaceToZSpace(const VectorXd &cvSpacePoint)
{
    float x = cvSpacePoint(0);
    float y = cvSpacePoint(1);
    
    return Eigen::Vector2d(x, y);
}

VectorXd LaserMeasurementModel::CTRVSpaceToZSpace(const VectorXd& ctrvSpacePoint)
{
    float x = ctrvSpacePoint(0);
    float y = ctrvSpacePoint(1);
    
    return Eigen::Vector2d(x, y);
};

VectorXd LaserMeasurementModel::ZSpaceToСVSpace(const VectorXd &zSpacePoint)
{
    float x = zSpacePoint(0);
    float y = zSpacePoint(1);
    
    return Eigen::Vector4d(x, y, 0, 0);
}

VectorXd LaserMeasurementModel::ZSpaceToСTRVSpace(const VectorXd &zSpacePoint)
{
    float x = zSpacePoint(0);
    float y = zSpacePoint(1);
    
    VectorXd ctrvSpacePoint(5);
    ctrvSpacePoint << x, y, 0, 0, 0;
    
    return ctrvSpacePoint;
}
