//
//  MeasurementModel.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#ifndef MeasurementModel_hpp
#define MeasurementModel_hpp

#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class MeasurementModel
{
public:
    
    virtual VectorXd GetZSpacePointsDiff(const VectorXd& pointA,
                                         const VectorXd& pointB) = 0;
    virtual MatrixXd GetHMatrix(const VectorXd& cvSpacePoint) = 0;
    virtual MatrixXd GetNoiseMatrix() = 0;
    virtual VectorXd ZSpaceToСVSpace(const VectorXd& zSpacePoint) = 0;
    virtual VectorXd ZSpaceToСTRVSpace(const VectorXd& zSpacePoint) = 0;
    virtual VectorXd CTRVSpaceToZSpace(const VectorXd& ctrvSpacePoint) = 0;
    virtual VectorXd CVSpaceToZSpace(const VectorXd& cvSpacePoint) = 0;
    int ZSpaceSize() { return spaceSize; };
    
protected:
    MeasurementModel(int spaceSize) : spaceSize(spaceSize) {}
    float NormaliseAngle(float angle);
    int spaceSize;
};

#endif /* MeasurementModel_hpp */
