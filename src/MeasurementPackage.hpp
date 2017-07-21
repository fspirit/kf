//
//  MeasurementPackage.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#ifndef MeasurementPackage_hpp
#define MeasurementPackage_hpp

#include <stdio.h>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class MeasurementPackage
{
public:
    MeasurementPackage(long long timestamp) : timestamp(timestamp) {};
    virtual VectorXd GetDiff(VectorXd& state) = 0;
    long long GetTimestamp() { return timestamp; };
    virtual MatrixXd GetHMatrix(VectorXd& state) = 0;
    virtual MatrixXd GetNoiseMatrix() = 0;
    virtual VectorXd GetCartesianCoords() = 0;
    
private:
    long long timestamp;
};

#endif /* MeasurementPackage_hpp */
