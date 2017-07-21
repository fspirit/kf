//
//  LaserMeasurement.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#ifndef LaserMeasurement_hpp
#define LaserMeasurement_hpp

#include <stdio.h>

#include "MeasurementPackage.hpp"

class LaserMeasurement: public MeasurementPackage
{
public:
    LaserMeasurement(float x, float y, long long timestamp) :
        MeasurementPackage(timestamp), x(x), y(y) {};
    
    virtual VectorXd GetDiff(VectorXd& state);
    virtual MatrixXd GetHMatrix(VectorXd& state);
    virtual MatrixXd GetNoiseMatrix();
    virtual VectorXd GetCartesianCoords();
    
private:
    float x;
    float y;
};

#endif /* LaserMeasurement_hpp */
