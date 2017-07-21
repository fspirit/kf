//
//  RadarMeasurement.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#ifndef RadarMeasurement_hpp
#define RadarMeasurement_hpp

#include <stdio.h>

#include "MeasurementPackage.hpp"

class RadarMeasurement: public MeasurementPackage
{
public:
    RadarMeasurement(float ro, float phi, float ro_dot, long long timestamp):
        MeasurementPackage(timestamp), ro(ro), phi(phi), ro_dot(ro_dot) {};
    
    virtual VectorXd GetDiff(VectorXd& state);
    virtual MatrixXd GetHMatrix(VectorXd& state);
    virtual MatrixXd GetNoiseMatrix();
    virtual VectorXd GetCartesianCoords();
    
private:
    float ro;
    float phi;
    float ro_dot;
    
    float NormaliseAngle(float angle);
    VectorXd CartesianToPolar(const VectorXd &state);
};

#endif /* RadarMeasurement_hpp */
