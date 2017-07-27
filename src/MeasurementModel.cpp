//
//  MeasurementModel.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#include "MeasurementModel.hpp"

float MeasurementModel::NormaliseAngle(float angle)
{
    float normilisedAngle = angle;
    
    while (normilisedAngle > M_PI) normilisedAngle -= 2 * M_PI;
    while (normilisedAngle < -M_PI) normilisedAngle += 2 * M_PI;
    
    return normilisedAngle;
}
