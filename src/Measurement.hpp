//
//  Measurement.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#ifndef Measurement_hpp
#define Measurement_hpp

#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

struct Measurement
{
    Measurement(VectorXd value, long long timestamp):
        Value(value), Timestamp(timestamp) {}
    VectorXd Value;
    long long Timestamp;    
};

#endif /* Measurement_hpp */
