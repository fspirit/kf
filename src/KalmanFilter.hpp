//
//  KalmanFilter.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 18/07/2017.
//
//

#ifndef KalmanFilter_hpp
#define KalmanFilter_hpp

#include <stdio.h>

#include "Eigen/Dense"
#include "MeasurementPackage.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::shared_ptr;

class KalmanFilterK
{
public:
    KalmanFilterK(): isInitialised(false), previousTimestamp(0) {};
    void ProcessMeasurement(shared_ptr<MeasurementPackage> measurement);
    VectorXd GetState() { return state; };

private:
    VectorXd state;
    MatrixXd covariance;
    MatrixXd F;
    bool isInitialised;
    long long previousTimestamp;    
    
    void Initialise(shared_ptr<MeasurementPackage> measurement);
    void Predict(long long timestamp);
    void Update(shared_ptr<MeasurementPackage> measurement);
    float GetTimeDelta(long long timestamp);
    void UpdateFMatrix(float dt);
    MatrixXd CalculateMotionNoise(float dt);
};


#endif /* KalmanFilter_hpp */
