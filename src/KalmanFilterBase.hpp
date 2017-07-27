//
//  KalmanFilterBase.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#ifndef KalmanFilterBase_hpp
#define KalmanFilterBase_hpp

#include <memory>

#include "Eigen/Dense"
#include "MeasurementModel.hpp"
#include "Measurement.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::shared_ptr;

class KalmanFilterBase
{
public:
    KalmanFilterBase(): isInitialised(false), previousTimestamp(0) {};
    void ProcessMeasurement(shared_ptr<MeasurementModel> model,
                            Measurement measurement);
    VectorXd GetState() { return state; };
protected:
    VectorXd state;
    MatrixXd covariance;
    bool isInitialised;
    long long previousTimestamp;
    
    virtual void Initialise(shared_ptr<MeasurementModel> model,
                    Measurement measurement);
    virtual void Predict(long long timestamp) = 0;
    virtual void Update(shared_ptr<MeasurementModel> model,
                Measurement measurement) = 0;
    float GetTimeDelta(long long timestamp);
};

#endif /* KalmanFilterBase_hpp */
