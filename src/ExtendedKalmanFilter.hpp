//
//  KalmanFilter.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 18/07/2017.
//
//

#ifndef KalmanFilter_hpp
#define KalmanFilter_hpp

#include <memory>

#include "Eigen/Dense"
#include "MeasurementModel.hpp"
#include "Measurement.hpp"
#include "KalmanFilterBase.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::shared_ptr;

class ExtendedKalmanFilter : public KalmanFilterBase
{
public:
    ExtendedKalmanFilter(double aXNoise, double aYNoise):
        aXNoise(aXNoise), aYNoise(aYNoise) {};
    virtual VectorXd GetStateAsCVSpacePoint();
    
protected:
    virtual void Initialise(shared_ptr<MeasurementModel> model,
                            Measurement measurement);
    virtual void Predict(long long timestamp);
    virtual void Update(shared_ptr<MeasurementModel> model,
                        Measurement measurement);
private:
    MatrixXd F;
    double aXNoise;
    double aYNoise;
    
    void UpdateFMatrix(float dt);
    MatrixXd CalculateMotionNoise(float dt);
};


#endif /* KalmanFilter_hpp */
