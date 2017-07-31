//
//  KalmanFilterBase.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#include "KalmanFilterBase.hpp"

void KalmanFilterBase::ProcessMeasurement(shared_ptr<MeasurementModel> model,
                                              Measurement measurement)
{
    if (!isInitialised)
        Initialise(model, measurement);
    else
    {
        Predict(measurement.Timestamp);
        Update(model, measurement);
    }
}

void KalmanFilterBase::Initialise(shared_ptr<MeasurementModel> model,
                                      Measurement measurement)
{    
    previousTimestamp = measurement.Timestamp;
    isInitialised = true;
}

float KalmanFilterBase::GetTimeDelta(long long timestamp)
{
    float dt = (timestamp - previousTimestamp) / 1000000.0;
    previousTimestamp = timestamp;
    
    return dt;
}
