//
//  KalmanFilter.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 18/07/2017.
//
//

#include "ExtendedKalmanFilter.hpp"

void ExtendedKalmanFilter::Initialise(shared_ptr<MeasurementModel> model,
                                      Measurement measurement)
{
    F = MatrixXd(4, 4);
    F << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
    covariance = MatrixXd(4, 4);
    covariance << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1.5, 0,
        0, 0, 0, 1.5;
    
    state = model->ZSpaceToСVSpace(measurement.Value);
    
    KalmanFilterBase::Initialise(model, measurement);
}

MatrixXd ExtendedKalmanFilter::CalculateMotionNoise(float dt)
{
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    
    MatrixXd Q = MatrixXd(4, 4);
    Q << dt_4/4 * aXNoise, 0, dt_3 / 2 * aXNoise, 0,
        0, dt_4 / 4 * aYNoise, 0, dt_3 / 2 * aYNoise,
        dt_3 / 2 * aXNoise, 0, dt_2 * aXNoise, 0,
        0, dt_3 / 2 * aYNoise, 0, dt_2 * aYNoise;
    
    return Q;
}

void ExtendedKalmanFilter::Update(shared_ptr<MeasurementModel> model,
                                  Measurement z)
{    
    VectorXd y = model->GetZSpacePointsDiff(z.Value, model->CVSpaceToZSpace(state));
    MatrixXd H = model->GetHMatrix(state);
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * covariance * Ht + model->GetNoiseMatrix();
    MatrixXd Si = S.inverse();
    MatrixXd K =  covariance * Ht * Si;
    
    state = state + (K * y);
    
    long x_size = state.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    covariance = (I - K * H) * covariance;
}

void ExtendedKalmanFilter::UpdateFMatrix(float dt)
{
    F(0, 2) = dt;
    F(1, 3) = dt;
}

void ExtendedKalmanFilter::Predict(long long timestamp)
{
    float dt = GetTimeDelta(timestamp);
    
    UpdateFMatrix(dt);
    MatrixXd Q = CalculateMotionNoise(dt);
    
    state = F * state;
    MatrixXd Ft = F.transpose();
    covariance = F * covariance * Ft + Q;
}

VectorXd ExtendedKalmanFilter::GetStateAsCVSpacePoint()
{
    return state;
}
