//
//  KalmanFilter.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 18/07/2017.
//
//

#include "KalmanFilter.hpp"

void KalmanFilterK::Initialise(shared_ptr<MeasurementPackage> measurement)
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
    
    state = measurement->GetCartesianCoords();
    previousTimestamp = measurement->GetTimestamp();
    
    isInitialised = true;
}

void KalmanFilterK::ProcessMeasurement(shared_ptr<MeasurementPackage> measurement)
{
    if (!isInitialised)
        Initialise(measurement);
    else
    {
        Predict(measurement->GetTimestamp());
        Update(measurement);
    }            
}

float KalmanFilterK::GetTimeDelta(long long timestamp)
{
    float dt = (timestamp - previousTimestamp) / 1000000.0;
    previousTimestamp = timestamp;
    
    return dt;
}

MatrixXd KalmanFilterK::CalculateMotionNoise(float dt)
{
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    
    const int noise_ax(9);
    const int noise_ay(9);
    
    MatrixXd Q = MatrixXd(4, 4);
    Q << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
        0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
        dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
        0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    
    return Q;
}

void KalmanFilterK::Update(shared_ptr<MeasurementPackage> measurement)
{
    VectorXd y = measurement->GetDiff(state);
    MatrixXd H = measurement->GetHMatrix(state);
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * covariance * Ht + measurement->GetNoiseMatrix();
    MatrixXd Si = S.inverse();
    MatrixXd K =  covariance * Ht * Si;
    
    state = state + (K * y);
    
    long x_size = state.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    covariance = (I - K * H) * covariance;
}

void KalmanFilterK::UpdateFMatrix(float dt)
{
    F(0, 2) = dt;
    F(1, 3) = dt;
}

void KalmanFilterK::Predict(long long timestamp)
{
    float dt = GetTimeDelta(timestamp);
    
    UpdateFMatrix(dt);
    MatrixXd Q = CalculateMotionNoise(dt);
    
    state = F * state;
    MatrixXd Ft = F.transpose();
    covariance = F * covariance * Ft + Q;
}
