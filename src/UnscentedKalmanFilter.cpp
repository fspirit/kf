//
//  UnscentedKalmanFilter.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 25/07/2017.
//
//

#include "UnscentedKalmanFilter.hpp"

const int StateSize = 5;
const int AugmentedStateSize = 7;
const double Lambda = 3 - AugmentedStateSize;
const int SigmaPointsCount = 2 * AugmentedStateSize + 1;

// Predict
//
void UnscentedKalmanFilter::Predict(long long timestamp)
{
    float dt = KalmanFilterBase::GetTimeDelta(timestamp);
    
    MatrixXd augmentedSigmaPoints = GenerateAugmentedSigmaPoints();
    
    MatrixXd predictedSigmaPoints = GetPredictedSigmaPoints(augmentedSigmaPoints, dt);
    
    PredictStateWithSigmaPoints(predictedSigmaPoints);
    
    sigmaPoints = predictedSigmaPoints;
}

MatrixXd UnscentedKalmanFilter::GenerateAugmentedSigmaPoints()
{
    VectorXd augmentedState = GetAugmentedState();
    MatrixXd augmentedCovariance = GetAugmentedCovariance();
    
    return SampleAugmentedSigmaPoints(augmentedState, augmentedCovariance);
}

VectorXd UnscentedKalmanFilter::GetAugmentedState()
{
    VectorXd augmentedState = VectorXd(AugmentedStateSize);
    augmentedState.head(5) = state;
    augmentedState.tail(2) = VectorXd::Zero(2);
    
    return augmentedState;
}

VectorXd UnscentedKalmanFilter::GetAugmentedCovariance()
{
    MatrixXd augmentedCovariance = MatrixXd(AugmentedStateSize, AugmentedStateSize);
    augmentedCovariance.fill(0.0);
    augmentedCovariance.topLeftCorner(StateSize, StateSize) = covariance;
    augmentedCovariance(5,5) = aNoiseVar;
    augmentedCovariance(6,6) = yawDDNoiseVar;
    
    return augmentedCovariance;
}

MatrixXd UnscentedKalmanFilter::SampleAugmentedSigmaPoints(VectorXd& augmentedState,
                                                           MatrixXd& augmentedCovariance)
{
    MatrixXd L = augmentedCovariance.llt().matrixL();
    
    MatrixXd augmentedSigmaPoints = MatrixXd(AugmentedStateSize, SigmaPointsCount);
    augmentedSigmaPoints.col(0) = augmentedState;
    for (int i = 0; i < AugmentedStateSize; i++)
    {
        augmentedSigmaPoints.col(i + 1) =
            augmentedState + sqrt(Lambda + AugmentedStateSize) * L.col(i);
        augmentedSigmaPoints.col(i + 1 + AugmentedStateSize) =
            augmentedState - sqrt(Lambda + AugmentedStateSize) * L.col(i);
    }
    return augmentedSigmaPoints;
}

MatrixXd UnscentedKalmanFilter::GetPredictedSigmaPoints(MatrixXd& augmentedSigmaPoints, float dt)
{
    MatrixXd predictedSigmaPoints = MatrixXd(StateSize, SigmaPointsCount);
    
    for (int i = 0; i < SigmaPointsCount; i++)
    {
        VectorXd augSigmaPoint = augmentedSigmaPoints.col(i);
        
        VectorXd noise = GetProcessModelNoise(dt, augSigmaPoint);
        VectorXd spMotion = GetProcessModelChange(dt, augSigmaPoint);
        
        VectorXd predictedSigmaPoint = augSigmaPoint.head(StateSize) + spMotion + noise;
        predictedSigmaPoints.col(i) = predictedSigmaPoint;
    }
    
    return predictedSigmaPoints;
}

VectorXd UnscentedKalmanFilter::GetProcessModelNoise(float dt, VectorXd& augmentedSigmaPoint)
{
    float yaw = augmentedSigmaPoint(3);
    float a_noise = augmentedSigmaPoint(5);
    float yaw_dd_noise = augmentedSigmaPoint(6);
    
    VectorXd noise(StateSize);
    noise << 0.5 * dt * dt * cos(yaw) * a_noise,
        0.5 * dt * dt * sin(yaw) * a_noise,
        dt * a_noise,
        0.5 * dt * dt * yaw_dd_noise,
        dt * yaw_dd_noise;
    
    return noise;
}

VectorXd UnscentedKalmanFilter::GetProcessModelChange(float dt, VectorXd& augmentedSigmaPoint)
{
    float v = augmentedSigmaPoint(2);
    float yaw = augmentedSigmaPoint(3);
    float yaw_dot = augmentedSigmaPoint(4);
    
    VectorXd spMotion(StateSize);
    if (fabs(yaw_dot) < 0.00001)
        spMotion << v * cos(yaw) * dt, v * sin(yaw) * dt,
        0, yaw_dot * dt, 0;
    else
        spMotion << v / yaw_dot * ( sin(yaw + yaw_dot*dt) - sin(yaw)),
        v / yaw_dot * (-cos(yaw + yaw_dot*dt) + cos(yaw)),
        0, yaw_dot * dt, 0;
    
    return spMotion;
}

void UnscentedKalmanFilter::PredictStateWithSigmaPoints(MatrixXd& sigmaPoints)
{
    VectorXd weights = GetSigmaPointsWeights();
    
    for (int i = 0; i < SigmaPointsCount; i++)
        state += weights(i) * sigmaPoints.col(i);
    
    for (int i = 0; i < SigmaPointsCount; i++)
    {
        VectorXd residual = ProcessSpacePointsDiff(sigmaPoints.col(i), state);
        covariance += weights(i) * residual * residual.transpose();
    }
}

// Update
//
void UnscentedKalmanFilter::Update(shared_ptr<MeasurementModel> model,
                                   Measurement measurement)
{
    MatrixXd zSigmaPoints = ConvertPointsToZSpace(sigmaPoints, model);
    VectorXd zSigmaPointsMean = CalculateSampleMean(zSigmaPoints, model->ZSpaceSize());
    
    MatrixXd S = CalculateSampleCovariance(zSigmaPoints, zSigmaPointsMean, model);
    MatrixXd Tc = CalculateCrossCorrelationMatrix(zSigmaPoints, zSigmaPointsMean, model);
    MatrixXd K = Tc * S.inverse();
    
    VectorXd zSpaceError = model->GetZSpacePointsDiff(measurement.Value, zSigmaPointsMean);
    state += K * zSpaceError;
    covariance -= K * S * K.transpose();
}

VectorXd UnscentedKalmanFilter::CalculateSampleMean(MatrixXd& sigmaPoints,
                                                    int zSpaceSize)
{
    VectorXd weights = GetSigmaPointsWeights();
    
    VectorXd sigmaPointsMean = VectorXd(zSpaceSize);
    sigmaPointsMean.fill(0.0);
    for (int i = 0; i < SigmaPointsCount; i++)
        sigmaPointsMean += weights(i) * sigmaPoints.col(i);
    
    return sigmaPointsMean;
}

VectorXd UnscentedKalmanFilter::GetSigmaPointsWeights()
{
    VectorXd weights = VectorXd(SigmaPointsCount);
    weights(0) = Lambda / (Lambda + AugmentedStateSize);
    for (int i = 1; i < SigmaPointsCount; i++)
        weights(i) = 0.5 / (AugmentedStateSize + Lambda);
    return  weights;
}

MatrixXd UnscentedKalmanFilter::CalculateSampleCovariance(MatrixXd& sigmaPoints,
                                                          VectorXd& mean,
                                                          shared_ptr<MeasurementModel> model)
{
    VectorXd weights = GetSigmaPointsWeights();
    
    MatrixXd S = MatrixXd(model->ZSpaceSize(), model->ZSpaceSize());
    S.fill(0.0);
    for (int i = 0; i < SigmaPointsCount; i++)
    {
        VectorXd residual = model->GetZSpacePointsDiff(sigmaPoints.col(i), mean);
        S += weights(i) * residual * residual.transpose();
    }
    
    S += model->GetNoiseMatrix();
    
    return S;
}

MatrixXd UnscentedKalmanFilter::CalculateCrossCorrelationMatrix(MatrixXd& zSigmaPoints,
                                                                VectorXd& zSigmaPointsMean,
                                                                shared_ptr<MeasurementModel> model)
{
    VectorXd weights = GetSigmaPointsWeights();
    
    MatrixXd Tc = MatrixXd(model->ZSpaceSize(), model->ZSpaceSize());
    Tc.fill(0.0);
    
    for (int i = 0; i < SigmaPointsCount; i++)
    {
        VectorXd zSpaceDiff = model->GetZSpacePointsDiff(zSigmaPoints.col(i),
                                                         zSigmaPointsMean);
        
        VectorXd processSpaceDiff = ProcessSpacePointsDiff(sigmaPoints.col(i), state);
        
        Tc = Tc + weights(i) * processSpaceDiff * zSpaceDiff.transpose();
    }
    
    return Tc;
}

VectorXd UnscentedKalmanFilter::ProcessSpacePointsDiff(const VectorXd& pointA, VectorXd& pointB)
{
    VectorXd diff = pointA - pointB;
    
    while (diff(3)> M_PI) diff(3)-=2.*M_PI;
    while (diff(3)<-M_PI) diff(3)+=2.*M_PI;
    
    return diff;
}

MatrixXd UnscentedKalmanFilter::ConvertPointsToZSpace(MatrixXd& sigmaPoints,
                                                      shared_ptr<MeasurementModel> model)
{
    MatrixXd zSigmaPoints = MatrixXd(model->ZSpaceSize(), SigmaPointsCount);
    for (int i = 0; i < SigmaPointsCount; i++)
    {
        VectorXd ctrvSpacePoint = sigmaPoints.col(i);
        zSigmaPoints.col(i) = model->CTRVSpaceToZSpace(ctrvSpacePoint);
    }
    return zSigmaPoints;
}





