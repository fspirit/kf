//
//  UnscentedKalmanFilter.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 25/07/2017.
//
//

#include "UnscentedKalmanFilter.hpp"
#include <iostream>

#include "RadarMeasurementModel.hpp"

using std::cout;
using std::endl;

const int StateSize = 5;
const int AugmentedStateSize = 7;
const double Lambda = 3 - AugmentedStateSize;
const int SigmaPointsCount = 2 * AugmentedStateSize + 1;

void UnscentedKalmanFilter::Initialise(shared_ptr<MeasurementModel> model,
                                      Measurement measurement)
{
    covariance = MatrixXd::Identity(StateSize, StateSize);
    
    state = model->ZSpaceToCTRVSpace(measurement.Value);
    
    KalmanFilterBase::Initialise(model, measurement);
}

// Predict
//
void UnscentedKalmanFilter::Predict(long long timestamp)
{
    float dt = KalmanFilterBase::GetTimeDelta(timestamp);
    
    MatrixXd augmentedSigmaPoints = GenerateAugmentedSigmaPoints();
    
    MatrixXd predictedSigmaPoints = GetPredictedSigmaPoints(augmentedSigmaPoints, dt);
    
    PredictStateWithSigmaPoints(predictedSigmaPoints);
    
    sigmaPoints_ = predictedSigmaPoints;
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
    augmentedState.fill(0.0);
    augmentedState.head(StateSize) = state;
    
    return augmentedState;
}

MatrixXd UnscentedKalmanFilter::GetAugmentedCovariance()
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
    
    augmentedSigmaPoints.fill(0.0);
    augmentedSigmaPoints.col(0) = augmentedState;
    for (int i = 0; i < AugmentedStateSize; i++)
    {
        augmentedSigmaPoints.col(i + 1) =
            ProcessSpacePointsSum(augmentedState,
                                  sqrt(Lambda + AugmentedStateSize) * L.col(i));
        
        augmentedSigmaPoints.col(i + 1 + AugmentedStateSize) =
            ProcessSpacePointsSum(augmentedState,
                                  - sqrt(Lambda + AugmentedStateSize) * L.col(i));
    }
    return augmentedSigmaPoints;
}

MatrixXd UnscentedKalmanFilter::GetPredictedSigmaPoints(MatrixXd& augmentedSigmaPoints, float dt)
{
    MatrixXd predictedSigmaPoints = MatrixXd(StateSize, SigmaPointsCount);
    predictedSigmaPoints.fill(0.0);
    
    for (int i = 0; i < SigmaPointsCount; i++)
    {
        VectorXd augSigmaPoint = augmentedSigmaPoints.col(i);
        
        VectorXd noise = GetProcessModelNoise(dt, augSigmaPoint);
        VectorXd spMotion = GetProcessModelChange(dt, augSigmaPoint);
        
        VectorXd predictedSigmaPoint =
            ProcessSpacePointsSum(augSigmaPoint.head(StateSize), spMotion + noise);
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
    spMotion.fill(0.0);
    if (fabs(yaw_dot) < 0.00001)
        spMotion << v * cos(yaw) * dt,
            v * sin(yaw) * dt,
            0.0,
            0.0,
            0.0;
    else
        spMotion << v / yaw_dot * ( sin(yaw + yaw_dot * dt) - sin(yaw)),
            v / yaw_dot * (-cos(yaw + yaw_dot * dt) + cos(yaw)),
            0.0,
            yaw_dot * dt,
            0.0;
    
    return spMotion;
}

void UnscentedKalmanFilter::PredictStateWithSigmaPoints(MatrixXd& sigmaPoints)
{
    VectorXd weights = GetSigmaPointsWeights();
    
    state.fill(0.0);
    for (int i = 0; i < SigmaPointsCount; i++)
        state += weights(i) * sigmaPoints.col(i);
    
    covariance.fill(0.0);
    for (int i = 0; i < SigmaPointsCount; i++)
    {
        VectorXd residual = ProcessSpacePointsSum(sigmaPoints.col(i), -state);
        covariance += weights(i) * residual * residual.transpose();
    }
}

// Update
//
void UnscentedKalmanFilter::Update(shared_ptr<MeasurementModel> model,
                                   Measurement measurement)
{
    MatrixXd zSigmaPoints = ConvertPointsToZSpace(sigmaPoints_, model);
    
    VectorXd zSigmaPointsMean = CalculateSampleMean(zSigmaPoints, model->ZSpaceSize());
    
    MatrixXd S = CalculateSampleCovariance(zSigmaPoints, zSigmaPointsMean, model);
    
    MatrixXd Tc = CalculateCrossCorrelationMatrix(zSigmaPoints, zSigmaPointsMean, model);
    MatrixXd K = Tc * S.inverse();
    
    VectorXd zSpaceError = model->GetZSpacePointsDiff(measurement.Value, zSigmaPointsMean);
    
    state = ProcessSpacePointsSum(state, K * zSpaceError);
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
    
    MatrixXd Tc = MatrixXd(StateSize, model->ZSpaceSize());
    Tc.fill(0.0);
    
    for (int i = 0; i < SigmaPointsCount; i++)
    {
        VectorXd zSpaceDiff = model->GetZSpacePointsDiff(zSigmaPoints.col(i),
                                                         zSigmaPointsMean);
        
        VectorXd processSpaceDiff = ProcessSpacePointsSum(sigmaPoints_.col(i), -state);
        
        Tc = Tc + weights(i) * processSpaceDiff * zSpaceDiff.transpose();
    }
    
    return Tc;
}

VectorXd UnscentedKalmanFilter::ProcessSpacePointsSum(const VectorXd& pointA, const VectorXd& pointB)
{
    VectorXd diff = pointA + pointB;
    
    if (diff(3) > 2 * M_PI || diff(3) < -2 * M_PI)
        diff(3) = fmod(diff(3), 2 * M_PI);
    
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

VectorXd UnscentedKalmanFilter::GetStateAsCVSpacePoint()
{
    float x = state(0);
    float y = state(1);
    float v = state(2);
    float yaw = state(3);
    
    return Eigen::Vector4d(x, y, v * cos(yaw), v * sin(yaw));
}

// Tests

void UnscentedKalmanFilter::GenerateSigmaPoints()
{
    // Example state
    VectorXd x = VectorXd(StateSize);
    x <<   5.7441,
        1.3800,
        2.2049,
        0.5015,
        0.3528;
    
    state = x;
    
    // Example cov
    MatrixXd P = MatrixXd(StateSize, StateSize);
    P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
        -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
        0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
        -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
        -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
    
    covariance = P;
    
    MatrixXd augmentedSigmaPoints = GenerateAugmentedSigmaPoints();
    
    cout << augmentedSigmaPoints << endl;
    
}

void UnscentedKalmanFilter::MoveSigmaPoints()
{
    MatrixXd Xsig_aug = MatrixXd(AugmentedStateSize, SigmaPointsCount);
    Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
    1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
    0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
    0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;
    
    
    double dt = 0.1;
    
    cout << GetPredictedSigmaPoints(Xsig_aug, dt) << endl;
}

void UnscentedKalmanFilter::PredictMeanAndCovarianceTest()
{
    MatrixXd Xsig_pred = MatrixXd(StateSize, SigmaPointsCount);
    Xsig_pred <<
    5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
    1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
    2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
    0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
    0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
    
    state = VectorXd(StateSize);
    state.fill(0.0);
    
    covariance = MatrixXd(StateSize, StateSize);
    covariance.fill(0.0);
    
    PredictStateWithSigmaPoints(Xsig_pred);
    
    cout << state << endl;
    cout << covariance << endl;
}

void UnscentedKalmanFilter::PredictRadarMeasurementTest()
{
    //radar measurement noise standard deviation radius in m
    double std_radr = 0.3;
    
    //radar measurement noise standard deviation angle in rad
    double std_radphi = 0.0175;
    
    //radar measurement noise standard deviation radius change in m/s
    double std_radrd = 0.1;
    
    //create example matrix with predicted sigma points
    MatrixXd Xsig_pred = MatrixXd(StateSize, SigmaPointsCount);
    Xsig_pred <<
    5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
    1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
    2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
    0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
    0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
    
    sigmaPoints_ = Xsig_pred;
    
    auto model  = std::make_shared<RadarMeasurementModel>(std_radr * std_radr,
                                                          std_radphi * std_radphi,
                                                          std_radrd * std_radrd);
    
    MatrixXd zSigmaPoints = ConvertPointsToZSpace(sigmaPoints_, model);
    VectorXd zSigmaPointsMean = CalculateSampleMean(zSigmaPoints, model->ZSpaceSize());
    
    MatrixXd S = CalculateSampleCovariance(zSigmaPoints, zSigmaPointsMean, model);
    
    cout << zSigmaPointsMean << endl;
    cout << S << endl;
}

void UnscentedKalmanFilter::UpdateStateTest()
{
    //create example matrix with predicted sigma points
    MatrixXd Xsig_pred = MatrixXd(StateSize, SigmaPointsCount);
    Xsig_pred <<
    5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
    1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
    2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
    0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
    0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
    
    sigmaPoints_ = Xsig_pred;
    
    //create example vector for predicted state mean
    VectorXd x = VectorXd(StateSize);
    x <<
    5.93637,
    1.49035,
    2.20528,
    0.536853,
    0.353577;
    
    state = x;
    
    //create example matrix for predicted state covariance
    MatrixXd P = MatrixXd(StateSize,StateSize);
    P <<
    0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
    -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
    0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
    -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
    -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;
    
    covariance = P;
    
    //create example matrix with sigma points in measurement space
    MatrixXd Zsig = MatrixXd(3, SigmaPointsCount);
    Zsig <<
    6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
    0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
    2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;
    
    //create example vector for mean predicted measurement
    VectorXd z_pred = VectorXd(3);
    z_pred <<
    6.12155,
    0.245993,
    2.10313;
    
    //create example matrix for predicted measurement covariance
    MatrixXd S = MatrixXd(3,3);
    S <<
    0.0946171, -0.000139448,   0.00407016,
    -0.000139448,  0.000617548, -0.000770652,
    0.00407016, -0.000770652,    0.0180917;
    
    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(3);
    z <<
    5.9214,
    0.2187,
    2.0062;
    
    //radar measurement noise standard deviation radius in m
    double std_radr = 0.3;
    
    //radar measurement noise standard deviation angle in rad
    double std_radphi = 0.0175;
    
    //radar measurement noise standard deviation radius change in m/s
    double std_radrd = 0.1;
    
    auto model  = std::make_shared<RadarMeasurementModel>(std_radr * std_radr,
                                                          std_radphi * std_radphi,
                                                          std_radrd * std_radrd);
    
                             
    MatrixXd Tc = CalculateCrossCorrelationMatrix(Zsig, z_pred, model);
    MatrixXd K = Tc * S.inverse();
    
    VectorXd zSpaceError = model->GetZSpacePointsDiff(z, z_pred);
    state += K * zSpaceError;
    covariance -= K * S * K.transpose();
    
    cout << state << endl;
    cout << covariance << endl;
}







