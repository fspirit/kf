//
//  UnscentedKalmanFilter.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 25/07/2017.
//
//

#ifndef UnscentedKalmanFilter_hpp
#define UnscentedKalmanFilter_hpp

#include <memory>


#include "Eigen/Dense"
#include "MeasurementModel.hpp"
#include "Measurement.hpp"
#include "KalmanFilterBase.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::shared_ptr;

class UnscentedKalmanFilter : public KalmanFilterBase
{
public:
    UnscentedKalmanFilter(double aNoiseVar, double yawDDNoiseVar):
        aNoiseVar(aNoiseVar), yawDDNoiseVar(yawDDNoiseVar) {}
    virtual VectorXd GetStateAsCVSpacePoint();
    
    // Tests
    void GenerateSigmaPoints();
    void MoveSigmaPoints();
    void PredictMeanAndCovarianceTest();
    void PredictRadarMeasurementTest();
    void UpdateStateTest();

protected:
    virtual void Initialise(shared_ptr<MeasurementModel> model,
                            Measurement measurement);
    virtual void Predict(long long timestamp);
    virtual void Update(shared_ptr<MeasurementModel> model,
                        Measurement measurement);
    
private:
    MatrixXd sigmaPoints_;
    double aNoiseVar;
    double yawDDNoiseVar;
    
    MatrixXd GenerateAugmentedSigmaPoints();
    VectorXd GetAugmentedState();
    MatrixXd GetAugmentedCovariance();
    MatrixXd SampleAugmentedSigmaPoints(VectorXd& augmentedState,
                                        MatrixXd& augmentedCovariance);    
    MatrixXd GetPredictedSigmaPoints(MatrixXd& augmentedSigmaPoints, float dt);
    VectorXd GetProcessModelNoise(float dt, VectorXd& augmentedSigmaPoint);
    VectorXd GetProcessModelChange(float dt, VectorXd& augmentedSigmaPoint);
    void PredictStateWithSigmaPoints(MatrixXd& sigmaPoints);
    
    VectorXd GetSigmaPointsWeights();
    VectorXd CalculateSampleMean(MatrixXd& sigmaPoints, int zSpaceSize);
    MatrixXd ConvertPointsToZSpace(MatrixXd& sigmaPoints, shared_ptr<MeasurementModel> model);
    MatrixXd CalculateSampleCovariance(MatrixXd& sigmaPoints,
                                       VectorXd& mean,
                                       shared_ptr<MeasurementModel> model);
    
    MatrixXd CalculateCrossCorrelationMatrix(MatrixXd& zSigmaPoints,
                                             VectorXd& zSigmaPointsMean,
                                             shared_ptr<MeasurementModel> model);
    VectorXd ProcessSpacePointsSum(const VectorXd& pointA, const VectorXd& pointB);
    
};


#endif /* UnscentedKalmanFilter_hpp */
