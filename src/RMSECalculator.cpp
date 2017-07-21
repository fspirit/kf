//
//  RMSECalculator.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 18/07/2017.
//
//

#include "RMSECalculator.hpp"

void RMSECalculator::AddGroundTruthValue(VectorXd groundTruthValue)
{
    groundTruth.push_back(groundTruthValue);
}

void RMSECalculator::AddEstimation(VectorXd estimation)
{
    estimations.push_back(estimation);
}

void RMSECalculator::SumResidualSquares(VectorXd& aggregate)
{
    for (int i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - groundTruth[i];
        
        residual = residual.array() * residual.array();
        aggregate += residual;
    }
}

bool RMSECalculator::IsStateInvalidForCalculation()
{
    return (estimations.size() != groundTruth.size() || estimations.size() == 0);
}

VectorXd RMSECalculator::Calculate()
{
    VectorXd error(4);
    error << 0, 0, 0, 0;
    
    if (IsStateInvalidForCalculation())
        return error;
    
    SumResidualSquares(error);
    
    error = error / estimations.size();
    
    return error.array().sqrt();
}
