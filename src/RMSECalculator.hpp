//
//  RMSECalculator.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 18/07/2017.
//
//

#ifndef RMSECalculator_hpp
#define RMSECalculator_hpp

#include <stdio.h>
#include <vector>

#include "Eigen/Dense"

using Eigen::VectorXd;

class RMSECalculator
{
public:
    void AddGroundTruthValue(VectorXd groundTruthValue);
    void AddEstimation(VectorXd estimation);
    VectorXd Calculate();
    
private:
    std::vector<VectorXd> groundTruth;
    std::vector<VectorXd> estimations;
    
    void SumResidualSquares(VectorXd& aggregate);
    bool IsStateInvalidForCalculation();
};

#endif /* RMSECalculator_hpp */
