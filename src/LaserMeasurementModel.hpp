//
//  LaserMeasurementModel.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#ifndef LaserMeasurementModel_hpp
#define LaserMeasurementModel_hpp

#include "MeasurementModel.hpp"

class LaserMeasurementModel: public MeasurementModel
{
public:
    LaserMeasurementModel(double xNoise, double yNoise);
    virtual VectorXd GetZSpacePointsDiff(const VectorXd& pointA, const VectorXd& pointB);
    virtual MatrixXd GetHMatrix(const VectorXd& state);
    virtual MatrixXd GetNoiseMatrix();
    virtual VectorXd ZSpaceToСVSpace(const VectorXd& zSpacePoint);
    virtual VectorXd ZSpaceToСTRVSpace(const VectorXd& zSpacePoint);    
    virtual VectorXd CTRVSpaceToZSpace(const VectorXd& ctrvSpacePoint);
    virtual VectorXd CVSpaceToZSpace(const VectorXd& cvSpacePoint);
private:
    MatrixXd R;
};

#endif /* LaserMeasurementModel_hpp */
