//
//  RadarMeasurementModel.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 26/07/2017.
//
//

#ifndef RadarMeasurementModel_hpp
#define RadarMeasurementModel_hpp

#include "MeasurementModel.hpp"

class RadarMeasurementModel: public MeasurementModel
{
public:
    RadarMeasurementModel(double roNoiseVar, double phiNoiseVar, double phiDotNoiseVar);
    virtual VectorXd GetZSpacePointsDiff(const VectorXd& pointA, const VectorXd& pointB);
    virtual MatrixXd GetHMatrix(const VectorXd& cvSpacePoint);
    virtual MatrixXd GetNoiseMatrix();
    virtual VectorXd ZSpaceToCVSpace(const VectorXd& zSpacePoint);
    virtual VectorXd ZSpaceToCTRVSpace(const VectorXd& zSpacePoint);
    virtual VectorXd CTRVSpaceToZSpace(const VectorXd& ctrvSpacePoint);
    virtual VectorXd CVSpaceToZSpace(const VectorXd& cvSpacePoint);
private:
    MatrixXd R;
};

#endif /* RadarMeasurementModel_hpp */
