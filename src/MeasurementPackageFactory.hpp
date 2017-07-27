//
//  MeasurementPackageFactory.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#ifndef MeasurementPackageFactory_hpp
#define MeasurementPackageFactory_hpp

#include <string>
#include <iostream>
#include <memory>

#include "MeasurementModel.hpp"
#include "RadarMeasurementModel.hpp"
#include "LaserMeasurementModel.hpp"
#include "Measurement.hpp"

using std::istringstream;
using std::shared_ptr;

struct MeasurementPackage
{
    MeasurementPackage(shared_ptr<MeasurementModel> model, Measurement value) :
        Model(model), Measurement(value) {}
    shared_ptr<MeasurementModel> Model;
    Measurement Measurement;
};

class MeasurementPackageFactory
{
public:
    MeasurementPackageFactory(shared_ptr<RadarMeasurementModel> radarMeasurementModel,
                              shared_ptr<LaserMeasurementModel> laserMeasurementModel) :
        radarMeasurementModel(radarMeasurementModel),
        laserMeasurementModel(laserMeasurementModel) {}
    MeasurementPackage CreatePackageFromStream(istringstream& stringStream);
private:
    MeasurementPackage CreateLaserMeasurementPackage(istringstream& stringStream);
    MeasurementPackage CreateRadarMeasurementPackage(istringstream& stringStream);
    
    shared_ptr<RadarMeasurementModel> radarMeasurementModel;
    shared_ptr<LaserMeasurementModel> laserMeasurementModel;    
};


#endif /* MeasurementPackageFactory_hpp */
