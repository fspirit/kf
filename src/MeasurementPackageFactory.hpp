//
//  MeasurementPackageFactory.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#ifndef MeasurementPackageFactory_hpp
#define MeasurementPackageFactory_hpp

#include <stdio.h>
#include <string>
#include <iostream>

#include "MeasurementPackage.hpp"
#include "RadarMeasurement.hpp"
#include "LaserMeasurement.hpp"

using std::istringstream;
using std::shared_ptr;

class MeasurementPackageFactory
{
public:
    shared_ptr<MeasurementPackage> CreatePackageFromStream(istringstream& stringStream);

private:
    shared_ptr<LaserMeasurement> CreateLaserMeasurementPackage(istringstream& stringStream);
    shared_ptr<RadarMeasurement> CreateRadarMeasurementPackage(istringstream& stringStream);
};


#endif /* MeasurementPackageFactory_hpp */
