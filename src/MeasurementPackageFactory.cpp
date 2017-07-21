//
//  MeasurementPackageFactory.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#include "MeasurementPackageFactory.hpp"

shared_ptr<MeasurementPackage> MeasurementPackageFactory::CreatePackageFromStream(istringstream& stringStream)
{
    std::string sensor_type;
    stringStream >> sensor_type;
    
    if (sensor_type.compare("L") == 0)
        return CreateLaserMeasurementPackage(stringStream);
    else
        return CreateRadarMeasurementPackage(stringStream);
}

shared_ptr<LaserMeasurement> MeasurementPackageFactory::CreateLaserMeasurementPackage(istringstream& stringStream)
{
    float px, py;
    long long timestamp;
    stringStream >> px >> py >> timestamp;
    
    return shared_ptr<LaserMeasurement>(new LaserMeasurement(px, py, timestamp));
}

shared_ptr<RadarMeasurement> MeasurementPackageFactory::CreateRadarMeasurementPackage(istringstream& stringStream)
{
    float ro, phi, ro_dot;
    long long timestamp;
    stringStream >> ro >> phi >> ro_dot >> timestamp;
    
    return shared_ptr<RadarMeasurement>(new RadarMeasurement(ro, phi, ro_dot, timestamp));
}
