//
//  MeasurementPackageFactory.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#include "MeasurementPackageFactory.hpp"

MeasurementPackage MeasurementPackageFactory::CreatePackageFromStream(istringstream& stringStream)
{
    std::string sensor_type;
    stringStream >> sensor_type;
    
    if (sensor_type.compare("L") == 0)
        return CreateLaserMeasurementPackage(stringStream);
    else
        return CreateRadarMeasurementPackage(stringStream);
}

MeasurementPackage MeasurementPackageFactory::CreateLaserMeasurementPackage(istringstream& stringStream)
{
    float px, py;
    long long timestamp;
    stringStream >> px >> py >> timestamp;
    
    return MeasurementPackage(laserMeasurementModel,
                              Measurement(Eigen::Vector2d(px, py), timestamp));
}

MeasurementPackage MeasurementPackageFactory::CreateRadarMeasurementPackage(istringstream& stringStream)
{
    float ro, phi, ro_dot;
    long long timestamp;
    stringStream >> ro >> phi >> ro_dot >> timestamp;
    
    return MeasurementPackage(radarMeasurementModel,
                              Measurement(Eigen::Vector3d(ro, phi, ro_dot), timestamp));
}
