#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>

#include "RMSECalculator.hpp"
#include "ExtendedKalmanFilter.hpp"
#include "UnscentedKalmanFilter.hpp"
#include "MeasurementPackageFactory.hpp"
#include "WebSocketMessageHandler.hpp"
#include "RadarMeasurementModel.hpp"
#include "LaserMeasurementModel.hpp"

using std::string;
using std::cout;
using std::endl;
using std::cerr;

const double RadarRoNoiseVar = 0.9;
const double RadarPhiNoiseVar = 0.009;
const double RadarRoDotNoiseVar = 0.9;

const double laserXNoiseVar = 0.0225;
const double laserYNoiseVar = 0.0225;

const double proccessANoiseVar = 0.04;
const double processYawDDNoiseVar = 0.04;

const double processAXNoiseVar = 9;
const double processAYNoiseVar = 9;

const int ExtendedKalmanFilterMode = 0;
const int UnscentedKalmanFilterMode = 1;

static void printUsage(std::string name)
{
    cerr << "Usage: " << name << " <mode>" << endl
        << "Modes:" << endl
        << "\t 0\t Use extended kalman filter" << endl
        << "\t 1\t Use unscented kalman filter" << endl;
}

int main(int argc, char * argv[])
{
    
    if (argc < 2)
    {
        printUsage(argv[0]);
        return 1;
    }
    
    int mode = std::stoi(std::string(argv[1]));
    
    if (mode != ExtendedKalmanFilterMode && mode != UnscentedKalmanFilterMode)
    {
        printUsage(argv[0]);
        return 1;
    }
    
    uWS::Hub h;
    
    shared_ptr<KalmanFilterBase> kalmanFilter;
    if (mode == ExtendedKalmanFilterMode)
        kalmanFilter = std::make_shared<ExtendedKalmanFilter>(processAXNoiseVar,
                                                              processAYNoiseVar);
    else
        kalmanFilter = std::make_shared<UnscentedKalmanFilter>(proccessANoiseVar,
                                                              processYawDDNoiseVar);
        
    auto rmseCalculator = std::make_shared<RMSECalculator>();
    
    auto radarMeasurementModel = std::make_shared<RadarMeasurementModel>(RadarRoNoiseVar,
                                                                         RadarPhiNoiseVar,
                                                                         RadarRoDotNoiseVar);
    auto laserMeasurementModel = std::make_shared<LaserMeasurementModel>(laserXNoiseVar,
                                                                         laserYNoiseVar);
    
    auto measurementPackageFactory = std::make_shared<MeasurementPackageFactory>(radarMeasurementModel,
                                                                                 laserMeasurementModel);
    
    WebSocketMessageHandler handler(kalmanFilter, rmseCalculator, measurementPackageFactory);

    h.onMessage([&handler](uWS::WebSocket<uWS::SERVER> ws,
                                           char * data,
                                           size_t length,
                                           uWS::OpCode opCode)
    {
        if (length == 0)
            return;
        
        string message (data, length);
        handler.HandleMessage(message, ws);
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        cout << "Connected" << endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
        ws.close();
        cout << "Disconnected" << endl;
    });

    const int port = 4567;
    if (h.listen(port))
        cout << "Listening to port " << port << endl;
    else
    {
        cerr << "Failed to listen to port" << endl;
        return -1;
    }
    
    h.run();
}
























































































