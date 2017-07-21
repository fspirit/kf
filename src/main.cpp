#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>

#include "RMSECalculator.hpp"
#include "KalmanFilter.hpp"
#include "MeasurementPackageFactory.hpp"
#include "WebSocketMessageHandler.hpp"

using std::string;
using std::cout;
using std::endl;
using std::cerr;

int main()
{
    uWS::Hub h;
    
    auto kalmanFilter = std::make_shared<KalmanFilterK>();
    auto rmseCalculator = std::make_shared<RMSECalculator>();
    auto measurementPackageFactory = std::make_shared<MeasurementPackageFactory>();
    
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























































































