//
//  WebSocketMessageHandler.hpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#ifndef WebSocketMessageHandler_hpp
#define WebSocketMessageHandler_hpp

#include <stdio.h>
#include <string>

#include <uWS/uWS.h>
#include "json.hpp"

#include "RMSECalculator.hpp"
#include "KalmanFilter.hpp"
#include "MeasurementPackageFactory.hpp"

using std::string;

class WebSocketMessageHandler
{
public:
    WebSocketMessageHandler(shared_ptr<KalmanFilterK> kalmanFilter,
                            shared_ptr<RMSECalculator> rmseCalculator,
                            shared_ptr<MeasurementPackageFactory> measurementPackageFactory) :
        kalmanFilter(kalmanFilter), rmseCalculator(rmseCalculator),
        measurementPackageFactory(measurementPackageFactory)
    {}
    void HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws);
private:
    shared_ptr<KalmanFilterK> kalmanFilter;
    shared_ptr<RMSECalculator> rmseCalculator;
    shared_ptr<MeasurementPackageFactory> measurementPackageFactory;
    
    bool MessageHasExpectedPrefix(const string& message);
    string GetMessageContent(const string& message);
    string ProcessMessageContent(string& content);
};

#endif /* WebSocketMessageHandler_hpp */
