//
//  WebSocketMessageHandler.cpp
//  ExtendedKF
//
//  Created by Stanislav Olekhnovich on 19/07/2017.
//
//

#include "WebSocketMessageHandler.hpp"
#include "Eigen/Dense"

using json = nlohmann::json;
using Eigen::Vector4d;

VectorXd ReadGroundTruthValue(istringstream& stream)
{
    float x, y, vx, vy;
    stream >> x >> y >> vx >> vy;
    
    return Vector4d(x, y, vx, vy);
}

string CreateResponseMessage(float px, float py, VectorXd error)
{
    json msgJson;
    msgJson["estimate_x"] = px;
    msgJson["estimate_y"] = py;
    msgJson["rmse_x"] =  error(0);
    msgJson["rmse_y"] =  error(1);
    msgJson["rmse_vx"] = error(2);
    msgJson["rmse_vy"] = error(3);
    
    return "42[\"estimate_marker\"," + msgJson.dump() + "]";
}

string WebSocketMessageHandler::ProcessMessageContent(string& content)
{
    auto jsonContent = json::parse(content);
    string eventType = jsonContent[0].get<std::string>();
    
    string response;
    
    if (eventType == "telemetry")
    {
        string measurement = jsonContent[1]["sensor_measurement"];
        
        istringstream iss(measurement);
        
        auto mp = measurementPackageFactory->CreatePackageFromStream(iss);
        
        VectorXd gt = ReadGroundTruthValue(iss);
        rmseCalculator->AddGroundTruthValue(gt);
        
        kalmanFilter->ProcessMeasurement(mp);
                
        VectorXd state = kalmanFilter->GetState();
        rmseCalculator->AddEstimation(state);
        VectorXd rmse = rmseCalculator->Calculate();
        
        response = CreateResponseMessage(state(0), state(1), rmse);
    }
    return response;
}

string WebSocketMessageHandler::GetMessageContent(const std::string& message)
{
    string content;
    
    bool hasNullContent = (message.find("null") != std::string::npos);
    if (hasNullContent)
        return content;
    
    auto b1 = message.find_first_of("[");
    auto b2 = message.find_first_of("]");
    
    if (b1 != string::npos && b2 != string::npos)
        content = message.substr(b1, b2 - b1 + 1);
    
    return content;
}

bool WebSocketMessageHandler::MessageHasExpectedPrefix(const std::string& message)
{
    // "42" at the start of the message means there's a websocket message event.
    //
    const string prefix ("42");
    return (message.substr(0, prefix.size()) == prefix);
}

void WebSocketMessageHandler::HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws)
{
    if (MessageHasExpectedPrefix(message))
    {
        auto content = GetMessageContent(message);
        if (!content.empty())
        {
            auto response = ProcessMessageContent(content);
            
            if (!response.empty())
                ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
        }
        else
        {
            string response = "42[\"manual\",{}]";
            ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
        }
    }
}
