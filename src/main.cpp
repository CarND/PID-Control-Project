#include <math.h>
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <vector>

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * Initialize the pid variable.
   */

  // p values manually tested
  std::vector<double> p{0.001, 3.0, 0.01};
  std::vector<double> dp{1., 1., 1.};
  pid = PID();
  pid.Init(p, dp);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * Calculate steering value here, remember the steering value is
           *   [-1, 1].
           *   Use another PID controller to control the speed!
           */
          double tol = 0.0002;
          pid.UpdateError(cte);
          pid.TwiddleOptimizeCoef(cte, tol); 

          // throttle inversely proportional to total error (low error -> fast speed)
          double throttle = speed / pid.TotalError();
          if (throttle < 0) {
            throttle = 0.1;
          } 
          double turn = tan(pid.TotalError()); 
          double max_steer = 1.;

          if (turn > max_steer) {
            turn = max_steer;
          } else if (turn < -max_steer) {
            turn = -max_steer;
          }

          steer_value = turn + angle;
          if (steer_value > 2.0 * M_PI) {
            steer_value -= (2.0 * M_PI);
          } else if (steer_value < 2.0 * M_PI) {
            steer_value += 2.0 * M_PI;
          }

          throttle = 0.3;
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["Kp"] = pid.Kp;
          msgJson["Ki"] = pid.Ki;
          msgJson["Kd"] = pid.Kd;          
          msgJson["p_error"] = pid.p_error;
          msgJson["i_error"] = pid.i_error;
          msgJson["d_error"] = pid.d_error;          
          msgJson["TotalError"] = pid.TotalError();
          msgJson["angle"] = angle;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}