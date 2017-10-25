#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "PID.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Counters
static int time_steps;

// Constants
const double MIN_SPEED = 30.0;
const double MAX_THROTTLE = 0.7;

/**
 * Helper method to restart the simulator
 */
void Restart(uWS::WebSocket<uWS::SERVER> ws){
  string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned
 */
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
  
  //****************************************************************************
  // Initialize the pid variables
  //****************************************************************************
  PID pid_steering;
  PID pid_throttle;
  
  // Coefficients   Kp    Ki     kd
  pid_steering.Init(0.21, 0.007, 0.7);
  pid_throttle.Init(0.07, 0.001, 0.01);

  h.onMessage([&pid_steering, &pid_throttle]
              (uWS::WebSocket<uWS::SERVER> ws, char *data,size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = stod(j[1]["cte"].get<string>());
          double speed = stod(j[1]["speed"].get<string>());
          double angle = stod(j[1]["steering_angle"].get<string>());
          
          //********************************************************************
          // Calculate controls using PID
          //********************************************************************
          
          double steer_value;
          double throttle_value;
          
          // Steering control signal
          pid_steering.UpdateError(cte);
          steer_value = pid_steering.TotalError();
          
          // Trottle control signal
          if (speed >= MIN_SPEED) {
            // For speeds in excess of 30mph, brake on large steering angles
            pid_throttle.UpdateError(fabs(steer_value));
            throttle_value = MAX_THROTTLE + pid_throttle.TotalError();
          } else {
            // Do not let speed drop below 30mph
            throttle_value = MIN_SPEED + 5.0;
          }
          
          //********************************************************************
          
          // DEBUG
          cout << "[" << time_steps++ << "] CTE: " << cte
               << " Steering Value: " << steer_value << endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          cout << msg << endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          cout << "-> Speed = " << speed << " Angle = " << angle << endl;
        } // End if - telemetry
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      } // End if/else - s has data
    } // End if - websocket
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // I guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << endl;
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  
  h.run();
}
