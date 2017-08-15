#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <stdexcept>
#include <string>

// for convenience
double EPSILON = 0.0000000000000000001;
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

bool AreSame(double a, double b) {
    return fabs(a - b) < EPSILON;
}

int main() {
    uWS::Hub h;

    // P controller: oscillates heavily, drives out of track quickly
//    PID pid = PID(0.2, 0, 0, false);

    // PD controller: better, but still oscillates - should still have bias, but it is able to complete a track
//    PID pid = PID(0.2, 0, 3, false);

    // use full PID controller - this time we activate twiddling so we can tune the parameters (Ki term is created
    // on demand)
    PID pid = PID(0.2, 0, 3, true);

    // output from Twiddle run
//    PID pid = PID(0.612625, 2.52657e-05, 0.652344, false);


    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    // TODO: maybe use these
                    //double angle = std::stod(j[1]["steering_angle"].get<std::string>());

                    pid.UpdateError(cte);
                    double steer_value =
                            -pid.Kp() * pid.p_error
                            - pid.Ki() * pid.i_error
                            - pid.Kd() * pid.d_error;

                    // DEBUG
                    cout << "CTE: " << cte
                         << " Steering Value: " << steer_value
                         << " Total error: " << pid.TotalError()
                         << endl;

                    cout << "params: Kp=" << pid.Kp()
                         << ", Ki=" << pid.Ki()
                         << ", Kd=" << pid.Kd()
                         << endl << endl;

                    // steering values must be in range [-1, 1]
                    if (steer_value < -1) {
                        steer_value = -1;
                    } else if (steer_value > 1) {
                        steer_value = 1;
                    }

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//                    std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
