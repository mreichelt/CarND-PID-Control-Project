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

// TODO: move poor man's test suite to fully-blown one (like Boost Test or Google C++ Test Lib)
int test() {
    PID pid;

    if (pid.twiddle_index != 0) throw runtime_error("1");
    if (pid.mode != twiddle_increase) throw runtime_error("2");

    pid.mode = twiddle_decrease;
    pid.moveToNextTwiddle();

    if (pid.twiddle_index != 1) throw runtime_error("1");
    if (pid.mode != twiddle_increase) throw runtime_error("2");

    pid = PID();
    if (!AreSame(1.0, pid.p_error)) throw runtime_error("should be 1");
    pid.getErrorRef(0) = 42.0;
    if (!AreSame(42.0, pid.p_error)) throw runtime_error("should be 42");

    pid = PID();
    if (!AreSame(0.0, pid.Kp)) throw runtime_error("should be 0");
    pid.getParamRef(0) = 2.0;
    if (!AreSame(2.0, pid.Kp)) throw runtime_error("should be 2");

    cout << "All OK" << endl;
}

int main() {
//    return test();

    uWS::Hub h;

    PID pid;
    // TODO: init with different values than zeros?

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
                    // TODO: maybe use these
                    //double speed = std::stod(j[1]["speed"].get<std::string>());
                    //double angle = std::stod(j[1]["steering_angle"].get<std::string>());

                    double steer_value = pid.GetValue(cte);
                    pid.UpdateError(cte);

                    // DEBUG
                    std::cout << "CTE: " << cte
                              << " Steering Value: " << steer_value
                              << " Total error: " << pid.TotalError()
                              << std::endl;

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
                    std::cout << msg << std::endl;
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
