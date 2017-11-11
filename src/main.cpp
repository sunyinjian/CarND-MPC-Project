#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double ratio_steer2delta = deg2rad(25.0); //convert steer [-1, 1] into delta angle (unit: radian);
const double ratio_throttle2acceleration = 10.0;//convert throttle [-1, 1] into acceleration (unit: m/s^2);
const double Lf = 2.67;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/**@brief transform way points from map coordinate into vehicle coordinate;
 *
 * transform way points from map coordinate into vehicle coordinate;
 * @param ptsx_map [IN]: the global x positions of waypoints in map coordinate;
 * @param ptsy_map [IN]: the global y positions of waypoints in map coordinate;
 * @param px [IN]: the global x position of vehicle in map coordinate;
 * @param py [IN]: the global y position of vehicle in map coordinate;
 * @param psi [IN]: the orientation of vehicle in map coordinate;
 * @param ptsx_vehicle [OUT]: the transformed x positions of waypoints in vehicle coordinate;
 * @param ptsy_vehicle [OUT]: the transformed y positions of waypoints in vehicle coordinate;
 */
void TransformWayPoints(const std::vector<double> ptsx_map, const std::vector<double> ptsy_map, const double px, const double py,const double psi, std::vector<double>& ptsx_vehicle, std::vector<double>& ptsy_vehicle){
    ptsx_vehicle.reserve(ptsx_map.size());
    ptsy_vehicle.reserve(ptsx_map.size());
    double cos_psi = cos(-psi);
    double sin_psi = sin(-psi);
    for (unsigned int i = 0; i < ptsx_map.size(); i++){
        ptsx_vehicle.push_back((ptsx_map[i] - px) * cos_psi - (ptsy_map[i] - py) * sin_psi);
        ptsy_vehicle.push_back((ptsx_map[i] - px) * sin_psi + (ptsy_map[i] - py) * cos_psi);
    }
}

/**@brief predict the state forward at the latency time before feeding into the solver;
 *
 * predict the state forward at the latency time before feeding into the solver;
 * @param state [IN]: the state at current time;
 * @param coef [IN]: the coefficient of 3rd polynomials;
 * @param delta_angle [IN]: the steer angle, unit: radian;
 * @param acceleration [IN]: the acceleration value, unit: m/s^2;
 * @return the predicted state which is predicted forward at the latency time;
 * @note that apply the negative sign to the delta angle;
 */
Eigen::VectorXd PredictState(const Eigen::VectorXd& state, const Eigen::VectorXd& coef, const double delta_angle, const double acceleration){
    const double dt = 0.1; //the latency timestamp; unit:s;
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];
    double delta = -delta_angle; //unit:radian
    double a = acceleration; //unit:m/s^2
    double f0 = polyeval(coef, x);
    double psides0 = atan(coef[1]+2*coef[2]*x+3*coef[3]*x*x);

    Eigen::VectorXd pred_state(6);
    pred_state[0] = x + v*cos(psi)*dt; //predicted x
    pred_state[1] = y + v*sin(psi)*dt; //predicted y
    pred_state[2] = psi + v*delta*dt/Lf;//predicted psi
    pred_state[3] = v+a*dt;//predicted v
    pred_state[4] = (f0-y)+(v*sin(epsi)*dt);//predicted cte
    pred_state[5] = (psi - psides0) + v*delta*dt/Lf;//predicted epsi
    return pred_state;
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= 0.447;//convert velocity unit from mph to m/s
          double delta = j[1]["steering_angle"];//unit: radian
          double acceleration = j[1]["throttle"];//unit: m/s^2

          //transform the waypoints from map coordinate into vehicle coordinate;
          vector<double> ptsx_vehicle;
          vector<double> ptsy_vehicle;
          TransformWayPoints(ptsx, ptsy, px, py, psi, ptsx_vehicle, ptsy_vehicle);

          //fit the waypoints by 3rd polynomials;
          Eigen::VectorXd coef = polyfit(Eigen::VectorXd::Map(ptsx_vehicle.data(), ptsx_vehicle.size()), Eigen::VectorXd::Map(ptsy_vehicle.data(), ptsy_vehicle.size()), 3);

          //calculate cte and epsi; note that px0, py0 and psi0 of the origin point in vehicle coordinate are 0.0;
          double cte = polyeval(coef, 0.0);
          double epsi = -atan(coef[1]); //epsi = px0 - atan(coef[1] + 2*coef[2]*px0 + 3*coef[3]*px0*px0);

          //predict the state forward at the latency time before feeding into the solver;
          Eigen::VectorXd state(6);
          state << 0.0, 0.0, 0.0, v, cte, epsi;
          Eigen::VectorXd pred_state = PredictState(state, coef, delta, acceleration);

          //feed into mpc solver;
          double steer_value;
          double throttle_value;
          std::vector<double> solution_values = mpc.Solve(pred_state, coef);
          steer_value = -solution_values[0]/ratio_steer2delta;//range: [-1, 1]
          throttle_value = solution_values[1]/ratio_throttle2acceleration;//range: [-1, 1]

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (unsigned int i = 2; i < solution_values.size(); i++){
              if (i%2 == 0){
                  mpc_x_vals.push_back(solution_values[i]);
              }
              else {
                  mpc_y_vals.push_back(solution_values[i]);
              }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          next_x_vals.reserve(ptsx_vehicle.size());
          next_y_vals.reserve(ptsy_vehicle.size());
          for (unsigned int i = 0; i < ptsx_vehicle.size(); i++){
              next_x_vals.push_back(ptsx_vehicle[i]);
              next_y_vals.push_back(ptsy_vehicle[i]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
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


