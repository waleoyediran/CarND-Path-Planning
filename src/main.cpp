#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using namespace std;

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    const double TIME_INC = 0.02; // Time iteration increment
    const double SAFE_DISTANCE = 30.0;
    const double LANE_WIDTH = 4.0;
    const int LANE_COUNT = 3;

    const int LEFT_LANE = 0;
    const int MID_LANE = 1;
    const int RIGHT_LANE = 2;

    int lane = MID_LANE; // Car starts from middle lane
    double velocity_increment = 0.224;
    double max_velocity = 22;
    double car_velocity = 0.0;



    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    h.onMessage([&car_velocity, &max_velocity, &velocity_increment, &LEFT_LANE, &MID_LANE, &RIGHT_LANE, &TIME_INC, &SAFE_DISTANCE, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
                        &map_waypoints_dx,&map_waypoints_dy]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    int curr_path_size = previous_path_x.size();

                    if (curr_path_size > 0) {
                        car_s = end_path_s;
                    }

                    bool car_in_front = false;
                    bool car_left = false;
                    bool car_right = false;



                    // Analyse vehicles in proximity of our car
                    // TODO: In the future, we can analyse the trajectory of cars and predict what action they are taking
                    for (const auto &fusion_data : sensor_fusion) {
                        double o_car_velocity_x = fusion_data[3];
                        double o_car_velocity_y = fusion_data[4];
                        double o_car_velocity = sqrt(o_car_velocity_x * o_car_velocity_x + o_car_velocity_y * o_car_velocity_y);
                        double o_car_s = fusion_data[5];
                        double o_car_d = fusion_data[6];

                        int o_car_lane;

                        if (o_car_d <= 4) {
                            o_car_lane = LEFT_LANE;
                        } else if (o_car_d > 4 && o_car_d < 8) {
                            o_car_lane = MID_LANE;
                        } else if (o_car_d >= 8) {
                            o_car_lane = RIGHT_LANE;
                        }

                        // anticipate where the other car will be in the next update cycle
                        o_car_s += TIME_INC * o_car_velocity;

                        double dist_btw_cars = o_car_s - car_s;
                        if (dist_btw_cars > (-SAFE_DISTANCE + 10) && dist_btw_cars <  SAFE_DISTANCE) {
                            if (lane == o_car_lane && dist_btw_cars > 0) {
                                car_in_front = true;
                            }
                            if ((lane - o_car_lane) == 1) {
                                car_left = true;
                            }
                            if ((lane - o_car_lane) == -1) {
                                car_right = true;
                            }
                        }
                    }

                    if (car_in_front) {
                        if (lane > LEFT_LANE && !car_left) {
                            lane--;
                            cout << "Change lane left" << endl;
                        } else if (lane < RIGHT_LANE && !car_right) {
                            lane++;
                            cout << "Change lane right" << endl;
                        } else {
                            cout << "Reduce speed " << car_velocity << endl;
                        }
                        car_velocity -= velocity_increment;
                    } else if (car_velocity < max_velocity){
                        car_velocity += velocity_increment;
                        cout << "Increase speed " << car_velocity << endl;
                    }

                    // Points on new car path
                    vector<double> car_points_x, car_points_y;
                    double ref_yaw = deg2rad(car_yaw);
                    double ref_x = car_x;
                    double ref_y = car_y;

                    // Add path starting points
                    if (curr_path_size > 1) {
                        ref_x = previous_path_x[curr_path_size - 1];
                        ref_y = previous_path_y[curr_path_size - 1];

                        double prev_ref_x = previous_path_x[curr_path_size - 2];
                        double prev_ref_y = previous_path_y[curr_path_size - 2];

                        ref_yaw = atan2(ref_y -  prev_ref_y, ref_x - prev_ref_x);

                        car_points_x.push_back(previous_path_x[curr_path_size - 2]);
                        car_points_x.push_back(previous_path_x[curr_path_size - 1]);

                        car_points_y.push_back(previous_path_y[curr_path_size - 2]);
                        car_points_y.push_back(previous_path_y[curr_path_size - 1]);
                    } else {
                        car_points_x.push_back(car_x);
                        car_points_y.push_back(car_y);
                    }

                    // Add points after 30, 60 and 90m respectively
                    vector<double> wp30 = getXY(car_s + 30, 2 + lane * 4, map_waypoints_s,map_waypoints_x, map_waypoints_y);

                    vector<double> wp60 = getXY(car_s + 60, 2 + lane * 4, map_waypoints_s,map_waypoints_x, map_waypoints_y);
                    vector<double> wp90 = getXY(car_s + 90, 2 + lane * 4, map_waypoints_s,map_waypoints_x, map_waypoints_y);

                    car_points_x.push_back(wp30[0]);
                    car_points_x.push_back(wp60[0]);
                    car_points_x.push_back(wp90[0]);

                    car_points_y.push_back(wp30[1]);
                    car_points_y.push_back(wp60[1]);
                    car_points_y.push_back(wp90[1]);


                    for (int i = 0; i < car_points_x.size(); i++) {
                        double x_diff = car_points_x[i] - ref_x;
                        double y_diff = car_points_y[i] - ref_y;

                        car_points_x[i] = x_diff * cos(-ref_yaw) - y_diff * sin(-ref_yaw);
                        car_points_y[i] = x_diff * sin(-ref_yaw) + y_diff * cos(-ref_yaw);
                    }

                    tk::spline sp;
                    sp.set_points(car_points_x, car_points_y);

                    double goal_x = 30.0;
                    double goal_y = sp(goal_x);
                    double goal_distance = distance(0, 0, goal_x, goal_y);

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    for (int i = 0; i < curr_path_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    double x_inc = goal_x / (goal_distance / (car_velocity * TIME_INC));
                    for (int i = 1; i <= 50 - curr_path_size; i++) {
                        double xp = x_inc * i;
                        double yp = sp(xp);
                        double point_x = ref_x + (xp * cos(ref_yaw) - yp * sin(ref_yaw));
                        double point_y = ref_y + (xp * sin(ref_yaw) + yp * cos(ref_yaw));

                        next_x_vals.push_back(point_x);
                        next_y_vals.push_back(point_y);
                    }

                    json msgJson;


                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */


                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
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