#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  int lane = 1;   // 1 is middle lane
  int phase = 0;  // 0: running straight  1: changing lane
  int count = 500; // steps before moving to next status 

  
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &phase, &count]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
          * TODO: define a path made up of (x,y) points that the car will visit
          *   sequentially every .02 seconds
          */////////////////////////////////////////////////////////////////////////
      
          double max_s = 6945.554;
  		  int LW =4;  // lane width
 		  float dT = 0.02;
  		  double MAXVEL_MPH= 50.0;
  		  double MPS2MPH = 2.24; 
  		  double MAXACC = 10.0;
  		  double MAXJERK = 10.0; 
          
          
          int prev_size = previous_path_x.size(); 
          //std::cout << "prev size: " << prev_size << std::endl; 
          double min_gap = 30.0;
          
          // Initial goal params 
          double ref_vel = 49.5; // mph
          
          
          double goal_s = car_s + 60;
          double goal_d = (2+4*lane);      
          double goal_v = MAXVEL_MPH * 0.95 / MPS2MPH;
         
          
          // Get ohter cars data from sensor fusion 
          
          int id_front_samelane;
          double sgap_front_samelane = 9999.0; 
          double v_front_samelane; 
          
          int id_front_leftlane;
          double sgap_front_leftlane = 9999.0; 
          double v_front_leftlane; 
          
          int id_front_rightlane;
          double sgap_front_rightlane = 9999.0; 
          double v_front_rightlane; 
          
          int id_back_leftlane;
          double sgap_back_leftlane = 9999.0; 
          double v_back_leftlane; 
          
          int id_back_rightlane;
          double sgap_back_rightlane = 9999.0; 
          double v_back_rightlane; 
          
          // Obseve 
          for(int i=0; i<sensor_fusion.size(); i++){
          
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5]; 
            check_car_s += ((double)prev_size * dT * check_speed); 
            
            // observe same lane 
            
            if ( d < LW*(lane+1) && d > LW*lane){           
                // observe front
              	if (check_car_s > car_s && check_car_s - car_s < sgap_front_samelane ){               		
                   id_front_samelane = i; 
                   sgap_front_samelane = check_car_s - car_s;
                   v_front_samelane = check_speed; 
                } 
            } 
            // observe left lane 
            if ( d < LW*(lane) && d > LW*lane-1){
              // observe front
              	if (check_car_s > car_s && check_car_s - car_s < sgap_front_leftlane ){               		
                   id_front_leftlane = i; 
                   sgap_front_leftlane = check_car_s - car_s;
                   v_front_leftlane = check_speed; 
                } 
              // observe back
              	if (check_car_s <= car_s && car_s - check_car_s < sgap_back_leftlane ){               		
                   id_back_leftlane = i; 
                   sgap_back_leftlane = car_s - check_car_s;
                   v_back_leftlane = check_speed; 
                } 
            }
            // observe right lane
            if ( d < LW*(lane+2) && d > LW*lane+1){
              // observe front
              	if (check_car_s > car_s && check_car_s - car_s < sgap_front_rightlane ){               		
                   id_front_rightlane = i; 
                   sgap_front_rightlane = check_car_s - car_s;
                   v_front_rightlane = check_speed; 
                } 
              // observe back
              	if (check_car_s <= car_s && car_s - check_car_s < sgap_back_rightlane ){               		
                   id_back_rightlane = i; 
                   sgap_back_rightlane = car_s - check_car_s;
                   v_back_rightlane = check_speed; 
                } 
            }
          }
          
          
          // following front car
          
          if (sgap_front_samelane < 70.0 && sgap_front_samelane >= 50.0){
          		goal_v = v_front_samelane; 
          } else if (sgap_front_samelane < 50.0 && sgap_front_samelane >= 30.0){
          	 goal_v = v_front_samelane * 0.9;             
          } else if (sgap_front_samelane < 30.0){
          	 goal_v = std::min(29.5/MPS2MPH, v_front_samelane) ;
          }
          
          
          // wait for several steps before moving again
          if (phase == 1){ 
           		count = count - 1; 
            	if (count <= 0) {phase = 0; }
          }
          else {
              // change to left lane 
              if (lane == 1 && 
                  sgap_front_samelane < 70.0 && 
                  sgap_front_leftlane > 100.0 &&
                  sgap_back_leftlane > 50
                 ){
                    lane = 0;  
                	phase = 1; 
                	count = 500; 
                    goal_d = (2+4*lane);  
                    std::cout << "lane:" << lane << std::endl;
              }

              // back to center lane

              else if (lane == 0 && 
                  sgap_front_rightlane > 100.0 &&
                  sgap_back_rightlane > 50
                 ){
                    lane = 1;  
                	phase = 1;
                	count = 500; 
                    goal_d = (2+4*lane);   
                    std::cout << "lane:" << lane << std::endl;
              }
              // change to right lane 
              else if (lane == 1 && 
                  sgap_front_samelane < 70.0 && 
                  sgap_front_rightlane > 100.0 &&
                  sgap_back_rightlane > 50
                 ){
                    lane = 2;  
                	phase = 1; 
                	count = 500; 
                    goal_d = (2+4*lane);  
                    std::cout << "lane:" << lane << std::endl;
              }

              // back to center lane

              else if (lane == 2 && 
                  sgap_front_leftlane > 100.0 &&
                  sgap_back_leftlane > 50
                 ){
                    lane = 1;  
                	phase = 1;
                	count = 500; 
                    goal_d = (2+4*lane);   
                    std::cout << "lane:" << lane << std::endl;
              }
          }
          
          

          
          //-------------
          vector<double> ptsx;
          vector<double> ptsy; 
          
          double ref_x = car_x;
          double ref_y = car_y; 
          double ref_yaw = deg2rad(car_yaw); 
          
          if (prev_size < 2){
            
            double prev_car_x = car_x - cos(car_yaw); 
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
            
          } else {
          
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          
          }
          
          const double ext = 30.0; 
          vector<double> next_wp0 = getXY(goal_s, goal_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp1 = getXY(goal_s + ext*1, goal_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          vector<double> next_wp2 = getXY(goal_s + ext*2, goal_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          
          for (int i=0; i<ptsx.size(); i++){
           	double shift_x = ptsx[i] - ref_x; 
            double shift_y = ptsy[i] - ref_y; 
            
            // rotate 
            ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
            ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));           
            
          }
            
                       
          tk::spline s; 
          s.set_points(ptsx, ptsy); 
                       
              
          // velocity changes 
          int n_points = 50; 
          vector<double> velocities; 
           
  		  for(int i=0; i<n_points; i++){
            double velo = car_speed;
            double acc = MAXACC * 0.8;
            
            if (prev_size >1){
               if( i<prev_size-2){
                double x_next = previous_path_x[i+1]; 
                double x_curr = previous_path_x[i];
                double x_diff = x_next - x_curr;
                double y_next = previous_path_y[i+1]; 
                double y_curr = previous_path_y[i];
                double y_diff = y_next - y_curr;
                double past_v = sqrt(x_diff*x_diff + y_diff*y_diff)/dT;

                velocities.push_back(past_v); 

                } else {

                    if (velocities[i-1] < goal_v) { velo = velocities[i-1] + acc * dT ;}                   
                    else { 
                      velo = velocities[i-1] - acc * dT;
                      //std::cout << " i: " << i <<  " over speed:" <<  velocities[i-1] << std::endl;  
                    }

                  velocities.push_back(velo);                                
                  // std::cout << "i:" << i << " velo:" << velo << "  velocities[i]: " << velocities[i] << " goal v: "<< goal_v << std::endl;
                }  
            } else {
            	velocities.push_back(acc * dT);
            }
          }
            
          // position changes
          
          
          double x_add_on = 0.0;
          for(int i=0; i<n_points; i++){
           
            if (i<previous_path_x.size()){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);    
              
            } else {
           
              double x_point = x_add_on + velocities[i-1] * dT;
              double y_point = s(x_point); 
              x_add_on = x_point; 

              // rotate back
              double x_ref = x_point;
              double y_ref = y_point;
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
              x_point += ref_x; 
              y_point += ref_y;
            
              //std::cout << "after rorate: x_point:" << x_point << "     y_point:" << y_point << std::endl;

              // add points 
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
                    
            }
          }
        
          
              
     
          ///////////////////////////////////////////////////////////////////////////

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