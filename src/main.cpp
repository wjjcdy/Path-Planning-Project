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

double goal_distance_cost(int goal_lane, int intended_lane, int final_lane, 
                          double distance_to_goal) 
                          {
                            
                          }

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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int lane_big_change = 0;

          int lane = 1;
          static int lane_curr = 1;
          int lane_next = lane_curr;

          double ref_vel = 49;
          const double vel_max = 49;
          const double vel_min = 10.0;
          static double ref_vel_curr = 0;

          int prev_size =  previous_path_x.size();

          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          //check car is collision the other car or not in future
          double front_car_vel  = 1000;
          int lane_check_collision_flag[3] = {0,0,0}; 
          int middle_collision = 0;
          for (int offset=0;offset < 3;++offset)
          {
              int lane_check = offset;
              if(lane_check>=0 && lane_check<3)
              {
                for(int i=0;i<sensor_fusion.size();++i)
                {
                  double traffic_car_d = sensor_fusion[i][6];
                  if((traffic_car_d > lane_check*4) && (traffic_car_d < (lane_check+1)*4))
                  {              
                    double traffic_car_x_vel = sensor_fusion[i][3];
                    double traffic_car_y_vel = sensor_fusion[i][4];
                    double traffic_car_s = sensor_fusion[i][5];
                    double traffic_vel = sqrt(traffic_car_x_vel*traffic_car_x_vel + traffic_car_y_vel*traffic_car_y_vel);
                    double next_traffic_s = traffic_car_s + prev_size * 0.02 * traffic_vel; 

                    if((lane_check == lane_curr) && next_traffic_s > car_s && next_traffic_s < (car_s + 30))
                    {
                      lane_check_collision_flag[lane_check] = 1;
                      if(traffic_vel < front_car_vel)
                      {
                        front_car_vel = traffic_vel;
                      }
                    }
                    else if(next_traffic_s > (car_s-8) && next_traffic_s < (car_s + 30))
                    {
                      lane_check_collision_flag[lane_check] = 1;
                    }

                    if(lane_check == 1 && next_traffic_s > (car_s-8) && next_traffic_s < (car_s + 15))
                    {
                      middle_collision = 1;
                    }
                  }
                }
              }
          }
          // if(lane_curr==0)
          //   {
          //     lane_check_collision_flag[2] = 1;
          //   }
          //   else if(lane_curr == 2)
          //   {
          //     lane_check_collision_flag[0] = 1;
          //   }
          

          // printf("curr_lane=%d, lane0=%d,lane1=%d,lane2=%d,lane1_pass = %d\n",lane_curr,
          //                                                     lane_check_collision_flag[0],
          //                                                     lane_check_collision_flag[1],
          //                                                     lane_check_collision_flag[2],
          //                                                     middle_collision);

          int change_lane_flag = 0;
          if(lane_check_collision_flag[0] == 1 &&
             lane_check_collision_flag[1] == 1 &&
             lane_check_collision_flag[2] == 1)
          {
            ref_vel = front_car_vel * 2.24;
            lane_next = lane_curr;
          } 
          // else if (lane_check_collision_flag[1] == 0)
          // {
          //   lane_next = 1;
          // }
          else
          {
            if(lane_check_collision_flag[lane_curr] == 1)
            {
              if (lane_curr == 0)       //left lane
              {
                if(lane_check_collision_flag[1] == 0)
                {
                  lane_next = 1;
                }
                else if (lane_check_collision_flag[2] == 0 && middle_collision == 0)
                {
                  lane_next = 1;
                  lane_big_change = 1;
                  ref_vel = 40;
                }
                else
                {
                  ref_vel = front_car_vel * 2.24;
                  lane_next = lane_curr;
                }
              }
              else if (lane_curr == 2)  //right lane
              {
                if(lane_check_collision_flag[1] == 0)
                {
                  lane_next = 1;
                }
                else if (lane_check_collision_flag[0] == 0 && middle_collision == 0)
                {
                  lane_next = 1;
                  lane_big_change = 1;
                  ref_vel = 40;
                }
                else
                {
                  ref_vel = front_car_vel * 2.24;
                  lane_next = lane_curr;
                }
              }
              else                      //middle lane
              {
                if(lane_check_collision_flag[0] == 0)
                {
                  lane_next = 0;
                }
                else
                {
                  lane_next = 2;
                } 
              }
            }
          }
          
          //printf("lane_next = %d,change_lane_flag=%d\n",lane_next,change_lane_flag);
        
          vector <double> ptsx;
          vector <double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;

          if(prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

            double err_x = ref_x - ref_x_prev;
            double err_y = ref_y - ref_y_prev;
            ref_yaw = atan2(err_y,err_x);
          }
          
         
          static int cout =0;
          if (lane_next != lane_curr)
          {
            if(cout < 20)
            {
              lane_next = lane_curr;
             // ref_vel = front_car_vel * 2.24;
              cout++;
              printf("cout=%d\n",cout);
            }
            else
            {
              printf("change lane\n");
              cout =0;
            }
          }
          else
          {
            cout++;
            printf("keep cout=%d\n",cout);
          }
          
          

          vector <double> next_pw0 = getXY(car_s+30, 2+lane_next*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> next_pw1 = getXY(car_s+60, 2+lane_next*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> next_pw2 = getXY(car_s+90, 2+lane_next*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          // if(lane_big_change)   //when lane change beyond 1,need add middle lane cross point
          // {
          //   vector <double> next_middle = getXY(car_s+25, 2+1*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //   ptsx.push_back(next_middle[0]);
          //   ptsy.push_back(next_middle[1]);
          //   printf("**********add middle lane cross point************8\n");
          //   next_pw0 = getXY(car_s+50, 2+lane_next*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // }
          ptsx.push_back(next_pw0[0]);
          ptsx.push_back(next_pw1[0]);
          ptsx.push_back(next_pw2[0]);

          ptsy.push_back(next_pw0[1]);
          ptsy.push_back(next_pw1[1]);
          ptsy.push_back(next_pw2[1]);

          lane_curr =  lane_next;
          for(int i=0;i<ptsx.size();++i)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw); 
          }


          if(ref_vel_curr < ref_vel)
          {
            ref_vel_curr += 0.8;
          }
          else if(ref_vel_curr > ref_vel)
          {
            ref_vel_curr -= 0.8;
          }
          if (ref_vel_curr > vel_max)
          {
            ref_vel_curr = vel_max;
          }
          
          tk::spline s;
          s.set_points(ptsx,ptsy);

          double target_x = 30.0;
          double target_y = s(target_x);

          double distance = sqrt(target_x * target_x + target_y * target_y);
          int N = distance / (0.02 * ref_vel_curr/2.24 );   // MPH transform m/s
          
          for(int i=0;i<prev_size;++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          double add_x = 0;
          for(int i=1;i<(50 - prev_size);++i)
          {
            double px = add_x + target_x / N;
            double py = s(px);
            add_x = px;
            
            double x_ref = px;
            double y_ref = py;
            px = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            py = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            px += ref_x;
            py += ref_y;

            next_x_vals.push_back(px);
            next_y_vals.push_back(py);
          } 

        

          //end TODO
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