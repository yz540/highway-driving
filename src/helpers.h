#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

int get_lane(int d){
  return (int)(d/4);
}

vector<double> get_predictions_from_sensor_fusion(vector<vector<double>> sensor_fusion, int prev_size, double car_s, int lane){
  bool car_left = false;
  bool car_front = false;
  bool car_right = false;
  double front_speed;
  for(int i = 0; i < sensor_fusion.size(); i++){
    double check_car_s = sensor_fusion[i][5];
    // filter the cars that are too far from the ego car
    if(abs(check_car_s - car_s) < 100){
      double check_car_d = sensor_fusion[i][6];
      int check_car_lane = get_lane(check_car_d);
      // check if the car in front is too close
      double check_car_vx = sensor_fusion[i][3];
      double check_car_vy = sensor_fusion[i][4];
      double check_car_speed = sqrt(pow(check_car_vx, 2) + pow(check_car_vy, 2));
      check_car_s = check_car_s + prev_size * 0.02 * check_car_speed;
      if(check_car_lane == lane){
        if((check_car_s > car_s) && (check_car_s < car_s + 30)){
          car_front = true;
          front_speed = check_car_speed; // m/s
        }
      } else {
        if((check_car_lane == lane - 1) && (check_car_s > car_s && (check_car_s - car_s) < 40 ||  check_car_s < car_s && (car_s - check_car_s ) < 30)|| lane == 0)
          car_left = true;
        if((check_car_lane == lane + 1) && (check_car_s > car_s && (check_car_s - car_s) < 40 ||  check_car_s < car_s && (car_s - check_car_s ) < 30)|| lane == 2)
          car_right = true;
      }
    }
  }

  return {car_left, car_right, car_front, front_speed};
}

vector<std::pair<int, double>> behaviour_planning(bool car_left, bool car_right, bool car_front, int lane, double ref_vel, double front_speed){
  vector<std::pair<int, double>> successor_lanes_ref_vels;
  if(car_front){
    successor_lanes_ref_vels.push_back(std::make_pair(lane, ref_vel - 5 * 0.02));
    if(!car_left)
      successor_lanes_ref_vels.push_back(std::make_pair(std::max(lane - 1, 0), std::min(49.5/2.24, ref_vel + 5 * 0.02)));
    if(!car_right)
      successor_lanes_ref_vels.push_back(std::make_pair(std::min(lane + 1, 2), std::min(49.5/2.24, ref_vel + 5 * 0.02)));
  }

  if(!car_front){
    successor_lanes_ref_vels.push_back(std::make_pair(lane, std::min(49.5/2.24, ref_vel + 8 * 0.02)));
  }
  return successor_lanes_ref_vels;
}

double calculate_cost(int lane, double car_speed, int successor_lane, double successor_ref_vel){

  double cost_lane_change = std::abs(lane - successor_lane)/3;
  // cost of acceleration  
  double acc = (successor_ref_vel - car_speed)/0.02;
  double cost_acc = 0;
  if(acc<-9.8 || acc >9.8)
    cost_acc = 1;
  else
    cost_acc = std::abs(acc)/9.8;

  //   cost of speed
  double target_speed = 49.5/2.24;
  double cost_speed = 0;
  if(successor_ref_vel < target_speed)
    cost_speed = 0.8 * (target_speed - successor_ref_vel) / target_speed;
  else{
    if(car_speed > 50)
      cost_speed = 1;
    else
      cost_speed = (successor_ref_vel - target_speed) / 0.5;
  }


  return 10*cost_lane_change + 10*cost_acc + 5*cost_speed;
}

vector<vector<double>> choose_next_trajectory(int lane, double ref_vel, double car_x, double car_y, double car_yaw, double car_s, double car_d, 
                                              const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y,
                                              vector<double> previous_path_x, vector<double> previous_path_y ){

  int prev_size = previous_path_x.size();
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  vector<double> ptsx;
  vector<double> ptsy;
  // find two points in the past
  if ( prev_size < 2 ) {    //Use two points thats makes path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);    

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }else{
    //Redefine the reference point to previous point
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y); 
  }
  // find another 3 points in the future
  for(int i = 1; i <4; i++){
    vector<double> next_wp = getXY(car_s + 30*i, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }
  // shift to car coordinates
  for(int i=0; i <ptsx.size(); i++){
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw));
    ptsy[i] = (shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw));
  }
  // build the spline based on the 5 points
  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  //For the smooth transition, we are adding previous path points
  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < 50 - prev_size; i++ ) {

    double N = target_dist/(0.02*ref_vel);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    //Rotate back to normal after rotating it earlier
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  vector<vector<double>> next_vals;
  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);

  return next_vals;
}
#endif  // HELPERS_H