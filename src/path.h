/*
 * trajectory.h
 *
 * A simple trajectory path generator for autonomous vehicles using a spline.
 *
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <iostream>
#include <vector>
#include <queue>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "debug.h"

using namespace std;

class path {
public:
  path(const double x_car, const double y_car, const double yaw_car,
          const double s_car, const double drive_lane, const vector<double> prev_path_x,
          const vector<double> prev_path_y, const double speed_car, const int path_size);

  virtual ~path();

  double deg2rad(double x);

  double rad2deg(double x);

  double distance(double x1, double y1, double x2, double y2);

  int closestwaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  vector<double> get_frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

  void initial_points();

  void makeSplinePts(const vector<double> map_waypoints_s, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y);

  void getSpline();

  double solveSpline(const double x);

  void getpathPts(vector<double> &next_x_vals, vector<double> &next_y_vals, const double ref_vel);

  void get_step(double &step, const double ref_vel, double &x_local, double &y_local, double &prev_x_local, double &prev_y_local);

  int prev_size;
  double max_vel;
  double waypoint_step;
  tk::spline spline;

  double car_x, car_y, car_yaw, car_s, car_speed;
  double global_x, global_y, global_yaw;

  int lane;

  vector<double> previous_path_x, previous_path_y;
  vector<double> ptsx, ptsy;
};


path::path(const double x_car, const double y_car, const double yaw_car,
        const double s_car, const double drive_lane, const vector<double> prev_path_x,
        const vector<double> prev_path_y, const double speed_car, const int path_size)
{
  car_x = x_car;
  max_vel = 50.0;
  car_y = y_car;
  car_yaw = yaw_car;
  car_s = s_car;
  lane = drive_lane;
  previous_path_x = prev_path_x;
  previous_path_y = prev_path_y;
  car_speed = speed_car;
  prev_size = path_size;

  waypoint_step = 25;
}

path::~path()
{

}

// For converting back and forth between radians and degrees.
double path::deg2rad(double x) { return x * M_PI / 180; }
double path::rad2deg(double x) { return x * 180 / M_PI; }

/*
 * Calculates the Euclidean distance between x and y coordinate points
 *
 * Args
 *      x1, coordinate point one x value
 *      y1, coordinate point one y value
 *      x2, coordinate point two x value
 *      y2, coordinate point two y value
 *
 * Return
 *     Euclidean distance
 */
double path::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int path::closestwaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
    double closestLen = 500000;
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
            double map_x = maps_x[i];
            double map_y = maps_y[i];
            double dist = path::distance(x,y,map_x,map_y);
            if(dist < closestLen)
            {
                    closestLen = dist;
                    closestWaypoint = i;
            }
    }
    return closestWaypoint;
}

int path::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int closestWaypoint = path::closestwaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > M_PI/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;
}


vector<double> path::get_frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = path::NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;

    LOGD();
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
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

    double frenet_d = path::distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = path::distance(center_x,center_y,x_x,x_y);
    double centerToRef = path::distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
            frenet_d *= -1;
    }

    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
      frenet_s += path::distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += path::distance(0,0,proj_x,proj_y);

    LOGD();
    return {frenet_s,frenet_d};

}


vector<double> path::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

  LOGD();

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

  LOGD();
	return {x,y};

}

void path::initial_points()
{
  // check if there are previous way points that can be used
  if(prev_size < 2){
    global_x = car_x;
    global_y = car_y;
    global_yaw = path::deg2rad(car_yaw);

    /* Convert back to car's reference frame */
    double shift_x = (global_x - cos(global_yaw)) - global_x;
    double shift_y = (global_y - sin(global_yaw)) - global_y;

    ptsx.push_back(shift_x*cos(0-global_yaw) - shift_y*sin(0-global_yaw));
    ptsy.push_back(shift_x*sin(0-global_yaw) + shift_y*cos(0-global_yaw));

    ptsx.push_back(0);
    ptsy.push_back(0);
  }
  else
  {
    global_x = previous_path_x[prev_size - 1];
    global_y = previous_path_y[prev_size - 1];
    double global_x_prev = previous_path_x[prev_size - 2];
    double global_y_prev = previous_path_y[prev_size - 2];

    global_yaw = atan2(global_y - global_y_prev, global_x - global_x_prev);

    double shift_x = global_x_prev - global_x;
    double shift_y = global_y_prev - global_y;

    ptsx.push_back(shift_x*cos(0-global_yaw) - shift_y*sin(0-global_yaw));
    ptsy.push_back(shift_x*sin(0-global_yaw) + shift_y*cos(0-global_yaw));

    ptsx.push_back(0);
    ptsy.push_back(0);
  }
}


void path::makeSplinePts(const vector<double> map_waypoints_s, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y)
{
  path::initial_points();
  double next_d = 2 + 4 * lane;
  for(int i=1; i <= 3; i++){
    vector<double> xy_pts = path::getXY(car_s+(i*waypoint_step), next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    /* Convert to cars reference frame */
    double shift_x = xy_pts[0] - global_x;
    double shift_y = xy_pts[1] - global_y;

    ptsx.push_back(shift_x*cos(0-global_yaw) - shift_y*sin(0-global_yaw));
    ptsy.push_back(shift_x*sin(0-global_yaw) + shift_y*cos(0-global_yaw));
  }
}

void path::getSpline()
{
  spline.set_points(ptsx, ptsy);
}


double path::solveSpline(const double x)
{
  return spline(x);
}

#define MAX_POINTS 50
void path::getpathPts(vector<double> &next_x_vals,
    vector<double> &next_y_vals,
    const double ref_vel)
{
  double x_local = 0; // the current x point being considered
  double y_local, prev_x_local, prev_y_local;

  // Store the unused old way points to create a smooth path transition
  for(int i=0; i < prev_size; i++){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double step;
  int next_size = next_x_vals.size();

  if(next_size < 2)
  {
    step = 0;
  }
  else
  {
    double step_x = next_x_vals[next_size - 1] - next_x_vals[next_size - 2];
    double step_y = next_y_vals[next_size - 1] - next_y_vals[next_size - 2];
    step = sqrt(step_x*step_x + step_y*step_y);

    prev_x_local = step_x;
    prev_y_local = step_y;
  }

  for(int i = 1; i < MAX_POINTS - prev_size; i++){

    path::get_step(step, ref_vel, x_local, y_local, prev_x_local, prev_y_local);

    double x_point = x_local * cos(global_yaw) - y_local * sin(global_yaw);
    double y_point = x_local * sin(global_yaw) + y_local * cos(global_yaw);

    next_x_vals.push_back(x_point + global_x);
    next_y_vals.push_back(y_point + global_y);
  }
}

void path::get_step(double &step, const double ref_vel,
        double &x_local, double &y_local, double &prev_x_local, double &prev_y_local)
{
  double acceleration = 0.0022;
  const double mile_to_meter = 0.44704;
  const double max_step = max_vel * mile_to_meter * 0.02;
  double ref_step = std::min<double>(ref_vel * mile_to_meter * 0.02, max_step);
  LOGD();

  if((car_speed > ref_vel))
  {
    step = std::max(step - acceleration, ref_step);
  }
  else if(car_speed < ref_vel)
  {
    step = std::min(step + acceleration, ref_step);
  }

  x_local += step;
  y_local = path::solveSpline(x_local);

  double diff_x = x_local - prev_x_local;
  double diff_y = y_local - prev_y_local;
  double diff_step = sqrt(diff_x*diff_x + diff_y*diff_y);
  int loop = 0;

  while(diff_step > max_step && loop < 6)
  {
    double error = std::max(max_step/diff_step, 0.97);

    x_local *= error; // decrease the x_step by the error amount
    y_local = path::solveSpline(x_local);

    diff_x = x_local - prev_x_local;
    diff_y = y_local - prev_y_local;
    diff_step = sqrt(diff_x*diff_x + diff_y*diff_y);

    loop++;
  }

  prev_x_local = x_local; // update the previous step points
  prev_y_local = y_local;

  LOGD();
}

#endif /* TRAJECTORY_H */
