#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <math.h>
#include <iostream>
#include <vector>
#include <queue>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "path.h"
#include "filter.h"
#include "debug.h"


typedef enum
{
  KEEP_GOING=0,
  PREPARE_FOR_LANE_CHANGE=1,
  TAIL_VEHICLE=2,
  GO_LEFT=3,
  GO_RIGHT=4,
}VEHICLE_STATE;

using namespace std;

class NextAction {
public:
  // class initialiser
  NextAction(const double max_speed);

  virtual ~NextAction();

  void setVehicleVariables(const double s_car, const double d_car, const double speed_car, const int path_size);

  int updateState(const vector<vector<double>> &sensor_fusion, double &ref_vel, int &state);

  double getCost(const double object_speed, const double object_car_s);

  /* Asses the situation by looking around */
  void lookAround(const vector<vector<double>> &sensor_fusion);

  double followSpeed();

private:
  double car_s, car_d, car_speed, max_vel; // vehicle parameters
  double relative_vel_cost, max_vel_cost, s_cost; // cost values parameters
  int lane, lane_left, lane_right; // current and neighboring lane values
  int prev_size; // number of previous trajectory points to use
  int filter_size; // Moving average filter size
  bool is_gap_left, is_gap_right; // Store if there is a merge gap present (true) or not (false)

  // Total cost of all objects in a lane and there position in front or behind the vehicle
  double left_front_cost, right_front_cost, left_back_cost, right_back_cost, current_lane_cost;

  // Action and tracking distance parameters for sensor fusion data
  double look_ahead_dist, action_ahead_dist, look_behind_dist, action_behind_dist, follow_dist;

  // add an offset to counter the sensor offset produced in the simulator which
  // predicts the vehicles position 15m ahead of the actual vehicles position.
  double sensor_offset;

  // Tracked object parameters
  struct vehicle {
    double distance_s;
    double speed;
  };
  MovingAverage left_filter, right_filter, current_filter; // moving average of each lanes cost
  vehicle center_front, left_front, right_front, left_back, right_back;
};

NextAction::NextAction(const double max_speed)
{
  car_s = 0; // Fernet s coordinate with respect to the vehicles reference frame
  car_d = 0;
  car_speed = 0;
  prev_size = 0;
  max_vel = max_speed;
  lane = -1;

  look_ahead_dist = 80.0; // distance in front of the vehicle that object will be tracked
  action_ahead_dist = 50.0; // distance in front of the vehicle that control actions will be performed
  look_behind_dist = 30.0; // distance behind the vehicle that object will be tracked
  action_behind_dist = 15.0; // distance behind the vehicle that control actions will be performed
  follow_dist = 25.0; // distance to follow a lead vehicle

  relative_vel_cost = 1; // cost of the difference between this vehicle and the objects velocity
  max_vel_cost = 1; // cost between the speed limit and the objects velocity
  s_cost = 100; // cost for the Fernet s distance between this vehicle and the object

  filter_size = 5; // number of past elements to calculate the moving average with
  left_filter.setSize(filter_size); // left lane moving average
  right_filter.setSize(filter_size); // right lane moving average
  current_filter.setSize(filter_size); // current vehicles lane moving average

  sensor_offset = 15.5;
}

NextAction::~NextAction()
{

}


#define NOMINAL_COST 1000
#define HIGH_COST 10000
#define VERY_HIGH_COST 20000

void NextAction::setVehicleVariables(const double s_car, const double d_car, const double speed_car, const int path_size)
{

  LOGD();
  car_s = s_car;
  car_d = d_car;
  car_speed = speed_car;
  prev_size = path_size;

  /* Make sure the lane number if valid */
  if(lane < 0){
    lane = car_d / 4;
  }

  lane_left = std::max(lane-1, 0);
  lane_right = std::min(lane+1, 2);
  left_front_cost = 0;
  left_back_cost = 0;
  current_lane_cost = 0;
  right_front_cost = 0;
  right_back_cost = 0;

  is_gap_left = true;
  is_gap_right = true;


  if(lane == 0){
    /* Left most lane. So dont merge into any traffic
     * Keep the cost to take left very high*/
    is_gap_left = false;
    left_front_cost = HIGH_COST;
    left_back_cost = HIGH_COST;
  } else if(lane == 2)
  {
    /* Right lane, so keep cost to take right very high */
    is_gap_right = false;
    right_front_cost = HIGH_COST;
    right_back_cost = HIGH_COST;
  }

  center_front.distance_s = NOMINAL_COST;
  center_front.speed = NOMINAL_COST;
  left_front.distance_s = NOMINAL_COST;
  left_front.speed = NOMINAL_COST;
  right_front.distance_s = NOMINAL_COST;
  right_front.speed = NOMINAL_COST;
  left_back.distance_s = -NOMINAL_COST;
  left_back.speed = NOMINAL_COST;
  right_back.distance_s = -NOMINAL_COST;
  right_back.speed = NOMINAL_COST;
  LOGD();
}

double NextAction::getCost(const double object_speed, const double object_car_s){
  const double distance_s = object_car_s - car_s + sensor_offset;

  // velocity cost between this vehicle and the object speed
  double cost = std::abs(car_speed - object_speed) * relative_vel_cost;

  LOGD();
  // velocity cost between the speed limit and the objects speed
  cost += std::abs(max_vel - object_speed) * max_vel_cost;
  // cost for the distance between this vehicle and the object
  cost += (1 - std::abs(distance_s) / look_ahead_dist) * s_cost;
  // cost for the distance between this vehicle and the object 1 second in the future
  cost += (std::abs((object_car_s + object_speed) - (car_s + car_speed) + sensor_offset) / look_ahead_dist) * s_cost;

  LOGD();
  return cost;
}

void NextAction::lookAround(const vector<vector<double>> &sensor_fusion)
{

  LOGD();
  for(int i=0; i < sensor_fusion.size(); i++){
    float d = sensor_fusion[i][6];
    double object_car_s = sensor_fusion[i][5];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double object_speed = sqrt(vx*vx + vy*vy);

    double distance_s = object_car_s - car_s + sensor_offset;

    if((d > (2+4*lane_left-1.8)) && (d < (2+4*lane_right+1.8)) && (distance_s > -look_behind_dist) && (distance_s < look_ahead_dist)){

      double cost = NextAction::getCost(object_speed, object_car_s);

      if((d < (2+4*lane+1.8)) && (d > (2+4*lane-1.8)) && (distance_s > 0)){
        current_lane_cost += cost;

        if(center_front.distance_s > distance_s){
          center_front.distance_s = distance_s;
          center_front.speed = object_speed;
        }
      } else if((d < (2+4*lane_left+2)) && (d > (2+4*lane_left-2))){
        if(distance_s > 0) { // is the object in front
          left_front_cost += cost;

          if(left_front.distance_s > distance_s){
            left_front.distance_s = distance_s;
            left_front.speed = object_speed;
          }
        } else {
          left_back_cost += cost;

          if(left_back.distance_s < distance_s){
            left_back.distance_s = distance_s;
            left_back.speed = object_speed;
          }
        }

        if((distance_s > -action_behind_dist) && (distance_s < follow_dist)){
          is_gap_left = false;
        }
      } else if((d < (2+4*lane_right+2)) && (d > (2+4*lane_right-2))){
        if(distance_s > 0) {
          right_front_cost += cost;

          if(right_front.distance_s > distance_s){
            right_front.distance_s = distance_s;
            right_front.speed = object_speed;
          }
        } else {
          right_back_cost += cost;

          if(right_back.distance_s < distance_s){
            right_back.distance_s = distance_s;
            right_back.speed = object_speed;
          }
        }

        if((distance_s > -action_behind_dist) && (distance_s < follow_dist)){
          is_gap_right = false;
        }
      }
    }
  }

  LOGD();
}


double NextAction::followSpeed()
{
  double ref_vel;
  LOGD();

  /* If vehicle ahead isn't moving, we should stop */
  if(((center_front.speed <= 0.3) && (center_front.distance_s < 5)) || (center_front.distance_s < 5)){
    ref_vel = 0;
  }

  if(center_front.distance_s < 15){
    ref_vel = center_front.speed/2; // slow down to increase the distance between the car in front
  } else if(center_front.distance_s < 10){
    ref_vel = center_front.speed/4; // slow a lot as we are too close to the car in front
  } else {
    ref_vel = center_front.speed; // slow down to the speed of the vehicle in front and follow it
  }

  LOGD();
  return ref_vel;
}

int NextAction::updateState(const vector<vector<double>> &sensor_fusion, double &ref_vel, int &state)
{
  double left_cost, right_cost; // total left and right lane costs

  LOGD();
  switch(state){
    case(KEEP_GOING):
      ref_vel = max_vel;

      // check the vehicles surrounds for objects
      NextAction::lookAround(sensor_fusion);

      // Check if there is a vehicle in front and in same lane
      if(center_front.distance_s < action_ahead_dist){
        state = PREPARE_FOR_LANE_CHANGE;
      }
      break;
    case(PREPARE_FOR_LANE_CHANGE):
      NextAction::lookAround(sensor_fusion);

      if(center_front.distance_s < follow_dist){
        state = TAIL_VEHICLE;
      } else {
        state = KEEP_GOING;
      }

      // get the averaged cost of being in each lane
      left_cost = left_filter.nextAverage(left_front_cost);
      right_cost = right_filter.nextAverage(right_front_cost);
      current_lane_cost = current_filter.nextAverage(current_lane_cost);

      if(left_filter.getSize() >= filter_size){ // only evaluate costs if the filter is full

        // Find the lane with the lowest cost and favour passing on the left
        if(is_gap_left && (left_cost <= right_cost) && (left_cost < current_lane_cost)){
          // change lanes to the left
          state = GO_LEFT;
          lane = std::max(lane - 1, 0);
        } else if(is_gap_right && (right_cost < left_cost) && (right_cost < current_lane_cost)){
          // change lanes to the right
          state = GO_RIGHT;
          lane = std::min(lane + 1, 2);
        }
      }
      break;
    case(TAIL_VEHICLE):
      NextAction::lookAround(sensor_fusion);

      // Check if the object in front is too close and slow down
      if(center_front.distance_s < follow_dist) {
        state = PREPARE_FOR_LANE_CHANGE;

        ref_vel = NextAction::followSpeed();
      } else {
        state = KEEP_GOING;
      }
      break;
    case(GO_LEFT):
      NextAction::lookAround(sensor_fusion);

      // update the vehicles speed to match the speed of the object in the left lane
      if((center_front.distance_s < follow_dist) || (left_front.distance_s < follow_dist)) {
        ref_vel = NextAction::followSpeed();
      } else {
        ref_vel = max_vel;
      }

      // only update the state if the vehicle is in the center of the lane
      if((car_d < (2+4*lane+0.5)) && (car_d > (2+4*lane-0.5))){
        state = KEEP_GOING;
      }

      left_filter.emptyQueue();
      right_filter.emptyQueue();
      current_filter.emptyQueue();
      break;
    case(GO_RIGHT):
      NextAction::lookAround(sensor_fusion);

      if((center_front.distance_s < follow_dist) || (right_front.distance_s < follow_dist)) {
        ref_vel = NextAction::followSpeed();
      } else {
        ref_vel = max_vel;
      }

      // only update the state if the vehicle is in the center of the lane
      if((car_d < (2+4*lane+0.5)) && (car_d > (2+4*lane-0.5))){
        state = KEEP_GOING;
      }

      left_filter.emptyQueue();
      right_filter.emptyQueue();
      current_filter.emptyQueue();
      break;
    default:
      state = KEEP_GOING;
  }

  state = state; // set the state for the next time around

  LOGD();
  return lane;
}

#endif /* STATE_MACHINE_H */
